#include "vcortex_bridge/ingest_client.hpp"

#include <algorithm>
#include <utility>

#include <boost/asio/buffer.hpp>
#include <boost/asio/post.hpp>

namespace vcortex_bridge {
    namespace {
        namespace beast = boost::beast;
        namespace websocket = beast::websocket;
        using tcp = boost::asio::ip::tcp;

        constexpr std::chrono::seconds kConnectTimeout{5};
        // the backend answers our pings; silent this long means the wi-fi is gone,
        // where tcp alone takes minutes to notice
        constexpr std::chrono::seconds kIdleTimeout{5};
        constexpr std::chrono::seconds kFirstRetry{1};
        constexpr std::chrono::seconds kLastRetry{10};
    } // namespace

    IngestClient::IngestClient(std::string host, std::string port, std::string hello, Log log)
        : host_(std::move(host)), port_(std::move(port)), hello_(std::move(hello)), log_(std::move(log))
    {
        connect();
        thread_ = std::thread([this]() { io_.run(); });
    }

    IngestClient::~IngestClient()
    {
        io_.stop();
        thread_.join();
    }

    std::uint64_t IngestClient::session() const
    {
        return session_;
    }

    void IngestClient::send(std::uint64_t session, std::string frame)
    {
        boost::asio::post(io_, [this, session, frame = std::move(frame)]() mutable {
            if (session != session_) {
                return;
            }
            queue_.push_back(std::move(frame));
            if (queue_.size() == 1) {
                write_next(); // otherwise the write already running takes it next
            }
        });
    }

    void IngestClient::connect()
    {
        failed_ = false;
        queue_.clear();
        reply_.clear();
        socket_.emplace(io_);
        resolver_.async_resolve(host_, port_, [this](beast::error_code error, tcp::resolver::results_type endpoints) {
            if (error) {
                return fail("resolve", error);
            }
            beast::get_lowest_layer(*socket_).expires_after(kConnectTimeout);
            beast::get_lowest_layer(*socket_).async_connect(endpoints, [this](beast::error_code error, tcp::endpoint) {
                if (error) {
                    return fail("connect", error);
                }
                on_connect();
            });
        });
    }

    void IngestClient::on_connect()
    {
        // from here the websocket keeps its own timeouts
        beast::get_lowest_layer(*socket_).expires_never();
        websocket::stream_base::timeout timeout{};
        timeout.handshake_timeout = kConnectTimeout;
        timeout.idle_timeout = kIdleTimeout;
        timeout.keep_alive_pings = true;
        socket_->set_option(timeout);

        socket_->async_handshake(host_ + ":" + port_, "/ingest", [this](beast::error_code error) {
            if (error) {
                return fail("handshake", error);
            }
            on_handshake();
        });
    }

    // a refused hello (bad token, another source connected) comes back as a close with a reason
    void IngestClient::on_handshake()
    {
        socket_->text(true);
        socket_->async_write(boost::asio::buffer(hello_), [this](beast::error_code error, std::size_t) {
            if (error) {
                return fail("hello", error);
            }
            socket_->async_read(reply_, [this](beast::error_code error, std::size_t) {
                if (error) {
                    return fail("welcome", error);
                }
                on_welcome();
            });
        });
    }

    void IngestClient::on_welcome()
    {
        reply_.clear();
        socket_->binary(true);
        retry_delay_ = kFirstRetry;
        session_ = ++last_session_;
        log_("connected to " + host_ + ":" + port_);
        read_next();
    }

    // the backend only ever sends resync now; reading also answers its pings and sees its close
    void IngestClient::read_next()
    {
        socket_->async_read(reply_, [this](beast::error_code error, std::size_t) {
            if (error) {
                return fail("read", error);
            }
            if (beast::buffers_to_string(reply_.data()).find("resync") != std::string::npos) {
                session_ = ++last_session_;
                // keep only the frame being written: the rest belongs to the old map
                if (queue_.size() > 1) {
                    queue_.erase(queue_.begin() + 1, queue_.end());
                }
                log_("backend asked for a full map");
            }
            reply_.clear();
            read_next();
        });
    }

    void IngestClient::write_next()
    {
        socket_->async_write(boost::asio::buffer(queue_.front()), [this](beast::error_code error, std::size_t) {
            if (error) {
                return fail("write", error);
            }
            queue_.pop_front();
            if (!queue_.empty()) {
                write_next();
            }
        });
    }

    void IngestClient::fail(const std::string &step, beast::error_code error)
    {
        if (failed_) {
            return;
        }
        failed_ = true;
        session_ = 0;

        std::string message = step + " failed: " + error.message();
        if (error == websocket::error::closed) {
            const auto &reason = socket_->reason().reason;
            message += " (" + std::string(reason.data(), reason.size()) + ")";
        }
        log_(message + ", retry in " + std::to_string(retry_delay_.count()) + " s");

        // also ends the other pending operation, which then lands here and returns
        beast::get_lowest_layer(*socket_).close();
        retry_timer_.expires_after(retry_delay_);
        retry_timer_.async_wait([this](beast::error_code) { connect(); });
        retry_delay_ = std::min(retry_delay_ * 2, kLastRetry);
    }
}
