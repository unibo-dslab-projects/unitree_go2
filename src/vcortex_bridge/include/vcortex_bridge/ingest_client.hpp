#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace vcortex_bridge {
    // the websocket to the vcortex backend's /ingest, no ROS inside.
    // its own thread connects, sends the hello, waits for the welcome, then sends frames;
    // after any error it starts again from the connect.
    class IngestClient {
    public:
        using Log = std::function<void(const std::string &)>;

        // log is called from the client thread
        IngestClient(std::string host, std::string port, std::string hello, Log log);
        ~IngestClient();

        // 0 while not connected. A new number after each welcome and each resync:
        // both mean the backend needs a full map before any delta.
        std::uint64_t session() const;

        // dropped unless session is still the current one, so a frame never lands on another map
        void send(std::uint64_t session, std::string frame);

    private:
        using WebSocket = boost::beast::websocket::stream<boost::beast::tcp_stream>;

        void connect();
        void on_connect();
        void on_handshake();
        void on_welcome();
        void read_next();
        void write_next();
        void fail(const std::string &step, boost::beast::error_code error);

        const std::string host_;
        const std::string port_;
        const std::string hello_;
        const Log log_;

        boost::asio::io_context io_;
        boost::asio::ip::tcp::resolver resolver_{io_};
        boost::asio::steady_timer retry_timer_{io_};
        std::optional<WebSocket> socket_;  // a closed websocket cannot open again: a new one per connect
        boost::beast::flat_buffer reply_;

        // touched only on the client thread
        std::deque<std::string> queue_;
        std::uint64_t last_session_ = 0;
        bool failed_ = false;  // a broken link fails both the read and the write: reconnect once
        std::chrono::seconds retry_delay_{1};

        std::atomic<std::uint64_t> session_{0};
        std::thread thread_;
    };
}
