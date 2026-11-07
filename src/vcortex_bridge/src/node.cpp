#include "vcortex_bridge/node.hpp"

#include <cinttypes>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace vcortex_bridge {
    namespace {
        using geometry_msgs::msg::PoseStamped;
        using go2_mapping_msgs::msg::MapDelta;
        using go2_mapping_msgs::srv::GetMap;

        // the ingest format, as the backend's protocol/ingest.hpp defines it
        constexpr int kProtocol = 2;
        constexpr std::uint8_t kMapFrame = 1;
        constexpr std::uint8_t kDeltaFrame = 2;
        constexpr std::uint8_t kPosesFrame = 3;
        constexpr std::size_t kHeaderSize = 16;
        constexpr std::size_t kVoxelRecordSize = 20;
        constexpr std::size_t kPoseRecordSize = 24;

        constexpr std::chrono::milliseconds kMapCheckPeriod{200};
        // an answer that never comes (map_builder restarted) must not block the next request
        constexpr std::chrono::seconds kMapTimeout{5};

        // the robot and the laptop are both little-endian, like the format: a plain copy
        template <typename T>
        void put(std::string &bytes, std::size_t at, T value)
        {
            std::memcpy(&bytes[at], &value, sizeof value);
        }

        // header filled in, records still zero
        std::string empty_frame(
            std::uint8_t kind, std::uint32_t run_id, std::uint64_t seq, std::size_t count, std::size_t record_size)
        {
            std::string bytes(kHeaderSize + count * record_size, '\0');
            put(bytes, 0, kind);
            put(bytes, 4, run_id);
            put(bytes, 8, static_cast<std::uint32_t>(seq)); // the backend keeps 32 bits
            put(bytes, 12, static_cast<std::uint32_t>(count));
            return bytes;
        }

        std::string voxel_frame(
            std::uint8_t kind, std::uint32_t run_id, std::uint64_t seq, const sensor_msgs::msg::PointCloud2 &cloud)
        {
            const std::size_t count = std::size_t{cloud.width} * cloud.height;
            std::string bytes = empty_frame(kind, run_id, seq, count, kVoxelRecordSize);
            sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
            sensor_msgs::PointCloud2ConstIterator<std::uint32_t> num_scan_seen(cloud, "num_scan_seen");
            for (std::size_t at = kHeaderSize; at < bytes.size(); at += kVoxelRecordSize) {
                put(bytes, at, *x);
                put(bytes, at + 4, *y);
                put(bytes, at + 8, *z);
                put(bytes, at + 12, *intensity);
                put(bytes, at + 16, *num_scan_seen);
                ++x, ++y, ++z, ++intensity, ++num_scan_seen;
            }
            return bytes;
        }

        std::string poses_frame(std::uint32_t run_id, const std::vector<Pose> &poses)
        {
            std::string bytes = empty_frame(kPosesFrame, run_id, 0, poses.size(), kPoseRecordSize);
            std::size_t at = kHeaderSize;
            for (const Pose &pose : poses) {
                put(bytes, at, pose.time_ns);
                put(bytes, at + 8, pose.x);
                put(bytes, at + 12, pose.y);
                put(bytes, at + 16, pose.z);
                put(bytes, at + 20, pose.yaw);
                at += kPoseRecordSize;
            }
            return bytes;
        }

        // no escaping: frame_id and token must not hold quotes or backslashes
        std::string hello(const std::string &frame_id, double voxel_size, const std::string &token)
        {
            return "{\"type\":\"hello\",\"protocol\":" + std::to_string(kProtocol) +
                ",\"source\":\"unitree_go2\",\"frame_id\":\"" + frame_id +
                "\",\"voxel_size\":" + std::to_string(voxel_size) +
                ",\"token\":\"" + token + "\"}";
        }

        float yaw_of(const geometry_msgs::msg::Quaternion &q)
        {
            return static_cast<float>(std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        }
    } // namespace

    BridgeNode::BridgeNode()
        : Node("vcortex_bridge"),
        backend_host_(declare_parameter<std::string>("backend_host", "localhost")),
        backend_port_(std::to_string(declare_parameter<int>("backend_port", 9002))),
        token_(declare_parameter<std::string>("token", "")),
        voxel_size_(declare_parameter<double>("voxel_size", 0.05))
    {
        const double pose_period = declare_parameter<double>("pose_period", 0.5);
        if (pose_period <= 0.0) {
            throw std::invalid_argument("pose_period must be positive");
        }

        // same depth as map_builder's publisher, so a short stall here loses no delta
        delta_sub_ = create_subscription<MapDelta>(
            "map_builder/delta",
            rclcpp::QoS(rclcpp::KeepLast(25)).reliable(),
            [this](MapDelta::ConstSharedPtr delta) { on_delta(delta); }
        );
        pose_sub_ = create_subscription<PoseStamped>(
            "map_builder/pose",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](PoseStamped::ConstSharedPtr pose) { on_pose(pose); }
        );
        get_map_ = create_client<GetMap>("map_builder/get_map");

        map_timer_ = create_wall_timer(kMapCheckPeriod, [this]() { request_map_if_needed(); });
        pose_timer_ = create_wall_timer(std::chrono::duration<double>(pose_period), [this]() { send_poses(); });

        RCLCPP_INFO(
            get_logger(),
            "backend %s:%s, voxel size %.3f m, poses every %.2f s, token %s",
            backend_host_.c_str(), backend_port_.c_str(), voxel_size_, pose_period, token_.empty() ? "off" : "on"
        );
    }

    void BridgeNode::request_map_if_needed()
    {
        // with no client yet the map is still asked for: its frame goes into the hello
        const std::uint64_t session = client_ ? client_->session() : 0;
        if (client_ && (session == 0 || session == synced_session_)) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (map_requested_ && now - *map_requested_ < kMapTimeout) {
            return;
        }
        if (!get_map_->service_is_ready()) {
            return;
        }

        map_requested_ = now;
        deltas_while_waiting_.clear();
        get_map_->async_send_request(
            std::make_shared<GetMap::Request>(),
            [this, session](rclcpp::Client<GetMap>::SharedFuture answer) { on_map(session, *answer.get()); }
        );
    }

    void BridgeNode::on_map(std::uint64_t session, const GetMap::Response &map)
    {
        map_requested_.reset();
        const std::string &frame_id = map.points.header.frame_id;
        if (frame_id.empty()) {
            return; // no scan yet: the next check asks again
        }
        if (!client_) {
            client_ = std::make_unique<IngestClient>(
                backend_host_, backend_port_, hello(frame_id, voxel_size_, token_),
                [this](const std::string &line) { RCLCPP_INFO(get_logger(), "%s", line.c_str()); }
            );
            return;
        }
        if (session != client_->session()) {
            return; // the link changed meanwhile: the next check asks again
        }

        run_id_ = map.run_id;
        seq_ = map.seq;
        client_->send(session, voxel_frame(kMapFrame, run_id_, seq_, map.points));
        synced_session_ = session;
        RCLCPP_INFO(
            get_logger(), "sent map: %u voxels, run_id %u, seq %" PRIu64,
            map.points.width * map.points.height, run_id_, seq_
        );

        const auto waiting = std::move(deltas_while_waiting_);
        deltas_while_waiting_.clear();
        for (const auto &delta : waiting) {
            on_delta(delta);
        }
    }

    void BridgeNode::on_delta(const MapDelta::ConstSharedPtr &delta)
    {
        if (!synced()) {
            if (map_requested_) {
                deltas_while_waiting_.push_back(delta);
            }
            return;
        }
        // GetMap first sends its pending points as a delta, so the map already holds that one
        if (delta->run_id == run_id_ && delta->seq <= seq_) {
            return;
        }
        seq_ = delta->seq;
        client_->send(synced_session_, voxel_frame(kDeltaFrame, delta->run_id, delta->seq, delta->points));
    }

    void BridgeNode::on_pose(const PoseStamped::ConstSharedPtr &pose)
    {
        if (!synced()) {
            return;
        }
        const auto &p = pose->pose.position;
        poses_.push_back(Pose{
            rclcpp::Time(pose->header.stamp).nanoseconds(),
            static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
            yaw_of(pose->pose.orientation),
        });
    }

    void BridgeNode::send_poses()
    {
        if (synced() && !poses_.empty()) {
            client_->send(synced_session_, poses_frame(run_id_, poses_));
        }
        poses_.clear();
    }

    bool BridgeNode::synced() const
    {
        return client_ && synced_session_ != 0 && synced_session_ == client_->session();
    }
}
