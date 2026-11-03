#pragma once

#include "vcortex_bridge/ingest_client.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <go2_mapping_msgs/msg/map_delta.hpp>
#include <go2_mapping_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vcortex_bridge {
    // one robot pose, as the backend stores it
    struct Pose {
        std::int64_t time_ns;
        float x, y, z, yaw;
    };

    // carries map_builder's map and poses to the vcortex backend.
    // every new session starts with a full map from GetMap, then the deltas that follow it.
    class BridgeNode : public rclcpp::Node {
    public:
        BridgeNode();

    private:
        void request_map_if_needed();
        void on_map(std::uint64_t session, const go2_mapping_msgs::srv::GetMap::Response &map);
        void on_delta(const go2_mapping_msgs::msg::MapDelta::ConstSharedPtr &delta);
        void on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose);
        void send_poses();
        bool synced() const;

        std::string backend_host_;
        std::string backend_port_;
        std::string token_;
        double voxel_size_;

        // made once the first map names its frame: the hello needs the frame
        std::unique_ptr<IngestClient> client_;
        // session that has the map; deltas and poses go only to it
        std::uint64_t synced_session_ = 0;
        std::uint32_t run_id_ = 0;
        std::uint64_t seq_ = 0; // last delta the backend has
        std::optional<std::chrono::steady_clock::time_point> map_requested_;
        // a delta can overtake the map answer it follows: keep it until the map is sent
        std::vector<go2_mapping_msgs::msg::MapDelta::ConstSharedPtr> deltas_while_waiting_;
        std::vector<Pose> poses_;

        rclcpp::Subscription<go2_mapping_msgs::msg::MapDelta>::SharedPtr delta_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Client<go2_mapping_msgs::srv::GetMap>::SharedPtr get_map_;
        rclcpp::TimerBase::SharedPtr map_timer_;
        rclcpp::TimerBase::SharedPtr pose_timer_;
    };
}
