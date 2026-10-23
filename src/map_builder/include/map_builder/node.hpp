#pragma once

#include "map_builder/voxel_map.hpp"
#include "map_builder/trajectory.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <go2_mapping_msgs/msg/map_delta.hpp>
#include <go2_mapping_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace map_builder {
    // collect Point-LIO's scans into a voxel map and its odometry into a trajectory
    // offering the map live as deltas, snapshots and on disk.
    class MapBuilderNode : public rclcpp::Node {
    public:
        MapBuilderNode();

    private:
        void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
        void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
        void publish_delta();
        void publish_pose();
        void publish_full_map();
        void check_single_instance();
        void on_get_map(
            const std::shared_ptr<go2_mapping_msgs::srv::GetMap::Request>& request,
            std::shared_ptr<go2_mapping_msgs::srv::GetMap::Response>& response
        );
        void on_save(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
            std::shared_ptr<std_srvs::srv::Trigger::Response>& response
        );

        VoxelMap map_;
        Trajectory trajectory_;
        std::string output_dir_;

        std::uint32_t run_id_ = 0;
        std::uint64_t seq_ = 0;
        std::string frame_id_;
        std::int64_t first_stamp_ns_ = 0;
        builtin_interfaces::msg::Time newest_stamp_;
        std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
        bool pose_published_ = false;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<go2_mapping_msgs::msg::MapDelta>::SharedPtr delta_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Service<go2_mapping_msgs::srv::GetMap>::SharedPtr get_map_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
        rclcpp::TimerBase::SharedPtr delta_timer_;
        rclcpp::TimerBase::SharedPtr pose_timer_;
        rclcpp::TimerBase::SharedPtr map_timer_;
        rclcpp::TimerBase::SharedPtr startup_timer_;
    };
}
