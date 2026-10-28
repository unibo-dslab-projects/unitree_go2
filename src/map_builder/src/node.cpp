#include "map_builder/node.hpp"
#include "map_builder/cloud_msg.hpp"
#include "map_builder/map_io.hpp"

#include <cinttypes>
#include <cstdint>
#include <filesystem>
#include <random>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

namespace map_builder {
    namespace {
        constexpr int kInputQueueSize = 100;
        constexpr int kDeltaQueueSize = 25;
        constexpr int kLogPeriodMs = 2'000;
        constexpr std::chrono::seconds kStartupCheckDelay{2};

        rcl_interfaces::msg::ParameterDescriptor read_only()
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            return descriptor;
        }

        double declare_rate(rclcpp::Node &node, const std::string &name, double default_value, bool zero_allowed)
        {
            const double rate = node.declare_parameter<double>(name, default_value, read_only());
            if (rate < 0.0 || (rate == 0.0 && !zero_allowed)) {
                throw std::invalid_argument("Rate must be positive");
            }
            return rate;
        }

        std::chrono::nanoseconds period_of(double rate_hz)
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(1) / rate_hz);
        }

        std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time &stamp)
        {
            return rclcpp::Time(stamp).nanoseconds();
        }
    } // namespace

    MapBuilderNode::MapBuilderNode()
        : Node("map_builder"),
        map_(declare_parameter<double>("voxel_size", 0.05, read_only())) // no default constructor
    {
        const double delta_rate_hz = declare_rate(*this, "delta_rate_hz", 2.0, false);
        const double pose_rate_hz = declare_rate(*this, "pose_rate_hz", 10.0, false);
        const double full_map_rate_hz = declare_rate(*this, "full_map_rate_hz", 0.0, true);
        output_dir_ = declare_parameter<std::string>("output_dir", "", read_only());
        save_on_exit_ = declare_parameter<bool>("save_on_exit", false, read_only());

        run_id_ = std::random_device{}();

        const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(kInputQueueSize)).reliable();

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud",
            input_qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                on_cloud(message);
            }
        );
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            input_qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                on_odometry(message);
            }
        );
        delta_pub_ = create_publisher<go2_mapping_msgs::msg::MapDelta>(
            "~/delta",
            rclcpp::QoS(rclcpp::KeepLast(kDeltaQueueSize)).reliable()
        );
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/pose",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
        );
        get_map_srv_ = create_service<go2_mapping_msgs::srv::GetMap>(
            "~/get_map",
            [this](const std::shared_ptr<go2_mapping_msgs::srv::GetMap::Request> request,
                std::shared_ptr<go2_mapping_msgs::srv::GetMap::Response> response) {
                on_get_map(request, response);
            }
        );
        save_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/save",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                on_save(request, response);
            }
        );

        delta_timer_ = create_wall_timer(period_of(delta_rate_hz), [this]() { publish_delta(); });
        pose_timer_ = create_wall_timer(period_of(pose_rate_hz), [this]() { publish_pose(); });

        if (full_map_rate_hz > 0.0) {
            // transient local: a viewer that connects later still gets the last map
            cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                "~/full_map",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
            );
            map_timer_ = create_wall_timer(period_of(full_map_rate_hz), [this]() { publish_full_map(); });
        }

        startup_timer_ = create_wall_timer(kStartupCheckDelay, [this]() { check_single_instance(); });
        RCLCPP_INFO(
            get_logger(),
            "voxel size %.3f m, delta %.1f Hz, pose %.1f Hz, full map %.1f Hz, run_id %u, output_dir %s",
            map_.voxel_size(), delta_rate_hz, pose_rate_hz, full_map_rate_hz, run_id_, output_dir_.c_str()
        );
    }

    void MapBuilderNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message)
    {
        std::vector<Point> points;

        try {
            points = from_message(*message);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to convert cloud: %s", e.what());
            return;
        }

        // the first scan fixes the map frame and the name of the save folder
        if (frame_id_.empty()) {
            frame_id_ = message->header.frame_id;
            first_stamp_ns_ = to_nanoseconds(message->header.stamp);
            RCLCPP_INFO(get_logger(), "First scan: frame_id=%s, stamp=%" PRId64 " ns", frame_id_.c_str(), first_stamp_ns_);
        } else if (message->header.frame_id != frame_id_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "Frame ID mismatch: expected %s, got %s", frame_id_.c_str(), message->header.frame_id.c_str());
            return;
        }

        const std::size_t added = map_.insert(points);
        newest_stamp_ = message->header.stamp;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "Voxels %zu, added %zu points from cloud", map_.size(), added);
    }

    void MapBuilderNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message)
    {
        const auto &position = message->pose.pose.position;
        const auto &orientation = message->pose.pose.orientation;

        trajectory_.add(Pose {
            to_nanoseconds(message->header.stamp),
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        });

        geometry_msgs::msg::PoseStamped pose;
        pose.header = message->header;
        pose.pose = message->pose.pose;
        latest_pose_ = pose;
        pose_published_ = false;
    }

    void MapBuilderNode::publish_delta()
    {
        std::vector<Point> points = map_.last_voxel_points_added();
        if (points.empty()) {
            return;
        }
        std_msgs::msg::Header header;
        header.stamp = newest_stamp_;
        header.frame_id = frame_id_;

        go2_mapping_msgs::msg::MapDelta delta;
        delta.header = header;
        delta.run_id = run_id_;
        delta.seq = ++seq_;
        delta.points = to_message(points, header);
        delta_pub_->publish(delta);
    }

    void MapBuilderNode::publish_pose()
    {
        if (!latest_pose_ || pose_published_) {
            return;
        }
        pose_pub_->publish(*latest_pose_);
        pose_published_ = true;
    }

    void MapBuilderNode::publish_full_map()
    {
        if (!map_.size()) {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = newest_stamp_;
        header.frame_id = frame_id_;
        cloud_pub_->publish(to_message(map_.all(), header));
    }

    void MapBuilderNode::check_single_instance()
    {
        startup_timer_->cancel(); // run once

        // the count includes this node own's publisher
        const std::size_t pubs = count_publishers(delta_pub_->get_topic_name());
        if (pubs > 1) {
            RCLCPP_FATAL(
                get_logger(),
                "%zu publishers detected on topic %s: another map builder is running",
                pubs,
                delta_pub_->get_topic_name()
            );
            rclcpp::shutdown();
        }
    }

    void MapBuilderNode::on_get_map(
        const std::shared_ptr<go2_mapping_msgs::srv::GetMap::Request>& /*request*/,
        std::shared_ptr<go2_mapping_msgs::srv::GetMap::Response>& response
    )
    {
        // sending pending points as a delta, so the snapshot matches seq_ exactly
        publish_delta();

        std_msgs::msg::Header header;
        header.stamp = newest_stamp_;
        header.frame_id = frame_id_;
        response->run_id = run_id_;
        response->seq = seq_;
        response->points = to_message(map_.all(), header);
    }

    void MapBuilderNode::on_save(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response>& response
    )
    {
        try {
            response->message = save_map().string();
            response->success = true;
        }
        catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(get_logger(), "save failed: %s", e.what());
        }
    }

    void MapBuilderNode::save_at_exit()
    {
        if (!save_on_exit_) {
            return;
        }

        try {
            save_map();
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "save at exit failed: %s", e.what());
        }
    }

    std::filesystem::path MapBuilderNode::save_map()
    {
        if (output_dir_.empty()) {
            throw std::runtime_error("output_dir is not set");
        }

        if (!map_.size()) {
            throw std::runtime_error("map is empty");
        }

        const std::filesystem::path folder = make_run_folder(output_dir_, first_stamp_ns_);
        save_pcd(folder / "map.pcd", map_.all());
        save_tum(folder / "trajectory.tum", trajectory_);
        RCLCPP_INFO(
            get_logger(),
            "saved %zu points and %zu poses to %s",
            map_.size(),
            trajectory_.poses().size(),
            folder.string().c_str()
        );
        return folder;
    }
}
