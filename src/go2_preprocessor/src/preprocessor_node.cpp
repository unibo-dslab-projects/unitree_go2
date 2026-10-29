#include "go2_preprocessor/preprocessor_node.hpp"
#include "go2_preprocessor/calibration.hpp"
#include "go2_preprocessor/convertor.hpp"
#include "go2_preprocessor/imu_transform.hpp"
#include "go2_preprocessor/robot_utils.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/vector3.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace go2_preprocessor {
    namespace {

    constexpr char kCloudInputTopic[] = "/utlidar/cloud";
    constexpr char kCloudOutputTopic[] = "/utlidar/transformed_cloud";
    constexpr char kBodyStateInputTopic[] = "/sportmodestate";
    constexpr char kImuOutputTopic[] = "/utlidar/transformed_imu";
    constexpr char kBodyFrame[] = "body";
    constexpr int kCloudQueueSize = 10;
    constexpr int kImuQueueSize = 50;  // the IMU publishes much faster than the lidar
    constexpr int kLogPeriodMs = 2000;
    constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

    // calibration defaults
    const std::vector<double> kLidarRotation = {
        -0.965512905, 0.0, 0.260355201, 0.0, 1.0, 0.0, -0.260355201, 0.0, -0.965512905};
    const std::vector<double> kLidarTranslation = {0.0, 0.0, -0.046825};  // m

    // the node reads its settings once at start-up, so a later `ros2 param set`
    // would be silently ignored. Read-only makes it fail with an error instead.
    rcl_interfaces::msg::ParameterDescriptor read_only()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return descriptor;
    }

    /// ROS parameters have no float type: declared as double, stored as float (required by PointCloud2).
    float declare_float(rclcpp::Node &node, const std::string &name, double default_value)
    {
        return static_cast<float>(node.declare_parameter<double>(name, default_value, read_only()));
    }

    // the conversion functions do not know the parameter, so its name is added here.
    Eigen::Matrix3f declare_rotation(rclcpp::Node &node, const std::string &name, const std::vector<double> &default_values)
    {
        try {
            return rotation_from_parameter(node.declare_parameter<std::vector<double>>(name, default_values, read_only()));
        } catch (const std::invalid_argument &error) {
            throw std::invalid_argument(name + ": " + error.what());
        }
    }

    Eigen::Vector3f declare_vector3(rclcpp::Node &node, const std::string &name, const std::vector<double> &default_values)
    {
        try {
            return vector_from_parameter(node.declare_parameter<std::vector<double>>(name, default_values, read_only()));
        } catch (const std::invalid_argument &error) {
            throw std::invalid_argument(name + ": " + error.what());
        }
    }

    // names as go2_calibration writes them (report.cpp): a typo is not an error, the value
    // from the file is silently ignored and the default used.
    SensorCalibration read_calibration(rclcpp::Node &node)
    {
        SensorCalibration calibration;

        calibration.lidar_to_body.linear() = declare_rotation(node, "lidar_to_body.rotation", kLidarRotation);
        calibration.lidar_to_body.translation() = declare_vector3(node, "lidar_to_body.translation", kLidarTranslation);
        // a calibration file from before the body IMU has none of these: the defaults then leave
        // the readings as they are, and the start-up log shows bias 0 and scale 1.
        calibration.body_imu.gyro_bias.x() = declare_float(node, "body_imu.gyro_bias.x", 0.0);
        calibration.body_imu.gyro_bias.y() = declare_float(node, "body_imu.gyro_bias.y", 0.0);
        calibration.body_imu.gyro_bias.z() = declare_float(node, "body_imu.gyro_bias.z", 0.0);
        calibration.body_imu.accel_scale = declare_float(node, "body_imu.accel_scale", 1.0);

        calibration.range_offset = declare_float(node, "range_offset", 0.0);

        return calibration;
    }

    // one line per value, so the log shows whether the parameter file was applied.
    std::string describe(const SensorCalibration &calibration)
    {
        const Eigen::IOFormat one_line(6, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
        std::ostringstream out;

        out << "lidar_to_body rotation " << calibration.lidar_to_body.linear().format(one_line)
            << ", translation " << calibration.lidar_to_body.translation().transpose().format(one_line) << " m\n"
            << "body_imu gyro_bias " << calibration.body_imu.gyro_bias.transpose().format(one_line)
            << " rad/s, accel_scale " << calibration.body_imu.accel_scale << '\n'
            << "range_offset " << calibration.range_offset << " m";

        return out.str();
    }

    Eigen::Vector3f to_eigen(const std::array<float, 3> &values)
    {
        return Eigen::Vector3f(values[0], values[1], values[2]);
    }

    geometry_msgs::msg::Vector3 to_vector_message(const Eigen::Vector3f &vector)
    {
        geometry_msgs::msg::Vector3 message;

        message.x = vector.x();
        message.y = vector.y();
        message.z = vector.z();

        return message;
    }

    }  // namespace

    PreprocessorNode::PreprocessorNode() : Node("go2_preprocessor")
    {
        // where the lidar sees the robot's own back, body frame, metres.
        robot_body_.min_x = declare_float(*this, "robot_box.min_x", -0.7);
        robot_body_.max_x = declare_float(*this, "robot_box.max_x", -0.1);
        robot_body_.min_y = declare_float(*this, "robot_box.min_y", -0.3);
        robot_body_.max_y = declare_float(*this, "robot_box.max_y", 0.3);
        robot_body_.min_z = declare_float(*this, "robot_box.min_z", -0.646825);
        robot_body_.max_z = declare_float(*this, "robot_box.max_z", -0.046825);

        calibration_ = read_calibration(*this);
        RCLCPP_INFO(get_logger(), "calibration:\n%s", describe(calibration_).c_str());

        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kCloudOutputTopic, kCloudQueueSize);

        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            kCloudInputTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(message); });

        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(kImuOutputTopic, kImuQueueSize);

        body_state_subscription_ = create_subscription<unitree_go::msg::SportModeState>(
            kBodyStateInputTopic,
            rclcpp::SensorDataQoS(),
            [this](unitree_go::msg::SportModeState::ConstSharedPtr message) { on_body_state(message); });
    }

    void PreprocessorNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
    {
        PointCloud cloud;

        try {
            cloud = from_message(*message);
        } catch (const std::runtime_error &error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "cloud skipped: %s", error.what());
            return;
        }

        const std::size_t points_in = cloud.points.size();

        preprocess_cloud(cloud, calibration_.lidar_to_body, robot_body_, calibration_.range_offset);

        cloud_publisher_->publish(to_message(cloud));

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), kLogPeriodMs, "points in %zu, out %zu, robot points removed %zu",
            points_in, cloud.points.size(), points_in - cloud.points.size());
    }

    void PreprocessorNode::on_body_state(unitree_go::msg::SportModeState::ConstSharedPtr message)
    {
        // the stamp inside the message uses the lidar's clock; the receive time does not.
        const std::int64_t stamp_ns =
            static_cast<std::int64_t>(message->stamp.sec) * kNanosecondsPerSecond + message->stamp.nanosec;
        // 1 to 1.5 % of the messages repeat the previous sample, same stamp and same values
        // (2026-10-19 bags). Point-LIO would take the copy as one more measurement at the same instant.
        if (stamp_ns <= last_imu_stamp_ns_) {
            return;
        }
        last_imu_stamp_ns_ = stamp_ns;

        // a new message, so the covariances stay all zero ("unknown"), and the orientation stays
        // the identity (w = 1): Point-LIO reads only the rates and the acceleration.
        sensor_msgs::msg::Imu body_imu;
        body_imu.header.stamp.sec = message->stamp.sec;
        body_imu.header.stamp.nanosec = message->stamp.nanosec;
        body_imu.header.frame_id = kBodyFrame;
        body_imu.angular_velocity = to_vector_message(
            calibrate_gyro(to_eigen(message->imu_state.gyroscope), calibration_.body_imu.gyro_bias));
        body_imu.linear_acceleration = to_vector_message(
            calibrate_accel(to_eigen(message->imu_state.accelerometer), calibration_.body_imu.accel_scale));

        imu_publisher_->publish(body_imu);
    }
}
