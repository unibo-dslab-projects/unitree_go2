#include "go2_preprocessor/preprocessor_node.hpp"
#include "go2_preprocessor/convertor.hpp"
#include "go2_preprocessor/imu_transform.hpp"
#include "go2_preprocessor/robot_utils.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace go2_preprocessor {
    namespace {

    constexpr char kCloudInputTopic[] = "/utlidar/cloud";
    constexpr char kCloudOutputTopic[] = "/utlidar/transformed_cloud";
    constexpr char kImuInputTopic[] = "/utlidar/imu";
    constexpr char kRawImuOutputTopic[] = "/utlidar/transformed_raw_imu";
    constexpr char kImuOutputTopic[] = "/utlidar/transformed_imu";
    constexpr char kBodyFrame[] = "body";
    // Publishers keep the default reliable QoS: reliable subscribers (Point-LIO)
    // and best-effort ones (RViz) can both connect to them.
    constexpr int kCloudQueueSize = 10;
    constexpr int kImuQueueSize = 50;  // the IMU publishes much faster than the lidar
    constexpr int kLogPeriodMs = 2000;

    // The node reads its settings once at start-up, so a later `ros2 param set`
    // would be silently ignored. Read-only makes it fail with an error instead.
    rcl_interfaces::msg::ParameterDescriptor read_only()
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return descriptor;
    }

    // ROS parameters have no float type: declared as double, stored as float.
    float declare_float(rclcpp::Node &node, const std::string &name, double default_value)
    {
        return static_cast<float>(node.declare_parameter<double>(name, default_value, read_only()));
    }

    Eigen::Vector3f to_eigen(const geometry_msgs::msg::Vector3 &vector)
    {
        return Eigen::Vector3f(
            static_cast<float>(vector.x), static_cast<float>(vector.y), static_cast<float>(vector.z));
    }

    Eigen::Quaternionf to_eigen(const geometry_msgs::msg::Quaternion &quaternion)
    {
        // Eigen's constructor takes w first; the message stores it last.
        return Eigen::Quaternionf(
            static_cast<float>(quaternion.w), static_cast<float>(quaternion.x),
            static_cast<float>(quaternion.y), static_cast<float>(quaternion.z));
    }

    geometry_msgs::msg::Vector3 to_vector_message(const Eigen::Vector3f &vector)
    {
        geometry_msgs::msg::Vector3 message;

        message.x = vector.x();
        message.y = vector.y();
        message.z = vector.z();

        return message;
    }

    geometry_msgs::msg::Quaternion to_quaternion_message(const Eigen::Quaternionf &quaternion)
    {
        geometry_msgs::msg::Quaternion message;

        message.x = quaternion.x();
        message.y = quaternion.y();
        message.z = quaternion.z();
        message.w = quaternion.w();

        return message;
    }

    }  // namespace

    PreprocessorNode::PreprocessorNode() : Node("go2_preprocessor")
    {
        // Defaults are CMU's (transform_everything.py; the leaks are the values
        // it uses when no calibration file exists).

        // Where the lidar sees the robot's own back, body frame, metres.
        robot_body_.min_x = declare_float(*this, "robot_box.min_x", -0.7);
        robot_body_.max_x = declare_float(*this, "robot_box.max_x", -0.1);
        robot_body_.min_y = declare_float(*this, "robot_box.min_y", -0.3);
        robot_body_.max_y = declare_float(*this, "robot_box.max_y", 0.3);
        robot_body_.min_z = declare_float(*this, "robot_box.min_z", -0.646825);
        robot_body_.max_z = declare_float(*this, "robot_box.max_z", -0.046825);

        // IMU calibration, body frame: biases in rad/s and m/s^2, leaks unitless.
        gyro_calibration_.bias.x() = declare_float(*this, "gyro_bias.x", 0.0);
        gyro_calibration_.bias.y() = declare_float(*this, "gyro_bias.y", 0.0);
        gyro_calibration_.bias.z() = declare_float(*this, "gyro_bias.z", 0.0);
        gyro_calibration_.z_to_x_leak = declare_float(*this, "gyro_leak.z_to_x", 0.15);
        gyro_calibration_.z_to_y_leak = declare_float(*this, "gyro_leak.z_to_y", -0.28);
        accel_bias_.x() = declare_float(*this, "accel_bias.x", 0.0);
        accel_bias_.y() = declare_float(*this, "accel_bias.y", 0.0);
        accel_bias_.z() = declare_float(*this, "accel_bias.z", 0.0);

        // true: Point-LIO gets the gyroscope only, as in CMU.
        zero_acceleration_ = declare_parameter<bool>("zero_acceleration", true, read_only());

        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kCloudOutputTopic, kCloudQueueSize);

        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            kCloudInputTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(message); });

        raw_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(kRawImuOutputTopic, kImuQueueSize);
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(kImuOutputTopic, kImuQueueSize);

        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            kImuInputTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(message); });
    }

    void PreprocessorNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
    {
        PointCloud cloud;

        // A broken message is skipped; an uncaught exception would stop the node.
        try {
            cloud = from_message(*message);
        } catch (const std::runtime_error &error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogPeriodMs, "cloud skipped: %s", error.what());
            return;
        }

        const std::size_t points_in = cloud.points.size();

        preprocess_cloud(cloud, robot_body_);

        cloud_publisher_->publish(to_message(cloud));

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), kLogPeriodMs, "points in %zu, out %zu, robot points removed %zu",
            points_in, cloud.points.size(), points_in - cloud.points.size());
    }

    void PreprocessorNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message)
    {
        // Calibration values are in the body frame, so rotate first.
        const Eigen::Vector3f angular_velocity =
            calibrate_gyro(imu_to_body(to_eigen(message->angular_velocity)), gyro_calibration_);
        const Eigen::Vector3f acceleration =
            calibrate_accel(imu_to_body(to_eigen(message->linear_acceleration)), accel_bias_);

        // A new message, so the covariances stay all zero ("unknown"), as in CMU.
        sensor_msgs::msg::Imu body_imu;
        body_imu.header.stamp = message->header.stamp;
        body_imu.header.frame_id = kBodyFrame;
        body_imu.orientation = to_quaternion_message(imu_orientation_to_body(to_eigen(message->orientation)));
        body_imu.angular_velocity = to_vector_message(angular_velocity);
        body_imu.linear_acceleration = to_vector_message(acceleration);

        raw_imu_publisher_->publish(body_imu);

        // Point-LIO never gets the orientation (a default Quaternion message is
        // the identity, w = 1). By default it gets no acceleration either, as in CMU.
        body_imu.orientation = geometry_msgs::msg::Quaternion();
        if (zero_acceleration_) {
            body_imu.linear_acceleration = geometry_msgs::msg::Vector3();
        }

        imu_publisher_->publish(body_imu);
    }
}
