#include "go2_calibration/live_source.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <string>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace go2_calibration {
    namespace {
    // unitree's examples repeat motion commands in a loop; 50 Hz keeps Move and Euler smooth.
    constexpr double kRateHz = 50.0;
    // spin_some() takes one message per subscription, and a sample is stamped when its callback
    // runs: spinning at 50 Hz kept 1 IMU sample in 5, each one the oldest of a full queue (about
    // 0.4 s late). So the sequence spins far faster than the IMUs (250 and 300 Hz) and commands at 50 Hz.
    constexpr double kSpinRateHz = 1000.0;
    constexpr double kWaitForTopicsS = 5.0;
    constexpr double kBodySilentS = 0.5;
    constexpr double kStopMoveS = 0.5;
    // room for far more than a slow spin, while a cloud is converted (the body state comes at about 300 Hz).
    constexpr std::size_t kQueueSize = 100;
    }  // namespace

    LiveResult run_live(
        rclcpp::Node &node, const Topics &topics, const std::vector<Phase> &phases, CalibrationBuffer &buffer,
        const std::atomic<bool> &stop_requested)
    {
        // steady time for the sequence, which must not stretch if the system clock is set; receive
        // time on the ROS clock for the samples, as a bag stores them.
        rclcpp::Clock steady(RCL_STEADY_TIME);
        const auto seconds_since = [&steady](const rclcpp::Time &time) { return (steady.now() - time).seconds(); };
        const auto receive_ns = [&node]() { return node.now().nanoseconds(); };
        std::map<std::string, rclcpp::Time> last_seen;  // sensor topic -> steady time of its last message

        // 1. subscriptions fill the buffer; publishers drive the robot and name the phase.
        const auto qos = rclcpp::SensorDataQoS().keep_last(kQueueSize); // settings for sensors, how much data to keep
        const auto imu_subscription = node.create_subscription<sensor_msgs::msg::Imu>(
            topics.imu, qos, [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
                last_seen[topics.imu] = steady.now();
                buffer.add_lidar_imu(
                    receive_ns(), to_eigen(message->angular_velocity), to_eigen(message->linear_acceleration));
            });
        const auto body_subscription = node.create_subscription<unitree_go::msg::SportModeState>(
            topics.body_state, qos, [&](unitree_go::msg::SportModeState::ConstSharedPtr message) {
                last_seen[topics.body_state] = steady.now();
                buffer.add_body_imu(receive_ns(), body_gyro(*message), body_accel(*message));
            });
        // callback for cloud messages, converts to scan points and adds to buffer
        const auto on_cloud = [&](const std::string &topic, bool body_frame) {
            return [&, topic, body_frame](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                last_seen[topic] = steady.now();
                if (buffer.wants_scans()) {  // converting a cloud is the slow part
                    buffer.add_scan(to_nanoseconds(message->header.stamp), to_scan_points(*message), body_frame);
                }
            };
        };
        const auto cloud_subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            topics.cloud, qos, on_cloud(topics.cloud, false));
        const auto cloud_base_subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            topics.cloud_base, qos, on_cloud(topics.cloud_base, true));
        const auto request_publisher = node.create_publisher<unitree_api::msg::Request>(topics.request, 10); // motion controller commands
        const auto phase_publisher = node.create_publisher<std_msgs::msg::String>(topics.phase, 10); // phase name

        // rclcpp object that executes callbacks
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node.get_node_base_interface());
        rclcpp::WallRate rate(kRateHz);
        rclcpp::WallRate spin_rate(kSpinRateHz);

        // helper lambda to send a command to the motion controller
        const auto send = [&](const Command &command) { request_publisher->publish(to_request(command)); };
        const auto set_phase = [&](const std::string &name) {
            std_msgs::msg::String message;
            message.data = name;
            phase_publisher->publish(message);
            buffer.set_phase(name);
        };
        const auto stop_robot = [&]() {
            set_phase("stopping");
            const rclcpp::Time since = steady.now();
            while (seconds_since(since) < kStopMoveS) {
                send(Command{});  // a default Command is StopMove
                rate.sleep();
            }
        };

        // 2. every sensor topic must be alive before anything moves.
        const rclcpp::Time created = steady.now();
        for (const std::string &topic : {topics.cloud, topics.cloud_base, topics.imu, topics.body_state}) {
            // wait for the topic to start receiving messages (if I received it once, it must be alive)
            while (last_seen.count(topic) == 0) {
                if (stop_requested) {
                    return LiveResult::stopped;
                }
                if (seconds_since(created) > kWaitForTopicsS) {
                    RCLCPP_ERROR(node.get_logger(), "no message on %s within %.0f s", topic.c_str(), kWaitForTopicsS);
                    return LiveResult::no_data;
                }
                set_phase("waiting");
                executor.spin_some();
                rate.sleep();
            }
        }

        // 3. The phases, until the end, a stop request or a silent robot.
        RCLCPP_INFO(node.get_logger(), "all topics alive, the %.0f s sequence starts", total_duration_s(phases));
        try {
            const rclcpp::Time started = steady.now();
            std::optional<std::size_t> current;
            std::optional<rclcpp::Time> last_command;
            while (true) {
                executor.spin_some();
                if (stop_requested) {
                    RCLCPP_WARN(node.get_logger(), "stop requested");
                    stop_robot();
                    return LiveResult::stopped;
                }
                if (seconds_since(last_seen[topics.body_state]) > kBodySilentS) {
                    RCLCPP_ERROR(node.get_logger(), "%s went silent", topics.body_state.c_str());
                    stop_robot();
                    return LiveResult::robot_silent;
                }

                const auto setpoint = setpoint_at(phases, seconds_since(started));
                if (!setpoint) {
                    break;
                }
                if (!last_command || seconds_since(*last_command) >= 1.0 / kRateHz) {
                    last_command = steady.now();
                    const Phase &phase = phases[setpoint->phase];
                    const bool new_phase = current != setpoint->phase;
                    if (new_phase) {
                        current = setpoint->phase;
                        RCLCPP_INFO(node.get_logger(), "phase %s, %.1f s", phase.name.c_str(), phase.duration_s);
                    }
                    // a mode switch is asked once; motion commands are repeated on every command tick.
                    if (phase.action != Action::balance_stand || new_phase) {
                        send(setpoint->command);
                    }
                    set_phase(phase.name);
                }
                spin_rate.sleep();
            }
        } catch (...) {
            stop_robot();
            throw;
        }
        stop_robot();
        return LiveResult::completed;
    }
}
