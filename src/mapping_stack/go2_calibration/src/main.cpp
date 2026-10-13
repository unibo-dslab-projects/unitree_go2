#include "go2_calibration/bag_source.hpp"
#include "go2_calibration/buffer.hpp"
#include "go2_calibration/calibrate.hpp"
#include "go2_calibration/live_source.hpp"
#include "go2_calibration/messages.hpp"
#include "go2_calibration/report.hpp"
#include "go2_calibration/sequence.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace {
    constexpr int kCountdownS = 5;  // to step back from the robot, or to cancel with Ctrl-C

    std::atomic<bool> stop_requested{false};

    void on_signal(int)
    {
        stop_requested = true;
    }

    struct Settings {
        std::string bag;  // empty: live run
        std::string output = "go2_calibration.yaml";
        std::string node_name = "go2_preprocessor";  // the node the parameter file is for
        std::size_t max_scans = 100;
        go2_calibration::Topics topics;
        go2_calibration::SequenceSettings motion;
        go2_calibration::Limits limits;
    };

    // Every setting is a ROS parameter (--ros-args -p name:=value, or --params-file); the defaults
    // are the ones in the structs.
    Settings read_settings(rclcpp::Node &node)
    {
        Settings s;
        s.bag = node.declare_parameter("bag", s.bag);
        s.output = node.declare_parameter("output", s.output);
        s.node_name = node.declare_parameter("node_name", s.node_name);
        s.max_scans = static_cast<std::size_t>(
            node.declare_parameter("max_scans", static_cast<std::int64_t>(s.max_scans)));

        s.topics.cloud = node.declare_parameter("topics.cloud", s.topics.cloud);
        s.topics.cloud_base = node.declare_parameter("topics.cloud_base", s.topics.cloud_base);
        s.topics.imu = node.declare_parameter("topics.imu", s.topics.imu);
        s.topics.body_state = node.declare_parameter("topics.body_state", s.topics.body_state);
        s.topics.request = node.declare_parameter("topics.request", s.topics.request);
        s.topics.phase = node.declare_parameter("topics.phase", s.topics.phase);

        s.motion.still_s = node.declare_parameter("motion.still_s", s.motion.still_s);
        s.motion.pause_s = node.declare_parameter("motion.pause_s", s.motion.pause_s);
        s.motion.spin_rate = node.declare_parameter("motion.spin_rate", s.motion.spin_rate);
        s.motion.spin_turns = node.declare_parameter("motion.spin_turns", s.motion.spin_turns);
        s.motion.tilt_angle = node.declare_parameter("motion.tilt_angle", s.motion.tilt_angle);
        s.motion.tilt_frequency = node.declare_parameter("motion.tilt_frequency", s.motion.tilt_frequency);
        s.motion.tilt_s = node.declare_parameter("motion.tilt_s", s.motion.tilt_s);

        s.limits.min_matched_points = node.declare_parameter("limits.min_matched_points", s.limits.min_matched_points);
        s.limits.max_mount_error_mm = node.declare_parameter("limits.max_mount_error_mm", s.limits.max_mount_error_mm);
        s.limits.max_floor_tilt_deg = node.declare_parameter("limits.max_floor_tilt_deg", s.limits.max_floor_tilt_deg);
        s.limits.max_gyro_error = node.declare_parameter("limits.max_gyro_error", s.limits.max_gyro_error);
        s.limits.min_axis_spread = node.declare_parameter("limits.min_axis_spread", s.limits.min_axis_spread);
        s.limits.max_still_noise = node.declare_parameter("limits.max_still_noise", s.limits.max_still_noise);
        s.limits.max_leak = node.declare_parameter("limits.max_leak", s.limits.max_leak);
        return s;
    }

    std::string local_time_now()
    {
        const std::time_t now = std::time(nullptr);
        std::tm local{};
        localtime_r(&now, &local);
        std::ostringstream out;
        out << std::put_time(&local, "%F %T %Z");
        return out.str();
    }

    // Warns, then counts down; false if Ctrl-C came first.
    bool countdown(double sequence_s)
    {
        std::cout << "THE ROBOT WILL MOVE BY ITSELF for about " << static_cast<int>(sequence_s)
                  << " s: turns and body tilts. It must stand on a flat floor with 1 m clear around it.\n";
        for (int s = kCountdownS; s > 0; --s) {
            std::cout << "starting in " << s << " s, Ctrl-C cancels" << std::endl;
            for (int i = 0; i < 10; ++i) {
                if (stop_requested) {
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        return true;
    }

    std::string describe(go2_calibration::LiveResult result)
    {
        switch (result) {
        case go2_calibration::LiveResult::completed:
            return "completed";
        case go2_calibration::LiveResult::no_data:
            return "a sensor topic is silent: is the robot on and ~/unitree_ros2/setup.sh sourced?";
        case go2_calibration::LiveResult::robot_silent:
            return "the robot stopped sending its state: robot stopped";
        case go2_calibration::LiveResult::stopped:
            return "stopped by request: robot stopped";
        }
        return "unknown";
    }

    int run()
    {
        // 1. settings.
        const auto node = std::make_shared<rclcpp::Node>("go2_calibration");
        const Settings settings = read_settings(*node);

        // 2. fill the buffer, from the robot or from a bag.
        go2_calibration::CalibrationBuffer buffer(settings.max_scans);
        std::string source = settings.bag;
        if (settings.bag.empty()) {
            // checks the motion settings before anything moves.
            const auto phases = go2_calibration::build_sequence(settings.motion);
            if (!countdown(go2_calibration::total_duration_s(phases))) {
                std::cerr << "cancelled, nothing moved\n";
                return 1;
            }
            const auto result = go2_calibration::run_live(*node, settings.topics, phases, buffer, stop_requested);
            if (result != go2_calibration::LiveResult::completed) {
                std::cerr << "error: live run: " << describe(result) << '\n';
                return 1;
            }
            source = "a live run";
        } else {
            std::cout << "reading " << settings.bag << "...\n";
            go2_calibration::read_bag(settings.bag, settings.topics, buffer);
        }

        // 3. calibrate and report.
        const go2_calibration::Calibration calibration = go2_calibration::calibrate(buffer.data(), settings.limits);
        std::cout << '\n';
        for (const std::string &line : go2_calibration::describe(calibration)) {
            std::cout << line << '\n';
        }

        // 4. write the file, only if every check passed.
        if (!calibration.passed()) {
            std::cerr << "\nerror: a check failed, " << settings.output << " not written\n";
            return 1;
        }
        std::ofstream file(settings.output);
        file << go2_calibration::to_parameter_file(calibration, settings.node_name, source, local_time_now());
        file.close();
        if (!file) {
            throw std::runtime_error("cannot write " + settings.output);
        }
        std::cout << "\nwritten: " << settings.output << '\n';
        return 0;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp's own Ctrl-C handler would shut ROS down at once, before a turning robot gets
    // StopMove: run_live() watches stop_requested instead and stops the robot first.
    rclcpp::uninstall_signal_handlers();
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    int exit_code = 1;
    try {
        exit_code = run();
    } catch (const std::exception &error) {
        std::cerr << "error: " << error.what() << '\n';
    }
    rclcpp::shutdown();
    return exit_code;
}
