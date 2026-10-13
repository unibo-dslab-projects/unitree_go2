#pragma once

#include "go2_calibration/buffer.hpp"
#include "go2_calibration/messages.hpp"
#include "go2_calibration/sequence.hpp"

#include <atomic>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace go2_calibration {
    // How a live run ended.
    enum class LiveResult {
        completed,     // every phase done
        no_data,       // a topic sent nothing within 5 s of the start: the robot never moved
        robot_silent,  // the body state stopped for 0.5 s while running
        stopped,       // stop_requested (Ctrl-C or SIGTERM)
    };

    // The live source: drives the robot through `phases` with the sport API and meanwhile fills the
    // buffer from the topics, publishing the current phase name on topics.phase at 50 Hz (so a bag
    // recorded alongside can be replayed by read_bag()). It first waits up to 5 s for every sensor
    // topic. Every way out after that, exceptions included, sends StopMove for 0.5 s.
    LiveResult run_live(
        rclcpp::Node &node, const Topics &topics, const std::vector<Phase> &phases, CalibrationBuffer &buffer,
        const std::atomic<bool> &stop_requested);
}
