#pragma once

#include "go2_calibration/buffer.hpp"
#include "go2_calibration/messages.hpp"

#include <string>

namespace go2_calibration {
    // The bag source: replays a bag into the buffer in recording order, through the same calls the
    // live source makes. The bag must hold topics.phase as well as the four sensor topics, so record
    // it with `ros2 bag record` while go2_calibration runs live (README.md). Sample times are the
    // bag's receive times, as the live source uses receive times.
    // Throws std::runtime_error if the bag cannot be opened or lacks one of the topics.
    void read_bag(const std::string &bag, const Topics &topics, CalibrationBuffer &buffer);
}
