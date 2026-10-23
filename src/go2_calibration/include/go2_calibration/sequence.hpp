#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace go2_calibration {
    // which Unitree sport API call to send.
    enum class Action {
        stop_move,      // stand still
        balance_stand,  // balance-stand mode, which body tilts (Euler) need; sent once per phase
        move,           // Move(vx, vy, vyaw)
        euler,          // Euler(roll, pitch, yaw): body angles, the feet stay in place
    };

    // a sport API call and its three arguments: for move, x and y in m/s and z the yaw rate in
    // rad/s (+ = left); for euler, roll, pitch and yaw in rad.
    struct Command {
        Action action = Action::stop_move;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct SequenceSettings {
        double still_s = 30.0;        // first part, robot still: gyroscope bias, lidar mount, floor
        double pause_s = 3.0;         // still between the moving parts and at the end
        double spin_rate = 1.0;       // rad/s
        double spin_turns = 2.0;      // in each direction
        double tilt_angle = 0.2;      // rad; 0 leaves the tilts out
        double tilt_frequency = 0.4;  // Hz
        double tilt_s = 15.0;         // pitch for this long, then roll for as long
    };

    // limits no setting may exceed, whatever the parameters say.
    constexpr double kMaxSpinRate = 1.5;       // rad/s
    constexpr double kMaxTiltAngle = 0.3;      // rad
    constexpr double kMaxTiltFrequency = 1.0;  // Hz

    struct Phase {
        std::string name;  // published on the phase topic; use_of() reads it back
        double duration_s = 0.0;
        Action action = Action::stop_move;
        double yaw_rate = 0.0;  // move, rad/s
        // euler: angle = amplitude * sin(2 pi frequency t), t from the start of the phase
        double roll_amplitude = 0.0;
        double pitch_amplitude = 0.0;
        double frequency = 0.0;
    };

    // what the samples of a phase are for.
    enum class Use {
        none,    // transitions: dropped
        still,   // robot still: bias, lidar mount, floor
        moving,  // robot turning or tilting: IMU rotation, leak
    };

    // "still" is still; names starting with "spin_" or "tilt_" are moving; anything else is none.
    // The bag source only sees phase names, so this is the one place that maps them.
    Use use_of(const std::string &phase_name);

    // throws std::invalid_argument naming the first setting outside its range.
    void check_settings(const SequenceSettings &settings);

    // still, spin_left, pause, spin_right, pause; then, when tilt_angle > 0, balance_stand,
    // tilt_pitch, tilt_roll, level, pause. Checks the settings first.
    std::vector<Phase> build_sequence(const SequenceSettings &settings);

    double total_duration_s(const std::vector<Phase> &phases);

    struct Setpoint {
        std::size_t phase = 0;  // index into the phases
        Command command;
    };

    // what to send elapsed_s after the start of the sequence; empty once it is over.
    std::optional<Setpoint> setpoint_at(const std::vector<Phase> &phases, double elapsed_s);
}
