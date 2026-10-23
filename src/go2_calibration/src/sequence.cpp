#include "go2_calibration/sequence.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace go2_calibration {
    namespace {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kBalanceStandS = 1.0;  // time for the mode switch before the tilts
    constexpr double kLevelS = 2.0;         // body back to level before the last pause

    // Written so that NaN fails too: every comparison with NaN is false.
    void check_range(const char *name, double value, double min, double max)
    {
        if (!(value >= min && value <= max)) {
            throw std::invalid_argument(
                std::string(name) + " = " + std::to_string(value) + " is outside [" + std::to_string(min) + ", " +
                std::to_string(max) + "]");
        }
    }

    bool starts_with(const std::string &text, const std::string &prefix)
    {
        return text.rfind(prefix, 0) == 0;
    }

    Phase still(const std::string &name, double duration_s)
    {
        Phase phase;
        phase.name = name;
        phase.duration_s = duration_s;
        return phase;
    }

    Phase spin(const std::string &name, double duration_s, double yaw_rate)
    {
        Phase phase = still(name, duration_s);
        phase.action = Action::move;
        phase.yaw_rate = yaw_rate;
        return phase;
    }

    Phase tilt(const std::string &name, double duration_s, double roll, double pitch, double frequency)
    {
        Phase phase = still(name, duration_s);
        phase.action = Action::euler;
        phase.roll_amplitude = roll;
        phase.pitch_amplitude = pitch;
        phase.frequency = frequency;
        return phase;
    }
    }  // namespace

    Use use_of(const std::string &phase_name)
    {
        if (phase_name == "still") {
            return Use::still;
        }
        if (starts_with(phase_name, "spin_") || starts_with(phase_name, "tilt_")) {
            return Use::moving;
        }
        return Use::none;
    }

    /// Validates the user's parameter before any motion sequence is built
    void check_settings(const SequenceSettings &settings)
    {
        check_range("still_s", settings.still_s, 10.0, 300.0);
        check_range("pause_s", settings.pause_s, 1.0, 30.0);
        // below 0.3 rad/s a turn is mostly gyroscope noise for the IMU fit (it skips < 0.15).
        check_range("spin_rate", settings.spin_rate, 0.3, kMaxSpinRate);
        check_range("spin_turns", settings.spin_turns, 0.5, 5.0);
        check_range("tilt_angle", settings.tilt_angle, 0.0, kMaxTiltAngle);
        if (settings.tilt_angle > 0.0) {
            check_range("tilt_frequency", settings.tilt_frequency, 0.1, kMaxTiltFrequency);
            check_range("tilt_s", settings.tilt_s, 2.0, 60.0);
        }
    }

    std::vector<Phase> build_sequence(const SequenceSettings &settings)
    {
        check_settings(settings);
        const double spin_s = settings.spin_turns * 2.0 * kPi / settings.spin_rate;

        std::vector<Phase> phases;
        phases.push_back(still("still", settings.still_s));
        phases.push_back(spin("spin_left", spin_s, settings.spin_rate));
        phases.push_back(still("pause", settings.pause_s));
        phases.push_back(spin("spin_right", spin_s, -settings.spin_rate));
        phases.push_back(still("pause", settings.pause_s));

        if (settings.tilt_angle > 0.0) {
            Phase balance = still("balance_stand", kBalanceStandS);
            balance.action = Action::balance_stand;
            phases.push_back(balance);
            phases.push_back(tilt("tilt_pitch", settings.tilt_s, 0.0, settings.tilt_angle, settings.tilt_frequency));
            phases.push_back(tilt("tilt_roll", settings.tilt_s, settings.tilt_angle, 0.0, settings.tilt_frequency));
            phases.push_back(tilt("level", kLevelS, 0.0, 0.0, 0.0));
            phases.push_back(still("pause", settings.pause_s));
        }
        return phases;
    }

    double total_duration_s(const std::vector<Phase> &phases)
    {
        double total = 0.0;
        for (const Phase &phase : phases) {
            total += phase.duration_s;
        }
        return total;
    }

    /// Returns the setpoint at the given elapsed time, if one is defined.
    /// The setpoint is determined by the phase at the given elapsed time.
    std::optional<Setpoint> setpoint_at(const std::vector<Phase> &phases, double elapsed_s)
    {
        double phase_start_s = 0.0;
        for (std::size_t index = 0; index < phases.size(); ++index) {
            const Phase &phase = phases[index];
            if (elapsed_s < phase_start_s + phase.duration_s) {
                const double t = std::max(0.0, elapsed_s - phase_start_s);

                Setpoint setpoint;
                setpoint.phase = index;
                setpoint.command.action = phase.action;
                if (phase.action == Action::move) {
                    setpoint.command.z = phase.yaw_rate;
                } else if (phase.action == Action::euler) {
                    const double wave = std::sin(2.0 * kPi * phase.frequency * t);
                    setpoint.command.x = phase.roll_amplitude * wave;
                    setpoint.command.y = phase.pitch_amplitude * wave;
                }
                return setpoint;
            }
            phase_start_s += phase.duration_s;
        }
        return std::nullopt;
    }
}
