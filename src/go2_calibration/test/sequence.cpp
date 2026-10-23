// Unit tests for use_of(), check_settings(), build_sequence(), total_duration_s() and setpoint_at().
#include "go2_calibration/sequence.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> names(const std::vector<Phase> &phases)
{
    std::vector<std::string> result;
    for (const Phase &phase : phases) {
        result.push_back(phase.name);
    }
    return result;
}

// Seconds from the start of the sequence to the start of phase `index`.
double start_of(const std::vector<Phase> &phases, std::size_t index)
{
    double start = 0.0;
    for (std::size_t i = 0; i < index; ++i) {
        start += phases[i].duration_s;
    }
    return start;
}

TEST(UseOf, MapsPhaseNamesToTheirUse)
{
    EXPECT_EQ(use_of("still"), Use::still);
    EXPECT_EQ(use_of("spin_left"), Use::moving);
    EXPECT_EQ(use_of("spin_right"), Use::moving);
    EXPECT_EQ(use_of("tilt_pitch"), Use::moving);
    EXPECT_EQ(use_of("tilt_roll"), Use::moving);
    EXPECT_EQ(use_of("pause"), Use::none);
    EXPECT_EQ(use_of("balance_stand"), Use::none);
    EXPECT_EQ(use_of("level"), Use::none);
    EXPECT_EQ(use_of("waiting"), Use::none);
    EXPECT_EQ(use_of("stopping"), Use::none);
    EXPECT_EQ(use_of(""), Use::none);
}

TEST(BuildSequence, DefaultIsStillThenSpinsThenTilts)
{
    const std::vector<Phase> phases = build_sequence(SequenceSettings{});

    EXPECT_EQ(
        names(phases), (std::vector<std::string>{
                           "still", "spin_left", "pause", "spin_right", "pause", "balance_stand", "tilt_pitch",
                           "tilt_roll", "level", "pause"}));
    // 2 turns at 1 rad/s take 4 pi s in each direction.
    EXPECT_NEAR(phases[1].duration_s, 4.0 * kPi, 1e-9);
    // still 30 + 3 pauses of 3 + 2 spins + balance 1 + 2 tilts of 15 + level 2.
    EXPECT_NEAR(total_duration_s(phases), 30.0 + 9.0 + 8.0 * kPi + 1.0 + 30.0 + 2.0, 1e-9);
}

TEST(BuildSequence, ZeroTiltAngleLeavesTheTiltsOut)
{
    SequenceSettings settings;
    settings.tilt_angle = 0.0;

    EXPECT_EQ(
        names(build_sequence(settings)),
        (std::vector<std::string>{"still", "spin_left", "pause", "spin_right", "pause"}));
}

TEST(CheckSettings, AcceptsTheDefaults)
{
    EXPECT_NO_THROW(check_settings(SequenceSettings{}));
}

TEST(CheckSettings, RejectsValuesBeyondTheLimits)
{
    SequenceSettings fast;
    fast.spin_rate = kMaxSpinRate + 0.1;
    EXPECT_THROW(check_settings(fast), std::invalid_argument);

    SequenceSettings steep;
    steep.tilt_angle = kMaxTiltAngle + 0.01;
    EXPECT_THROW(check_settings(steep), std::invalid_argument);

    SequenceSettings shaky;
    shaky.tilt_frequency = kMaxTiltFrequency + 0.1;
    EXPECT_THROW(check_settings(shaky), std::invalid_argument);

    SequenceSettings short_still;
    short_still.still_s = 5.0;  // too short for the bias
    EXPECT_THROW(check_settings(short_still), std::invalid_argument);

    SequenceSettings not_a_number;
    not_a_number.spin_turns = std::nan("");
    EXPECT_THROW(check_settings(not_a_number), std::invalid_argument);
}

// Without tilts, their frequency and length do not matter.
TEST(CheckSettings, IgnoresTheTiltSettingsWithoutTilts)
{
    SequenceSettings settings;
    settings.tilt_angle = 0.0;
    settings.tilt_frequency = 99.0;

    EXPECT_NO_THROW(check_settings(settings));
}

TEST(SetpointAt, StandsStillThenTurnsLeftThenRight)
{
    const std::vector<Phase> phases = build_sequence(SequenceSettings{});

    const auto still = setpoint_at(phases, 1.0);
    ASSERT_TRUE(still.has_value());
    EXPECT_EQ(still->phase, 0U);
    EXPECT_EQ(still->command.action, Action::stop_move);

    const auto left = setpoint_at(phases, start_of(phases, 1) + 1.0);
    ASSERT_TRUE(left.has_value());
    EXPECT_EQ(left->command.action, Action::move);
    EXPECT_EQ(left->command.z, 1.0);

    const auto right = setpoint_at(phases, start_of(phases, 3) + 1.0);
    ASSERT_TRUE(right.has_value());
    EXPECT_EQ(right->command.z, -1.0);
}

// A quarter period (1 / (4 * 0.4 Hz) = 0.625 s) into a tilt, the angle is at its 0.2 rad peak.
TEST(SetpointAt, TiltsFollowASine)
{
    const std::vector<Phase> phases = build_sequence(SequenceSettings{});

    const auto pitch = setpoint_at(phases, start_of(phases, 6) + 0.625);
    ASSERT_TRUE(pitch.has_value());
    EXPECT_EQ(pitch->command.action, Action::euler);
    EXPECT_NEAR(pitch->command.y, 0.2, 1e-9);
    EXPECT_EQ(pitch->command.x, 0.0);

    const auto roll = setpoint_at(phases, start_of(phases, 7) + 0.625);
    ASSERT_TRUE(roll.has_value());
    EXPECT_NEAR(roll->command.x, 0.2, 1e-9);
    EXPECT_EQ(roll->command.y, 0.0);
}

TEST(SetpointAt, EmptyAfterTheEnd)
{
    const std::vector<Phase> phases = build_sequence(SequenceSettings{});

    EXPECT_TRUE(setpoint_at(phases, total_duration_s(phases) - 0.01).has_value());
    EXPECT_FALSE(setpoint_at(phases, total_duration_s(phases) + 0.01).has_value());
}

}  // namespace
}  // namespace go2_calibration
