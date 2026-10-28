#include "go2_calibration/messages.hpp"

#include <cstddef>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace go2_calibration {
    namespace {
    constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

    // The 4 bytes of a float as an integer, so that equal floats compare equal in a key.
    std::uint32_t bits(float value)
    {
        std::uint32_t result;
        std::memcpy(&result, &value, sizeof result);
        return result;
    }

    std::string xyz_json(const Command &command)
    {
        std::ostringstream json;
        json.imbue(std::locale::classic());  // always a decimal point, whatever the system locale
        json << std::setprecision(6) << "{\"x\":" << command.x << ",\"y\":" << command.y << ",\"z\":" << command.z
             << '}';
        return json.str();
    }
    }  // namespace

    std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nanosec;
    }

    Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3 &vector)
    {
        // casting vector to eigen in order to allow to perform operation on them
        return Eigen::Vector3d(vector.x, vector.y, vector.z);
    }

    std::vector<ScanPoint> to_scan_points(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        std::vector<ScanPoint> points;
        const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
        if (count == 0) {
            return points;
        }

        // The iterators throw std::runtime_error for a missing field.
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
        sensor_msgs::PointCloud2ConstIterator<float> time(cloud, "time");
        sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(cloud, "ring");

        points.reserve(count);
        for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++intensity, ++time, ++ring) {
            const Eigen::Vector3d position(*x, *y, *z);
            if (position.allFinite()) {
                points.push_back(ScanPoint{PointKey{bits(*time), bits(*intensity), *ring}, position});
            }
        }
        return points;
    }

    Eigen::Vector3d body_gyro(const unitree_go::msg::SportModeState &state)
    {
        const auto &gyroscope = state.imu_state.gyroscope;
        return Eigen::Vector3d(gyroscope[0], gyroscope[1], gyroscope[2]);
    }

    Eigen::Vector3d body_accel(const unitree_go::msg::SportModeState &state)
    {
        const auto &accelerometer = state.imu_state.accelerometer;
        return Eigen::Vector3d(accelerometer[0], accelerometer[1], accelerometer[2]);
    }

    unitree_api::msg::Request to_request(const Command &command)
    {
        unitree_api::msg::Request request;
        switch (command.action) {
        case Action::stop_move:
            request.header.identity.api_id = kStopMoveApiId;
            break;
        case Action::balance_stand:
            request.header.identity.api_id = kBalanceStandApiId;
            break;
        case Action::move:
            request.header.identity.api_id = kMoveApiId;
            request.parameter = xyz_json(command);
            break;
        case Action::euler:
            request.header.identity.api_id = kEulerApiId;
            request.parameter = xyz_json(command);
            break;
        }
        return request;
    }
}
