#include "go2_calibration/report.hpp"

#include <iomanip>
#include <sstream>

namespace go2_calibration {
    namespace {
    // Always with a decimal point and never in scientific notation: the ROS parameter parser reads
    // "1" as an integer and "1e-05" as a string, and an array must not mix types.
    std::string number(double value, int decimals)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(decimals) << value;
        return out.str();
    }

    // For the comments only: the shortest readable form, whatever the size.
    std::string short_number(double value)
    {
        std::ostringstream out;
        out << std::setprecision(6) << value;
        return out.str();
    }

    std::string vector_text(const Eigen::Vector3d &vector, int decimals)
    {
        return "[" + number(vector.x(), decimals) + ", " + number(vector.y(), decimals) + ", " +
               number(vector.z(), decimals) + "]";
    }

    // Row-major: the first three values are the first row.
    std::string matrix_text(const Eigen::Matrix3d &matrix)
    {
        std::string text = "[";
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                text += number(matrix(row, column), 9);
                if (row < 2 || column < 2) {
                    text += ", ";
                }
            }
        }
        return text + "]";
    }
    }  // namespace

    std::vector<std::string> describe(const Calibration &calibration)
    {
        const Calibration &c = calibration;
        std::vector<std::string> lines = {
            "lidar_to_body: " + std::to_string(c.scans) + " scans, " + std::to_string(c.lidar_to_body.samples) +
                " matched points, error rms " + number(c.lidar_to_body.rms * 1000.0, 3) + " mm, max " +
                number(c.lidar_to_body.max_error * 1000.0, 3) + " mm",
            "floor: tilt " + number(c.floor_tilt_deg, 2) + " deg after lidar_to_body, lidar " +
                number(c.range.distance, 3) + " m above it",
            "range_offset: " + number(c.range.offset, 3) + " m (every lidar range reads this much too long), fit residual " +
                number(c.range.residual * 1000.0, 1) + " mm over " + std::to_string(c.range.points) + " floor points",
            "body_imu gyro_bias: " + std::to_string(c.body_gyro_bias.samples) + " samples, mean " +
                vector_text(c.body_gyro_bias.mean, 6) + " rad/s, noise std " + vector_text(c.body_gyro_bias.std_dev, 6) +
                " rad/s",
            "body_imu accel: mean " + vector_text(c.body_accel.mean, 3) + " m/s^2, |a| " +
                number(c.body_accel.mean.norm(), 3) + ", accel_scale " + number(c.accel_scale, 4) + ", gravity " +
                number(c.body_tilt_deg, 2) + " deg from +z",
            "lidar IMU, not used by go2_preprocessor:",
            "imu_to_body: " + std::to_string(c.imu_to_body.fit.samples) + " turning samples, error rms " +
                number(c.imu_to_body.fit.rms, 4) + " rad/s, lag " +
                number(static_cast<double>(c.imu_to_body.lag_ns) / 1e6, 0) + " ms, axis spread " +
                number(c.axis_spread, 3) + ", " + number(c.imu_to_lidar_deg, 2) + " deg from lidar_to_body",
            "gyro_bias: " + std::to_string(c.gyro_bias.samples) + " samples, mean " + vector_text(c.gyro_bias.mean, 6) +
                " rad/s, noise std " + vector_text(c.gyro_bias.std_dev, 6) + " rad/s",
            "gyro_leak left: z_to_x " + number(c.leak.z_to_x, 4) + ", z_to_y " + number(c.leak.z_to_y, 4) +
                " (near 0: the rotation explains the cross-talk)",
            "gravity after imu_to_body: " + number(c.gravity_tilt_deg, 2) + " deg from +z",
            "checks:",
        };
        for (const Check &check : c.checks) {
            lines.push_back(
                std::string(check.passed() ? "  PASS " : "  FAIL ") + check.name + ": " + short_number(check.value) +
                (check.at_least ? " (at least " : " (at most ") + short_number(check.limit) + ")");
        }
        return lines;
    }

    std::string to_parameter_file(
        const Calibration &calibration, const std::string &node_name, const std::string &source,
        const std::string &created)
    {
        const Calibration &c = calibration;
        std::ostringstream out;
        out << "# Go2 calibration for " << node_name << ", written by go2_calibration on " << created << "\n"
            << "# from " << source << ".\n"
            << "# Convention: p_body = R * p_sensor + t, R row-major. Body frame: x forward, y left, z up.\n"
            << "# body_imu is the IMU of /sportmodestate, already in body axes.\n"
            << "#\n";
        for (const std::string &line : describe(c)) {
            out << "# " << line << '\n';
        }

        out << node_name << ":\n"
            << "  ros__parameters:\n"
            << "    lidar_to_body:\n"
            << "      rotation: " << matrix_text(c.lidar_to_body.rotation) << '\n'
            << "      translation: " << vector_text(c.lidar_to_body.translation, 6) << '\n'
            << "    body_imu:\n"
            << "      gyro_bias:\n"
            << "        x: " << number(c.body_gyro_bias.mean.x(), 9) << '\n'
            << "        y: " << number(c.body_gyro_bias.mean.y(), 9) << '\n'
            << "        z: " << number(c.body_gyro_bias.mean.z(), 9) << '\n'
            << "      accel_scale: " << number(c.accel_scale, 6) << '\n'
            << "    range_offset: " << number(c.range.offset, 4) << '\n';
        return out.str();
    }
}
