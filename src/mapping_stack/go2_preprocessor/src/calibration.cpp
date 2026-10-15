#include "go2_preprocessor/calibration.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace go2_preprocessor {
    namespace {
    // the file has 9 decimals, so honest rounding stays near 1e-9; a typo is 1e-3 or more.
    constexpr double kRotationTolerance = 1e-3;

    void expect_size(const std::vector<double> &values, std::size_t size)
    {
        if (values.size() != size) {
            throw std::invalid_argument(
                "expected " + std::to_string(size) + " values, got " + std::to_string(values.size()));
        }
    }
    }  // namespace

    Eigen::Matrix3f rotation_from_parameter(const std::vector<double> &values)
    {
        // checked before the map, which would read past the end of a shorter vector.
        expect_size(values, 9);
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(values.data());

        // checked in double precision: rounding to float first would eat into the tolerance.
        if (!(rotation * rotation.transpose()).isIdentity(kRotationTolerance)) {
            throw std::invalid_argument("not a rotation: the axes are stretched or not perpendicular");
        }
        if (std::abs(rotation.determinant() - 1.0) > kRotationTolerance) {
            throw std::invalid_argument(
                "not a rotation: determinant " + std::to_string(rotation.determinant()) + " instead of +1 (mirror)");
        }

        return rotation.cast<float>();
    }

    Eigen::Vector3f vector_from_parameter(const std::vector<double> &values)
    {
        expect_size(values, 3);
        return Eigen::Vector3d(values[0], values[1], values[2]).cast<float>();
    }
}
