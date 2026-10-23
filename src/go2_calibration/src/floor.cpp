#include "go2_calibration/floor.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <random>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>  // cross()

namespace go2_calibration {
    namespace {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinCrossNorm = 1e-9;  // three points on (almost) one line give no plane
    }  // namespace

    std::optional<Plane> find_floor(const std::vector<Eigen::Vector3d> &points, const FloorSearch &search)
    {
        if (points.size() < 3) {
            return std::nullopt;
        }

        std::mt19937 random(search.seed);
        std::vector<Eigen::Vector3d> sample;
        if (points.size() > search.max_points) {
            sample.reserve(search.max_points);
            std::sample(points.begin(), points.end(), std::back_inserter(sample), search.max_points, random);
        } else {
            sample = points;
        }

        std::uniform_int_distribution<std::size_t> pick(0, sample.size() - 1);
        std::optional<Plane> best;
        for (int i = 0; i < search.iterations; ++i) {
            const Eigen::Vector3d &a = sample[pick(random)];
            const Eigen::Vector3d &b = sample[pick(random)];
            const Eigen::Vector3d &c = sample[pick(random)];

            Eigen::Vector3d normal = (b - a).cross(c - a);
            if (normal.norm() < kMinCrossNorm) {
                continue;
            }
            normal.normalize();
            double distance = -normal.dot(a);
            if (distance < 0.0) {  // turn the normal towards the origin
                normal = -normal;
                distance = -distance;
            }
            if (distance < search.min_distance || distance > search.max_distance) {
                continue;
            }

            const auto inliers = static_cast<std::size_t>(
                std::count_if(sample.begin(), sample.end(), [&](const Eigen::Vector3d &point) {
                    return std::abs(normal.dot(point) + distance) < search.tolerance;
                }));
            if (!best || inliers > best->inliers) {
                best = Plane{normal, distance, inliers, sample.size()};
            }
        }
        if (!best) {
            return best;
        }

        // Three random points give a rough normal; the least-squares plane through all the inliers
        // (the direction in which they spread least) gives the precise one.
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> inliers;
        for (const Eigen::Vector3d &point : sample) {
            if (std::abs(best->normal.dot(point) + best->distance) < search.tolerance) {
                inliers.push_back(point);
                centroid += point;
            }
        }
        centroid /= static_cast<double>(inliers.size());
        Eigen::Matrix3d spread = Eigen::Matrix3d::Zero();
        for (const Eigen::Vector3d &point : inliers) {
            spread += (point - centroid) * (point - centroid).transpose();
        }
        Eigen::Vector3d normal = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(spread).eigenvectors().col(0);
        if (normal.dot(best->normal) < 0.0) {  // keep it pointing towards the origin
            normal = -normal;
        }
        best->normal = normal;
        best->distance = -normal.dot(centroid);
        return best;
    }

    double tilt_from_up_deg(const Eigen::Vector3d &direction)
    {
        const double cosine = std::clamp(direction.normalized().z(), -1.0, 1.0);
        return std::acos(cosine) * 180.0 / kPi;
    }

    RangeOffset fit_range_offset(const std::vector<Eigen::Vector3d> &points, const Plane &plane, double band)
    {
        // Sums for the 2 x 2 normal equations of distance_i = h + offset * c_i.
        double n = 0.0;
        double sum_c = 0.0;
        double sum_d = 0.0;
        double sum_cc = 0.0;
        double sum_cd = 0.0;
        for (const Eigen::Vector3d &point : points) {
            // The normal points from the plane towards the lidar, so -normal . p is the distance
            // of the point along the normal, and divided by the range it is cos(a).
            const double distance = -plane.normal.dot(point);
            const double range = point.norm();
            if (std::abs(distance - plane.distance) > band || range <= 0.0) {
                continue;
            }
            const double c = distance / range;
            n += 1.0;
            sum_c += c;
            sum_d += distance;
            sum_cc += c * c;
            sum_cd += c * distance;
        }

        const double determinant = n * sum_cc - sum_c * sum_c;
        if (n < 3.0 || determinant <= kMinCrossNorm) {
            throw std::invalid_argument("fit_range_offset: needs 3 points seen at different angles");
        }

        RangeOffset result;
        result.offset = (n * sum_cd - sum_c * sum_d) / determinant;
        result.distance = (sum_d - result.offset * sum_c) / n;
        result.points = static_cast<std::size_t>(n);

        double squared_sum = 0.0;
        for (const Eigen::Vector3d &point : points) {
            const double distance = -plane.normal.dot(point);
            const double range = point.norm();
            if (std::abs(distance - plane.distance) <= band && range > 0.0) {
                const double error = distance - (result.distance + result.offset * distance / range);
                squared_sum += error * error;
            }
        }
        result.residual = std::sqrt(squared_sum / n);
        return result;
    }
}
