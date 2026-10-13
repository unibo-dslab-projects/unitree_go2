#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace go2_calibration {
    // The plane normal . p + distance = 0, with a unit normal pointing from the plane towards the
    // origin (the lidar): for the floor, "up".
    struct Plane {
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        double distance = 0.0;  // from the origin, metres
        std::size_t inliers = 0;
        std::size_t checked = 0;  // points tested: a random subset of large clouds
    };

    struct FloorSearch {
        double min_distance = 0.2;  // the Go2 lidar sits about 0.3 to 0.4 m above the floor
        double max_distance = 0.7;
        double tolerance = 0.03;  // metres from the plane that still count as on it
        int iterations = 600;
        std::size_t max_points = 40000;
        unsigned int seed = 1;  // fixed, so every run gives the same answer
    };

    // RANSAC: among the planes between min_distance and max_distance from the origin, the one with
    // the most points, then refitted by least squares through those points. In the raw cloud of a
    // standing robot that is the floor. Empty if none is found.
    std::optional<Plane> find_floor(const std::vector<Eigen::Vector3d> &points, const FloorSearch &search = FloorSearch{});

    // Angle between a direction and +z, in degrees: the tilt of a floor normal or of gravity.
    double tilt_from_up_deg(const Eigen::Vector3d &direction);

    // A lidar whose every range reads `offset` metres too long bends a flat plane into a bowl: a
    // point whose ray meets the plane normal at angle a appears at distance + offset * cos(a) from
    // the lidar, deepest straight below. The Go2 L1 read about +0.12 m on 2026-10-13.
    struct RangeOffset {
        double offset = 0.0;    // metres; the correction shortens every ray by this much
        double distance = 0.0;  // the true distance from the lidar to the plane, metres
        double residual = 0.0;  // rms of what the fit leaves, metres
        std::size_t points = 0;
    };

    // Least squares of distance_i = distance + offset * cos(a_i) over the points within `band`
    // metres of the plane (found with the offset still in it). Throws std::invalid_argument with
    // fewer than 3 such points or when they all see the plane at one angle.
    RangeOffset fit_range_offset(const std::vector<Eigen::Vector3d> &points, const Plane &plane, double band = 0.08);
}
