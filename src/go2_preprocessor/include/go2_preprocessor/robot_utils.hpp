#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "go2_preprocessor/schemas.hpp"

namespace go2_preprocessor {
    // turns every point from the lidar frame (utlidar_lidar) into the body frame
    // (x forward, y left, z up): p_body = R * p_lidar + t, with `transform`
    // = SensorCalibration::lidar_to_body. Sets frame_id to "body".
    void lidar_to_body(PointCloud &cloud, const Eigen::Isometry3f &transform);

    // returns the points outside the box, in their original order.
    std::vector<Point> remove_robot_points(const std::vector<Point> &points, const Box &box);

    // full lidar step: lidar_to_body, then removes the points inside robot_body,
    // where the lidar sees the robot's own back. the box is defined in the body
    // frame, so the order matters.
    void preprocess_cloud(PointCloud &cloud, const Eigen::Isometry3f &lidar_to_body, const Box &robot_body);
}
