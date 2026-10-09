#pragma once

#include <vector>

#include "go2_preprocessor/schemas.hpp"

namespace go2_preprocessor {
    // Turns every point from the lidar frame (utlidar_lidar, mounted upside down
    // and tilted) into the body frame (x forward, y left, z up), lowers it by the
    // lidar height offset and sets frame_id to "body".
    void lidar_to_body(PointCloud &cloud);

    // Returns the points outside the box, in their original order.
    std::vector<Point> remove_robot_points(const std::vector<Point> &points, const Box &box);

    // Full lidar step: lidar_to_body, then removes the points that hit the
    // robot's own back. The box is defined in the body frame, so the order matters.
    void preprocess_cloud(PointCloud &cloud);
}
