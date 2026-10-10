#include "go2_preprocessor/robot_utils.hpp"

#include <vector>

namespace go2_preprocessor {
    namespace {
    // Go2 L1 mount and body geometry, from CMU autonomy_stack_go2
    // Turn around the lidar's y axis that undoes the upside-down mount (180 deg)
    // and the 15.1 deg tilt in one go: 164.9 deg.
    constexpr float kLidarToBodyAngle = 2.87820258F;
    // CMU subtracts the lidar height offset, hence the minus sign.
    constexpr float kLidarToBodyShiftZ = -0.046825F;
    }  // namespace

    void lidar_to_body(PointCloud &cloud)
    {
        for (auto &point : cloud.points) {
            point.rotate_y_axis(kLidarToBodyAngle);
            point.shift_z_axis(kLidarToBodyShiftZ);
        }

        cloud.frame_id = "body";
    }

    std::vector<Point> remove_robot_points(const std::vector<Point> &points, const Box &box)
    {
        std::vector<Point> kept;
        kept.reserve(points.size());

        for (const auto &point : points) {
            if (!box.contains(point)) {
                kept.push_back(point);
            }
        }

        return kept;
    }

    void preprocess_cloud(PointCloud &cloud, const Box &robot_body)
    {
        lidar_to_body(cloud);
        cloud.points = remove_robot_points(cloud.points, robot_body);
    }
}
