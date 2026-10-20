#include "go2_preprocessor/robot_utils.hpp"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_preprocessor {
    void lidar_to_body(PointCloud &cloud, const Eigen::Isometry3f &transform)
    {
        for (auto &point : cloud.points) {
            const Eigen::Vector3f body = transform * Eigen::Vector3f(point.x, point.y, point.z);

            point.x = body.x();
            point.y = body.y();
            point.z = body.z();
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

    void preprocess_cloud(PointCloud &cloud, const Eigen::Isometry3f &lidar_to_body_transform, const Box &robot_body)
    {
        lidar_to_body(cloud, lidar_to_body_transform);
        cloud.points = remove_robot_points(cloud.points, robot_body);
    }
}
