#include "go2_preprocessor/robot_utils.hpp"

#include <utility>
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

    void shorten_ranges(PointCloud &cloud, float offset)
    {
        if (offset == 0.0F) {
            return;
        }

        std::vector<Point> kept;
        kept.reserve(cloud.points.size());

        for (auto point : cloud.points) {
            const float range = Eigen::Vector3f(point.x, point.y, point.z).norm();
            if (range <= offset) {
                continue;
            }

            const float scale = (range - offset) / range;
            point.x *= scale;
            point.y *= scale;
            point.z *= scale;
            kept.push_back(point);
        }

        cloud.points = std::move(kept);
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

    void preprocess_cloud(
        PointCloud &cloud, const Eigen::Isometry3f &lidar_to_body_transform, const Box &robot_body, float range_offset)
    {
        shorten_ranges(cloud, range_offset);
        lidar_to_body(cloud, lidar_to_body_transform);
        cloud.points = remove_robot_points(cloud.points, robot_body);
    }
}
