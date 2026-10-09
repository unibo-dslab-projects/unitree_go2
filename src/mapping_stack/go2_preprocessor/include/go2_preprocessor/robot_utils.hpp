#pragma once

#include <vector>
#include "schemas.hpp"

namespace go2_preprocessor {
    std::vector<Point> remove_robot_points(const std::vector<Point> &points, const Point max_point, const Point min_point);
}
