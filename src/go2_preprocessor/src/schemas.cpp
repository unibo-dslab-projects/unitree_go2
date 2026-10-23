#include "go2_preprocessor/schemas.hpp"

namespace go2_preprocessor {
    // strict comparisons: a point exactly on the box limits counts as outside, as in CMU.
    bool Box::contains(const Point &point) const
    {
        return min_x < point.x && point.x < max_x &&
               min_y < point.y && point.y < max_y &&
               min_z < point.z && point.z < max_z;
    }
} // namespace go2_preprocessor
