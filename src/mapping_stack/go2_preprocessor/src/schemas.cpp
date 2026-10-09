#include "go2_preprocessor/schemas.hpp"
#include <cmath>

namespace go2_preprocessor {
    void Point::rotate_y_axis(float angle)
    {
        const float cos_angle = std::cos(angle);
        const float sin_angle = std::sin(angle);

        const float new_x = x * cos_angle + z * sin_angle;
        const float new_z = -x * sin_angle + z * cos_angle;

        x = new_x;
        z = new_z;
    }

    void Point::shift_z_axis(float shift)
    {
        z += shift;
    }
} // namespace go2_preprocessor
