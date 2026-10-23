#include "lidar_data/cloud_layout.hpp"

#include <cstdint>
#include <sstream>
#include <sensor_msgs/msg/point_field.hpp>

namespace lidar_data {

namespace {

const char *datatype_name(std::uint8_t datatype)
{
    using sensor_msgs::msg::PointField;

    switch (datatype) {
        case PointField::INT8:    return "INT8";
        case PointField::UINT8:   return "UINT8";
        case PointField::INT16:   return "INT16";
        case PointField::UINT16:  return "UINT16";
        case PointField::INT32:   return "INT32";
        case PointField::UINT32:  return "UINT32";
        case PointField::FLOAT32: return "FLOAT32";
        case PointField::FLOAT64: return "FLOAT64";
        default:                  return "UNKNOWN";
    }
}

}  // namespace

std::string describe_layout(const sensor_msgs::msg::PointCloud2 &cloud)
{
    std::ostringstream report;

    report << std::boolalpha
           << "cloud layout\n"
           << "  frame_id: " << cloud.header.frame_id << "\n"
           << "  width x height: " << cloud.width << " x " << cloud.height << " points\n"
           << "  point_step: " << cloud.point_step << " bytes\n"
           << "  row_step: " << cloud.row_step << " bytes\n"
           << "  is_dense: " << cloud.is_dense << "\n"
           << "  data: " << cloud.data.size() << " bytes\n"
           << "  fields:";

    for (const auto &field : cloud.fields) {
        report << "\n    " << field.name
               << ": offset " << field.offset
               << ", " << datatype_name(field.datatype)
               << ", count " << field.count;
    }

    return report.str();
}

}  // namespace lidar_data
