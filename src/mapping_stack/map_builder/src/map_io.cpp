#include "map_builder/map_io.hpp"

#include <ctime>
#include <fstream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace map_builder {
    namespace {
        constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000;

        std::string utc_name(std::uint64_t stamp_ns)
        {
            std::time_t t = static_cast<std::time_t>(stamp_ns / kNanosecondsPerSecond);
            std::tm utc{};

            gmtime_r(&t, &utc);

            char buffer[32];
            strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &utc);
            return buffer;
        }
    } // namespace

    std::filesystem::path make_run_folder(const std::filesystem::path &output_dir, const std::uint64_t first_time_ns)
    {
        std::filesystem::path run_folder = output_dir / utc_name(first_time_ns);

        for (int suffix = 1; std::filesystem::exists(run_folder); ++suffix)
        {
            run_folder = output_dir / (utc_name(first_time_ns) + "_" + std::to_string(suffix));
        }

        std::filesystem::create_directories(run_folder);
        return run_folder;
    }

    void save_pcd(const std::filesystem::path &file, const std::vector<Point> &points)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(points.size());

        for (const auto &point : points) {
            // PCL 1.10 (Foxy) has no PointXYZI(x, y, z, intensity) constructor.
            pcl::PointXYZI pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            pcl_point.intensity = point.intensity;
            cloud.push_back(pcl_point);
        }

        if (pcl::io::savePCDFileBinary(file.string(), cloud) != 0) {
            throw std::runtime_error("Failed to save PCD file: " + file.string());
        }
    }

    void save_tum(const std::filesystem::path &file, const Trajectory &trajectory)
    {
        std::ofstream out(file);
        if (!out) {
            throw std::runtime_error("Failed to open TUM file: " + file.string());
        }

        trajectory.write_tum(out);
        if (!out) {
            throw std::runtime_error("Failed to write TUM file: " + file.string());
        }
    }
}
