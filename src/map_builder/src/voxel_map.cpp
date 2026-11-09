#include "map_builder/voxel_map.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace map_builder {
    namespace {
        // large primes from Teschner et al. 2003, "Optimized Spatial Hashing for
        // Collision Detection of Deformable Objects"
        constexpr std::size_t kPrimeX = 73856093;
        constexpr std::size_t kPrimeY = 19349663;
        constexpr std::size_t kPrimeZ = 83492791;

        bool is_finite(const Point &point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        // receivers find the cube again with floor(value / voxel_size) on the float they get, and
        // rounding the mean to float can land it on the edge of the next cube
        float inside_voxel(double mean, std::int32_t key, double voxel_size)
        {
            float value = static_cast<float>(mean);
            while (std::floor(value / voxel_size) > key) {
                value = std::nextafter(value, -std::numeric_limits<float>::infinity());
            }
            while (std::floor(value / voxel_size) < key) {
                value = std::nextafter(value, std::numeric_limits<float>::infinity());
            }
            return value;
        }

    } // namespace

    bool VoxelKey::operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    std::size_t VoxelKeyHash::operator()(const VoxelKey &key) const
    {
        // cast necessary cause multiplying signed numbers can overflow
        // also called 'spatial hashing'
        return (static_cast<std::size_t>(key.x) * kPrimeX) ^
                static_cast<std::size_t>(key.y) * kPrimeY ^
                static_cast<std::size_t>(key.z) * kPrimeZ;
    }

    VoxelMap::VoxelMap(double voxel_size)
        : voxel_size_(voxel_size)
    {
        if (!(voxel_size > 0.0))
        {
            throw std::invalid_argument("voxel_size must be positive");
        }
    }

    std::size_t VoxelMap::insert(const std::vector<Point> &points, std::uint64_t scan_index)
    {
        std::size_t new_voxel_count = 0;
        for (const auto &point : points)
        {
            if (!is_finite(point))
            {
                continue;
            }
            const VoxelKey key = key_of(point);
            const auto [entry, is_new] = voxels_.try_emplace(key);
            VoxelStatistics &statistics = entry->second;

            statistics.sum_x += point.x;
            statistics.sum_y += point.y;
            statistics.sum_z += point.z;
            statistics.sum_intensity += point.intensity;
            ++statistics.point_count;
            // every point moves the mean, so the cube goes out with the next delta
            mark_pending(key, statistics);

            if (is_new || statistics.last_scan_index != scan_index)
            {
                statistics.last_scan_index = scan_index;
                ++statistics.num_scan_seen;
            }
            if (is_new)
            {
                ++new_voxel_count;
            }
        }
        return new_voxel_count;
    }

    std::vector<Point> VoxelMap::take_changed_voxels()
    {
        std::vector<Point> changed;
        changed.reserve(pending_.size());
        for (const VoxelKey &key : pending_)
        {
            VoxelStatistics &statistics = voxels_.at(key);
            statistics.is_pending = false;
            changed.push_back(mean_of(key, statistics));
        }
        pending_.clear();
        return changed;
    }

    std::vector<Point> VoxelMap::all() const
    {
        std::vector<Point> all_points;
        all_points.reserve(voxels_.size());
        for (const auto &[key, statistics] : voxels_)
        {
            all_points.push_back(mean_of(key, statistics));
        }
        return all_points;
    }

    std::size_t VoxelMap::size() const
    {
        return voxels_.size();
    }

    double VoxelMap::voxel_size() const {
        return voxel_size_;
    }

    VoxelKey VoxelMap::key_of(const Point &point) const
    {
        // floor cause a cast rounds toward 0, so -0.03 and +0.03 both map to 0
        // floor puts -0.03 to -1
        return VoxelKey{
            static_cast<std::int32_t>(std::floor(point.x / voxel_size_)),
            static_cast<std::int32_t>(std::floor(point.y / voxel_size_)),
            static_cast<std::int32_t>(std::floor(point.z / voxel_size_)),
        };
    }

    Point VoxelMap::mean_of(const VoxelKey &key, const VoxelStatistics &statistics) const
    {
        const double point_count = statistics.point_count;
        Point mean;
        mean.x = inside_voxel(statistics.sum_x / point_count, key.x, voxel_size_);
        mean.y = inside_voxel(statistics.sum_y / point_count, key.y, voxel_size_);
        mean.z = inside_voxel(statistics.sum_z / point_count, key.z, voxel_size_);
        mean.intensity = static_cast<float>(statistics.sum_intensity / point_count);
        mean.num_scan_seen = statistics.num_scan_seen;
        return mean;
    }

    void VoxelMap::mark_pending(const VoxelKey &key, VoxelStatistics &statistics)
    {
        // a cube hit by many points inside one delta period is sent once
        if (!statistics.is_pending)
        {
            statistics.is_pending = true;
            pending_.push_back(key);
        }
    }

} // namespace map_builder
