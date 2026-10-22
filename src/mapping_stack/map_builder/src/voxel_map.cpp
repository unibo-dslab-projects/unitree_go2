#include "map_builder/voxel_map.hpp"

#include <cmath>
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

    std::size_t VoxelMap::insert(const std::vector<Point> &points)
    {
        std::size_t inserted_count = 0;
        for (const auto &point : points)
        {
            if (!is_finite(point))
            {
                continue;
            }
            const bool is_new = voxels_.try_emplace(key_of(point), point).second;
            if (is_new)
            {
                pending_.push_back(point);
                ++inserted_count;
            }
        }
        return inserted_count;
    }

    std::vector<Point> VoxelMap::last_voxel_points_added()
    {
        std::vector<Point> taken;
        taken.swap(pending_);
        return taken;
    }

    std::vector<Point> VoxelMap::all()
    {
        std::vector<Point> all_points;
        all_points.reserve(voxels_.size());
        for (const auto &pair : voxels_)
        {
            all_points.push_back(pair.second);
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

} // namespace map_builder
