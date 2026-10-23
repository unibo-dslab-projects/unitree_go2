#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace map_builder {

    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float intensity = 0.0f;
    };

    struct VoxelKey {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t z = 0;

        bool operator==(const VoxelKey& other) const;
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& key) const;
    };

    class VoxelMap {
    public:
        explicit VoxelMap(double voxel_size);
        std::size_t insert(const std::vector<Point> &points);
        std::vector<Point> last_voxel_points_added();
        std::vector<Point> all();
        std::size_t size() const;
        double voxel_size() const;

    private:
        VoxelKey key_of(const Point& point) const;
        double voxel_size_;
        std::unordered_map<VoxelKey, Point, VoxelKeyHash> voxels_;
        std::vector<Point> pending_;
    };
} // namespace map_builder
