#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace map_builder {

    // a point from Point-LIO, or a cube of the map (then x, y, z, intensity are the mean of its points)
    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float intensity = 0.0f;
        // scans that hit the cube: a wall seen from many places scores high, noise and people score 1-2;
        // 0 on points from Point-LIO, the map fills it
        std::uint32_t num_scan_seen = 0;
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

    // running sums instead of the points: memory stays fixed however long the robot walks
    struct VoxelStatistics {
        // double: a float sum of a thousand points 10 m away loses millimetres
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_intensity = 0.0;
        std::uint32_t point_count = 0;
        std::uint32_t num_scan_seen = 0;
        std::uint64_t last_scan_index = 0;
        bool is_pending = false;
    };

    class VoxelMap {
    public:
        explicit VoxelMap(double voxel_size);
        // scan_index must differ between scans, so a cube counts each scan once; returns the new cubes
        std::size_t insert(const std::vector<Point> &points, std::uint64_t scan_index);
        // cubes that received a point since the last call, each once, with its current mean
        std::vector<Point> take_changed_voxels();
        std::vector<Point> all() const;
        std::size_t size() const;
        double voxel_size() const;

    private:
        VoxelKey key_of(const Point& point) const;
        Point mean_of(const VoxelKey& key, const VoxelStatistics& statistics) const;
        void mark_pending(const VoxelKey& key, VoxelStatistics& statistics);

        double voxel_size_;
        std::unordered_map<VoxelKey, VoxelStatistics, VoxelKeyHash> voxels_;
        // keys, not points: the delta must carry the mean at send time, not at insert time
        std::vector<VoxelKey> pending_;
    };
} // namespace map_builder
