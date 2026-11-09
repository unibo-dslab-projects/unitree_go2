# Voxels that received a point since the previous MapDelta (new ones and updated ones).
# Replace any voxel you already hold with the same key.
# Apply deltas in order of seq. Start from the full map (GetMap) if you join late
# or if you detect a gap.

# stamp = newest lidar scan inside this delta (one delta holds several scans)
# frame_id = map frame (camera_init)
std_msgs/Header header

# Random number drawn when map_builder starts. A new value means a new map.
# seq starts over with each new run_id, so a different run_id means "throw away your map".
uint32 run_id

# Counts deltas within one run: 1, 2, 3, ... If you see a gap, you missed a delta.
uint64 seq

# One point per voxel: the mean of every point that fell in it, always inside the voxel,
# so floor(x / voxel_size) etc. gives back its key. In the header frame.
# Fields x y z intensity float32, num_scan_seen uint32 (20 bytes per point).
# num_scan_seen counts the scans that hit the voxel: walls score high, noise and people 1-2.
# Does not contain the unchanged part of the map.
sensor_msgs/PointCloud2 points
