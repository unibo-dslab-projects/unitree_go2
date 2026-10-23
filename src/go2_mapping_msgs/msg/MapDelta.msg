# Voxels added to the map since the previous MapDelta.
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

# One point per new voxel, in the header frame.
# Fields x y z intensity, float32 (16 bytes per point).
# Does not contain the old map, only what is new.
sensor_msgs/PointCloud2 points
