# lidar_data

Little project to get lidar data from the unitree go2 robot, in order to understand how the data are structured.

The node `cloud_inspector` subscribes to `/utlidar/cloud` (the point cloud of the Go2 L1 lidar) and prints
the layout of the message every 2 seconds: frame, number of points, bytes per point, and the name, offset
and type of every field. It does not change or republish anything.

## Requirements

- ROS 2 Foxy. The robot computer has it; on the laptop it lives in the distrobox container `foxy`.
- Every command runs from the repository root, which is also the colcon workspace.

## Run on the robot (live data)

```bash
ssh unitree@192.168.5.127
cd ~/unitree_go2
git pull
source ~/unitree_ros2/setup.sh        # ROS 2 Foxy + CycloneDDS, as set up by Unitree
colcon build --packages-select lidar_data
source install/setup.bash
ros2 run lidar_data cloud_inspector
```

Stop it with `Ctrl+C`.

## Run on the laptop (recorded bag)

Terminal 1, the node:

```bash
distrobox enter foxy
cd ~/Documents/Uni/thesis/unitree_go2
source /opt/ros/foxy/setup.bash
colcon build --packages-select lidar_data
source install/setup.bash
ros2 run lidar_data cloud_inspector
```

Terminal 2, the bag (`still` = robot standing, `walk` = robot walking):

```bash
distrobox enter foxy
source /opt/ros/foxy/setup.bash
ros2 bag play ~/Documents/Uni/thesis/bags/walk
```

The bag plays once and stops.

## Expected output

```
[INFO] [...] [cloud_inspector]: cloud layout
  frame_id: utlidar_lidar
  width x height: 650 x 1 points
  point_step: 32 bytes
  row_step: 20800 bytes
  is_dense: true
  data: 20800 bytes
  fields:
    x: offset 0, FLOAT32, count 1
    y: offset 4, FLOAT32, count 1
    z: offset 8, FLOAT32, count 1
    intensity: offset 16, FLOAT32, count 1
    ring: offset 20, UINT16, count 1
    time: offset 24, FLOAT32, count 1
```

The number of points changes from message to message (a few hundred to a few thousand).
No output means no cloud has arrived.

## Record a new bag

On the robot:

```bash
source ~/unitree_ros2/setup.sh
ros2 bag record -o walk /utlidar/cloud
```

Stop with `Ctrl+C`, then copy the bag folder to the laptop:

```bash
scp -r unitree@192.168.5.127:~/walk ~/Documents/Uni/thesis/bags/
```

Add `/utlidar/imu` to the record command if the bag will feed a lidar-inertial SLAM later: the
existing `still` and `walk` bags hold only `/utlidar/cloud`.
