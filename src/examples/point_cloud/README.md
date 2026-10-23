# point_cloud

Little project to build a typed point cloud from the lidar data of the unitree go2 robot.

The node `cloud_builder` subscribes to `/utlidar/cloud`, converts every message into the project's own
`PointCloud` type (`include/point_cloud/point_cloud.hpp`, no ROS inside), converts it back to a ROS
message and publishes it on `/cloud_builder/cloud`. Every 2 seconds it logs how many points the last
cloud had. A message that lacks one of the six expected fields is skipped and logged as an error.

For now the cloud goes out unchanged: the `TODO` in `src/cloud_builder.cpp` is where filtering or
accumulation will go.

## Requirements

- ROS 2 Foxy. The robot computer has it; on the laptop it lives in the distrobox container `foxy`.
- RViz2 to look at the cloud (installed in the `foxy` container).
- Every command runs from the repository root, which is also the colcon workspace.

## Run on the laptop (recorded bag)

Terminal 1, the node:

```bash
distrobox enter foxy
cd ~/Documents/Uni/thesis/unitree_go2
source /opt/ros/foxy/setup.bash
colcon build --packages-select point_cloud
source install/setup.bash
ros2 run point_cloud cloud_builder
```

Terminal 2, RViz2:

```bash
distrobox enter foxy
source /opt/ros/foxy/setup.bash
QT_QPA_PLATFORM=xcb rviz2             # the container's Qt cannot use the host's Wayland
```

Terminal 3, the bag (`still` = robot standing, `walk` = robot walking):

```bash
distrobox enter foxy
source /opt/ros/foxy/setup.bash
ros2 bag play ~/Documents/Uni/thesis/bags/walk
```

The bag plays once and stops.

## Set up RViz2

1. In **Global Options**, set **Fixed Frame** by typing `utlidar_lidar`. There is no TF tree, so the
   frame is not in the drop-down list. A typo makes RViz2 drop every cloud with
   `Message Filter dropping message ... reason 'Unknown'`.
2. Click **Add**, tab **By topic**, choose `/cloud_builder/cloud` > **PointCloud2**.

The points are in the lidar's own frame, so the cloud moves with the robot and nothing accumulates.

## Run live: node on the robot, RViz2 on the laptop

The lidar sits on the robot's internal wired network (`192.168.123.x`), the laptop on the lab Wi-Fi
(`192.168.5.x`). The node runs on the robot computer, which is on both networks:
`~/unitree_ros2/setup.sh` binds CycloneDDS to both interfaces. The laptop must use CycloneDDS too.

On the robot:

```bash
ssh unitree@192.168.5.127
cd ~/unitree_go2
git pull
source ~/unitree_ros2/setup.sh        # ROS 2 Foxy + CycloneDDS on both networks
colcon build --packages-select point_cloud
source install/setup.bash
ros2 run point_cloud cloud_builder
```

On the laptop:

```bash
distrobox enter foxy
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
QT_QPA_PLATFORM=xcb rviz2
```

Then set up RViz2 as above.

## Expected output

```
[INFO] [...] [cloud_builder]: published cloud with 650 points
[INFO] [...] [cloud_builder]: published cloud with 421 points
[INFO] [...] [cloud_builder]: published cloud with 543 points
```

One line every 2 seconds. No output means no cloud has arrived.

## Known issue (live run, 2026-09-30)

On the real robot the clouds reached `cloud_builder` only in short bursts, with about 70 seconds of
silence in between (6 log lines in about 90 seconds instead of about 45). The lidar itself published at
about 15 Hz the whole time. The cause is not found yet; the suspect is how CycloneDDS picks between the
wired and the Wi-Fi address. Bag playback on the laptop is not affected.
