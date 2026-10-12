# Recording bags on the Go2

How to record the lidar and IMU of the Go2 into ROS 2 bags, check them, copy them to the laptop and
play them back, plus the list of bags the mapping work needs.

The scripts in [tools/bags](../../tools/bags/README.md) carry out these steps: one `record_<type>.sh`
per bag of the [bag plan](#bag-plan), plus `fetch.sh` to copy and check bags on the laptop. This page
explains what they do and why.

## Overview

1. Record **on the robot computer**, not on the laptop. The lidar data reaches the robot computer over
   the robot's internal wired network (`192.168.123.x`); recording on the laptop would send every
   cloud over the lab Wi-Fi first, and every Wi-Fi hiccup would become a hole in the bag.
2. Copy the finished bag to the laptop with `rsync` over SSH.
3. Play it on the laptop inside the `foxy` container.

| Machine | Address | Notes |
|---|---|---|
| Laptop | `192.168.5.112` | Fedora, fish shell, ROS 2 Foxy only inside the distrobox container `foxy` |
| Robot computer | `192.168.5.127`, user `unitree` | Ubuntu, ROS 2 Foxy, environment loaded with `source ~/unitree_ros2/setup.sh` |

Both are on the lab Wi-Fi `dslab-robot`.

## Topics

| Topic | Type | Frame | Rate | Size |
|---|---|---|---|---|
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | `utlidar_lidar` | about 15.4 Hz | 360 to 4180 points per message, 32 bytes per point |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | `utlidar_imu` | about 250 Hz (11706 messages in 46.9 s, 2026-09-28) | a few hundred bytes per message |

For the calibration bag only, also record these if `ros2 topic list` shows them on the robot:
`/utlidar/cloud_deskewed`, `/utlidar/robot_odom`, `/tf`, `/tf_static`.

### Disk space

Measured on the first two bags (cloud only):

| Bag | Duration | Size | Rate |
|---|---|---|---|
| `walk` | 54.5 s | 101.7 MiB | about 1.9 MiB/s, about 110 MiB per minute |
| `still` | 75.9 s | 37.8 MiB | about 0.5 MiB/s |

A walking robot sees more points than a standing one. Plan with about 110 MiB per minute, so a
15-minute building walk is about 1.7 GiB. The IMU adds little on top.

## One-time setup on the laptop

Log in with a key instead of a password, and give the robot a short name.

```bash
ssh-keygen -t ed25519            # only if ~/.ssh/id_ed25519 does not exist yet
ssh-copy-id unitree@192.168.5.127
```

Add this to `~/.ssh/config`:

```
Host go2
    HostName 192.168.5.127
    User unitree
```

From now on `ssh go2` logs in. All the commands below use `go2`; without the alias, write
`unitree@192.168.5.127` instead.

## Before every session

1. Charge the robot battery. A walk that ends because the battery runs out ends the bag badly too.
2. Log in and start a `tmux` session, so that a Wi-Fi drop does not kill the recording:

   ```bash
   ssh go2
   tmux new -s rec              # later: Ctrl-b d to detach, tmux attach -t rec to come back
   ```

   If `tmux` is missing, see [Recording without tmux](#recording-without-tmux).
3. Set the robot clock. On 2026-09-30 the robot computer was about 45 hours behind, so the bags of that
   day show a start time two days early in `ros2 bag info`. The `header.stamp` inside each message was
   right, because the sensor sets it, but bag start times and file dates were wrong. From the laptop
   (fish or bash):

   ```bash
   ssh -t go2 "sudo date -s @$(date +%s)"
   ```

4. Load the environment and check that the data flows:

   ```bash
   source ~/unitree_ros2/setup.sh
   ros2 topic list | grep utlidar
   ros2 topic hz /utlidar/cloud     # about 15.4 Hz; Ctrl-C to stop
   ros2 topic hz /utlidar/imu       # write the rate down in the bag log
   df -h ~                          # free space, see Disk space above
   mkdir -p ~/bags
   ```

   Stop every `ros2 topic hz` and `ros2 topic echo` before recording: they are Python programs that
   decode every cloud and take CPU away from the recorder.

## Record

Name every bag `<date>_<scenario>_<number>`, for example `2026-10-14_loop_01`.

```bash
cd ~/bags
ros2 bag record -o 2026-10-14_loop_01 /utlidar/cloud /utlidar/imu
```

1. Keep the robot **still for 5 seconds after starting** the recorder, and again for 5 seconds before
   stopping it. Point-LIO starts by estimating gravity from the first IMU samples and assumes the
   robot is not moving then.
2. Do the scenario (see [Bag plan](#bag-plan)).
3. Stop with **Ctrl-C** and wait until the command returns to the prompt. On Ctrl-C the recorder writes
   `metadata.yaml`; a bag without it cannot be played on Foxy (see [Troubleshooting](#troubleshooting)).

A bag is a folder: `metadata.yaml` plus one `.db3` file (an SQLite database holding the messages).

### Recording without tmux

```bash
cd ~/bags
nohup ros2 bag record -o 2026-10-14_loop_01 /utlidar/cloud /utlidar/imu > 2026-10-14_loop_01.log 2>&1 &
```

`nohup` keeps the recorder alive when the SSH session drops. To stop it, send it the same signal as
Ctrl-C, then wait until it is gone:

```bash
pkill -INT -f 'ros2 bag record'
pgrep -af 'ros2 bag record'          # no output = finished
```

## Check the bag on the robot

```bash
ros2 bag info ~/bags/2026-10-14_loop_01
```

- `Duration` matches what you recorded.
- `/utlidar/cloud` count is about 15.4 x duration in seconds (for example 1169 messages in 75.9 s).
  Clearly fewer means clouds were lost.
- `/utlidar/imu` count is about the measured IMU rate x duration.

Then write a `notes.txt` inside the bag folder: place, scenario, what the robot did, tape
measurements, anything unusual (people walking by, a door closed, a stumble). The recorder ignores
extra files in the folder. Also add a line to the [bag log](#bag-log).

## Copy to the laptop

On the laptop, outside the container:

```bash
rsync -avP go2:bags/2026-10-14_loop_01 ~/Documents/Uni/thesis/bags/
```

- `-a` keeps the folder as it is, `-v` lists the files, `-P` shows progress and lets an interrupted
  copy resume where it stopped: just run the same command again.
- Several bags at once: put the remote path in quotes, otherwise fish tries to expand the `*` on the
  laptop and fails with "No matches for wildcard":

  ```bash
  rsync -avP 'go2:bags/2026-10-14_*' ~/Documents/Uni/thesis/bags/
  ```

- `scp -r go2:bags/2026-10-14_loop_01 ~/Documents/Uni/thesis/bags/` works too, but cannot resume.

Check the copy inside the container (next section) with `ros2 bag info`: the counts must match the
ones on the robot. Only then free the space on the robot:

```bash
ssh go2 rm -r bags/2026-10-14_loop_01
```

## Play on the laptop

```bash
distrobox enter foxy
source /opt/ros/foxy/setup.bash
ros2 bag info ~/Documents/Uni/thesis/bags/2026-10-14_loop_01
ros2 bag play ~/Documents/Uni/thesis/bags/2026-10-14_loop_01
```

Useful options of `ros2 bag play` on Foxy:

| Option | Effect |
|---|---|
| `-l` | loop forever, Ctrl-C to stop |
| `-r 0.5` | half speed (`2` = double) |
| `--topics /utlidar/cloud` | play only the listed topics |
| `--remap /utlidar/cloud:=/other` | publish under another name |

Foxy cannot pause playback, has no `--clock`, and has no `ros2 bag reindex`; those came in later
ROS 2 versions.

To keep a played bag away from the robot's live topics while the robot is on, run
`export ROS_LOCALHOST_ONLY=1` in every laptop terminal first.

## Bag plan

Rules for every bag:

- 5 seconds still at the start and at the end.
- Same gait mode in all bags, unless the bag is about gait.
- As few people moving around as possible: they end up in the map. If someone walks by, write it in
  `notes.txt`.
- Doors open or closed the way the floor plan should show them.
- Wipe the lidar dome before the session.

| Bag | What the robot does | Length | What it is for | Write down |
|---|---|---|---|---|
| `calib` | Stands on a flat floor, nose square to a flat wall about 1.5 m away. Nobody moves. Record the extra calibration topics too. | 20 s | Lidar and IMU mount rotation: the CMU transform leaves the floor tilted by about 25 degrees on our bags (E4) | Lidar height above the floor, lidar-to-wall distance, a photo |
| `standstill` | Stands (motors on, standing pose), nobody touches it | 120 s | Gyroscope and accelerometer bias and noise (IMU calibration) | |
| `lying` (optional) | Lies down, motors relaxed | 60 s | Bias without leg-motor vibration, to compare with `standstill` | |
| `spin` | Turns in place slowly: 2 full turns left, 5 s still, 2 full turns right | about 60 s | Gyroscope z scale and the z-to-x/y leak (CMU uses 0.15 and -0.28); Point-LIO under pure rotation | Start heading (tape arrow on the floor) |
| `line` | Walks straight along a taped line of known length, then stops | 30 to 60 s | Scale and drift: Point-LIO distance against the tape | Line length |
| `loop` | Walks a closed loop in one room or corridor and stops exactly on the start mark, facing the same way | 2 to 3 min | Drift: the gap between start and end pose of the estimated path | Start mark, room dimensions |
| `floor` | Walks the whole floor: every room and corridor that should be in the plan, back to the start mark | 5 to 15 min | The thesis dataset: map, then floor plan | Room dimensions (tape or laser meter) for the evaluation, photos |
| `speeds` (optional) | `loop` again, once slow and once fast | 2 x 2 to 3 min | Effect of speed and vibration on drift | |

## Bag log

| Bag | Place | Duration | Clouds | IMU messages | Notes |
|---|---|---|---|---|---|
| `still` (2026-09-30) | | 75.9 s | 1169 | not recorded | Cloud only; robot clock 45 h behind |
| `walk` (2026-09-30) | | 54.5 s | 840 | not recorded | Cloud only; robot clock 45 h behind |

## Troubleshooting

- **No `/utlidar/...` topics on the robot:** `source ~/unitree_ros2/setup.sh` was not run in that
  terminal (every new tmux window needs it too).
- **Far fewer clouds than 15.4 x duration:** check the rate live with `ros2 topic hz`, look for other
  programs using the CPU with `top`, and check the free disk space. Record the same scenario again
  before changing any system setting, so the two bags can be compared.
- **The recorder stops by itself:** the disk is probably full (`df -h ~`).
- **`metadata.yaml` is missing** (the recorder was killed, the robot lost power, or the SSH session
  dropped without tmux or nohup): Foxy cannot open the bag. The `.db3` file still holds the messages,
  and `ros2 bag reindex` from a newer ROS 2 can rebuild the metadata, but it is easier to record again.
- **`ros2 bag info` shows a start date in the past:** the robot clock is wrong (see
  [Before every session](#before-every-session)). The messages themselves are fine.
