# Bag recording scripts

Scripts that carry out [docs/bags/README.md](../../docs/bags/README.md): one script per bag type
from the bag plan, so every bag of a type has the same topics, checks and name.

| Script | Runs on | Does |
|---|---|---|
| `push.sh` | laptop | copies these scripts to the robot (`~/tools/bags`) |
| `record_<type>.sh` | robot, inside tmux | checks, records, checks the bag, writes `notes.txt` |
| `fetch.sh` | laptop | copies bags to the laptop and checks that every message arrived |
| `common.sh` | (sourced) | the shared logic; the type scripts only declare their data |

## Bag types

| Script | Topics | Stops |
|---|---|---|
| `record_calib.sh` | cloud, imu, cloud_base, sportmodestate (+ cloud_deskewed, robot_odom, tf, tf_static if present) | by itself after 20 s |
| `record_standstill.sh` | cloud, imu, cloud_base, sportmodestate | by itself after 120 s |
| `record_lying.sh` | cloud, imu, cloud_base, sportmodestate | by itself after 60 s |
| `record_spin.sh` | cloud, imu, cloud_base, sportmodestate | Ctrl-C, or after `[seconds]` |
| `record_line.sh`, `record_loop.sh`, `record_floor.sh`, `record_speeds.sh` | cloud, imu, sportmodestate (+ robot_odom if present) | Ctrl-C, or after `[seconds]` |

The robot stands still in the first three, so their length is part of the measurement (bias is
averaged over a known time) and a timer ends them. In the others the robot walks, and the bag ends
when the task does (back on the start mark, end of the tape), which only the operator knows.

- `/sportmodestate` holds the gyroscope and accelerometer of the robot body, the reference for the
  lidar IMU rotation and biases.
- `/utlidar/cloud_base` is the cloud already turned into the body frame by the Unitree firmware; it
  checks the lidar extrinsic in every calibration session (a firmware update could change it).
- `/utlidar/robot_odom` is Unitree's own odometry, to compare Point-LIO's drift with.

## Use

On the laptop, once and after every change to the scripts:

```bash
tools/bags/push.sh
```

On the robot (`ssh go2`, then `tmux new -s rec`):

```bash
~/tools/bags/record_spin.sh         # stop with Ctrl-C
~/tools/bags/record_spin.sh 60      # stops by itself after 60 s; Ctrl-C still stops it earlier
~/tools/bags/record_spin.sh --help
```

The optional length in seconds works for every type: on the walking types it lets one person record
and drive the robot with the remote without reaching for the keyboard; on the others it replaces the
default length (for example `record_standstill.sh 300`). It must be more than 10 s. On a walking
type with a length, the screen says GO after the first 5 s and KEEP THE ROBOT STILL for the last 5 s,
so plan the scenario to end before then.

The script:

1. loads ROS 2 if needed (`~/unitree_ros2/setup.sh`, override with `ROS_SETUP`);
2. stops if another recorder runs, a required topic is missing, the computer clock and the sensor
   clock disagree by more than 5 s, `/utlidar/cloud` is far from 15.4 Hz, or the disk is too full;
3. names the bag `~/bags/<date>_<type>_<NN>`, never overwriting one;
4. waits for Enter, then records. The first 5 s say KEEP THE ROBOT STILL. Fixed-length bags stop by
   themselves; for the others keep the robot still 5 s at the end, then press Ctrl-C once and wait;
5. checks `metadata.yaml` and the message counts against the expected rates;
6. asks for the notes of that type and saves them to `notes.txt` in the bag, next to `record.log`;
7. prints the row for the bag log and the `fetch.sh` command.

Back on the laptop, outside the container:

```bash
tools/bags/fetch.sh 2026-10-14_spin_01
tools/bags/fetch.sh '2026-10-14_*'                 # quoted: the robot expands the pattern
tools/bags/fetch.sh --check ../bags/2026-10-14_spin_01   # check a bag already on the laptop
```

`fetch.sh` prints the command to free the space on the robot once a copy is complete; it never
deletes anything itself.

## Overrides

| Variable | Default | Used by |
|---|---|---|
| `BAG_DIR` | `~/bags` | record scripts |
| `ROS_SETUP` | `~/unitree_ros2/setup.sh` | record scripts |
| `ALLOW_CLOCK_MISMATCH=1` | unset | record scripts: record although the clocks disagree |
| `ROBOT` | `go2` (the SSH alias) | `push.sh`, `fetch.sh` |
| `REMOTE_BAG_DIR` | `bags` | `fetch.sh` |
| `LOCAL_BAG_DIR` | `bags` next to this repository | `fetch.sh` |
| `REMOTE_TOOLS_DIR` | `tools/bags` | `push.sh` |

## Notes

- Playing a bag with `/sportmodestate` on the laptop needs the `unitree_go` messages in the
  container; without them, play the other topics only (`--topics /utlidar/cloud /utlidar/imu`).
- To add a bag type, copy a `record_<type>.sh`, change its variables, and add the expected rate of
  any new topic to `EXPECTED_HZ` in `common.sh`.
