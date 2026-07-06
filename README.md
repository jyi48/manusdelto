# manusdelto

Standalone bench-test rig: **Manus glove → DG5F-M hand**, with no RBY1, no
Vive, no pedal. Everything (glove reader, retargeting, hand driver, GUI) runs
on one PC.

```
Manus glove --(ManusSDK)--> manus_data_publisher --(ManusGlove msg)--> manus_tesollo_node
                                                                            |
                                                           /{hand_ns}/{lj,rj}_dg_pospid/reference
                                                                            v
                                                            dg5f_driver (ros2_control, Modbus TCP)
                                                                            v
                                                                       DG5F-M hand(s)
```

This repo vendors the packages needed for that path so it builds standalone,
without checking out the full `teleop` or `hw-core` workspaces:

| Package | Source | Role |
|---|---|---|
| `manus_ros2` | `teleop/src/input/manus_ros2` | ManusSDK client, publishes `/manus_glove_0`/`_1` |
| `manus_ros2_msgs` | `teleop/src/msgs/manus_ros2_msgs` | Glove message definitions |
| `manus_tesollo` | `teleop/src/core/manus_tesollo` | Retargeting (ergo / ik / dex / dex_vector) |
| `dg5f_driver` | `hw-core/tesollo_ros2/dg5f_ros2/dg5f_driver` | ros2_control hardware interface + PID controllers |
| `dg5f_description` | `hw-core/tesollo_ros2/dg5f_ros2/dg5f_description` | URDF/meshes for `dg5f_driver` |
| `delto_tcp_comm` | `hw-core/tesollo_ros2/dg_common/dg_tcp_comm` | Modbus TCP client used by `dg5f_driver` |
| `delto_hardware` | `hw-core/tesollo_ros2/dg_common/dg_hardware` | `hardware_interface::SystemInterface` impl |
| `manusdelto_bringup` | new | Top-level launch file |
| `manusdelto_gui` | new | Minimal PySide6 control panel |

These are **vendored copies** (not submodules) — if `manus_tesollo`'s
retargeting logic changes upstream in `teleop`, or the DG5F driver changes in
`hw-core`, re-copy the relevant package here manually.

## Prerequisites

- ROS 2 (Humble or later), `colcon`
- ManusSDK binary (for `manus_ros2`)
- `ros2_control`, `ros2_controllers`, `pid_controller`, `joint_state_broadcaster`
- Optional, for extra retarget modes: `pinocchio` (`ik`), `dex_retargeting` + `nlopt` (`dex` / `dex_vector`) — if missing, `manus_tesollo` degrades gracefully to `ergo` only
- PySide6 (for `manusdelto_gui`)

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Network

Test-rig defaults (override via launch args if your setup differs):

| Hand | IP |
|---|---|
| Left | `192.168.1.151` |
| Right | `192.168.1.152` |

## Launch

```bash
# Both hands (default)
ros2 launch manusdelto_bringup manusdelto.launch.py

# Single hand
ros2 launch manusdelto_bringup manusdelto.launch.py hand_ns:=dg5f_left delto_ip:=192.168.1.151
ros2 launch manusdelto_bringup manusdelto.launch.py hand_ns:=dg5f_right delto_ip:=192.168.1.152

# Override both-hand IPs
ros2 launch manusdelto_bringup manusdelto.launch.py \
    dg5f_left_ip:=192.168.1.151 dg5f_right_ip:=192.168.1.152

# ik mode at startup (requires pinocchio)
ros2 launch manusdelto_bringup manusdelto.launch.py use_ik:=true

# Without the GUI
ros2 launch manusdelto_bringup manusdelto.launch.py use_gui:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `hand_ns` | `dg5f_both` | `dg5f_both`, `dg5f_left`, or `dg5f_right` — which `dg5f_driver` launch file to include |
| `dg5f_left_ip` / `dg5f_left_port` | `192.168.1.151` / `502` | Left hand (used when `hand_ns:=dg5f_both`) |
| `dg5f_right_ip` / `dg5f_right_port` | `192.168.1.152` / `502` | Right hand (used when `hand_ns:=dg5f_both`) |
| `delto_ip` / `delto_port` | `192.168.1.151` / `502` | Used when `hand_ns:=dg5f_left` or `dg5f_right` |
| `use_ik` | `false` | Start `manus_tesollo` in `ik` mode instead of `ergo` |
| `orientation_weight` | `1.0` | IK orientation task weight |
| `use_gui` | `true` | Launch `manusdelto_gui` |

## GUI

`manusdelto_gui` is a trimmed-down control panel — just what this bench rig
needs (full RBY1/recording control lives in teleop's `scm_gui`):

- **Retarget mode** — Ergo / IK / DexPilot / Vector radios (publishes to
  `/manus_tesollo/retarget_mode`)
- **Pause Stream** — freezes the last output at `/manus_tesollo/pause`
- **Mirror mode** — swaps which glove drives which hand
  (`/teleop/mirror_mode`)
- **Recalibrate** — 2-phase rest/fist ROM calibration
  (`/manus_tesollo/calibrate`); watch the progress bar/status label for the
  "open hand" → "make a fist" prompts

## Power note

This rig was built to test DG5F current draw under a dedicated SMPS instead
of the RBY1 backpack supply (24V/8A backpack vs. DG5F's up to 24V/10A max
draw per hand) — see project notes for the motivating incident.
