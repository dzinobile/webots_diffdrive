# webots_multirobot_nav2_slam

A ROS 2 package that recreates the functionality of `webots_ros2_turtlebot3` — Webots simulation with Nav2 autonomous navigation and Cartographer SLAM — but extended to support **multiple custom differential drive robots** operating simultaneously in a shared environment, each with its own fully isolated navigation stack.

---

## Overview

The original `webots_ros2_turtlebot3` package provides a single TurtleBot3 Burger robot in Webots with Nav2 and SLAM. This repository reproduces that capability and adds:

- **Multi-robot support** — spawn N robots at launch time, each with isolated namespaces, tf trees, costmaps, and planners
- **Custom robot model** — a custom differential drive robot (`CustomBot`) with redesigned geometry, mesh-based visuals, and ball-joint caster wheels
- **Single-file configuration** — template-based URDF, ros2_control YAML, Nav2 params, Lua configs, and RViz configs; no per-robot duplicates
- **Dynamic world generation** — robots are injected into the Webots world file at launch time, spaced 0.5 m apart along the Y axis

---

## Package Structure

```
webots_multirobot_nav2_slam/
├── webots_diffdrive/                   # Core simulation + driver package
│   ├── launch/
│   │   ├── custom_bot_launch.launch.py       # Main multi-robot launch file
│   │   └── turtlebot_multi_launch.launch.py  # Reference single TurtleBot3 launch
│   ├── resource/
│   │   ├── custom_bot.urdf                   # CustomBot URDF template (uses ROBOT_NS)
│   │   ├── custom_bot_ros2control.yml        # ros2_control config template
│   │   ├── custom_bot_nav2_params.yaml       # Nav2 parameter template
|   |   ├── turtlebot_multi.urdf              # TurtleBot3Burger URDF template (uses ROBOT_NS)
│   │   ├── turtlebot_multi_ros2control.yml   # TurtleBot3Burger ros2_control config template
│   │   ├── turtlebot_multi_nav2_params.yaml  # TurtleBot3Burger nav2 paramter template
│   │   └── turtlebot_example_map.*           # Pre-built map for Nav2
│   └── worlds/
│       ├── custom_world.wbt                  # Apartment environment (custom bot)
│       ├── turtlebot_multi_wbt               # Apartment environment (turtlebot)
│       ├── CustomBot.proto                   # Custom robot PROTO definition
│       ├── TurtlebotProto.proto              # Original TurtleBot3 PROTO 
│       └── meshes/                           # DAE/STL mesh files for CustomBot
│
├── turtlebot3_navigation2_custom/            # Nav2 launch + parameter templates
│   ├── launch/navigation2.launch.py          # Namespaced Nav2 bringup
│   ├── param/
│   │   ├── burger.yaml                       # Nav2 param template (ROBOT_NS placeholder)
│   │   └── humble/burger.yaml                # ROS Humble variant
│   ├── map/                                  # Navigation map (map.pgm + map.yaml)
│   └── rviz/tb3_navigation2.rviz             # Nav2 RViz config template
│
└── turtlebot3_cartographer_custom/          # Cartographer SLAM launch + config
    ├── launch/
    │   ├── cartographer.launch.py            # Namespaced Cartographer bringup
    │   └── occupancy_grid.launch.py          # Occupancy grid node
    ├── config/turtlebot3_lds_2d.lua          # Cartographer 2D SLAM config template
    └── rviz/tb3_cartographer.rviz            # Cartographer RViz config template
```

---

## Dependencies

| Category | Packages |
|---|---|
| ROS2 distro | `Humble` |
| Simulation | `webots_ros2_driver`, `webots_ros2_control` |
| Navigation | `navigation2`, `nav2_bringup`, all `nav2_*` sub-packages |
| SLAM | `cartographer_ros` |
| Control | `ros2_control`, `ros2_controllers` (`diff_drive_controller`) |
| Visualization | `rviz2`, `robot_state_publisher` |
| TF | `tf2`, `tf2_ros` |

Install ROS 2 dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Robot Model
### CustomBot

`CustomBot` is a custom differential drive robot defined in `worlds/CustomBot.proto`.

| Parameter | Value |
|---|---|
| Wheel separation | 110 mm (hub-to-hub) |
| Wheel radius | 25 mm |
| Drive motors | 2× RotationalMotor, max 20 rad/s (0.5 m/s) |
| Caster wheels | 2× BallJoint + 30 mm sphere (front and rear) |
| LiDAR | RobotisLDS-01, 5 Hz, 360° |
| IMU | Accelerometer + Gyro + InertialUnit, 20 Hz |

The robot mesh files (`.dae`) live in `worlds/meshes/` and are automatically copied alongside the generated world file at launch time so that Webots can resolve relative PROTO/mesh paths correctly.

---
### Turtlebot

`TurtleBot3Burger` is the default turtlebot from the webots_ros2_turtlebot3 package.

| Parameter | Value |
|---|---|
| Wheel separation | 160 mm (hub-to-hub) |
| Wheel radius | 33 mm |
| Drive motors | 2× RotationalMotor, max 6.67 rad/s (0.2 m/s) |
| Caster wheels | 2× BallJoint + 4 mm sphere (front) |
| LiDAR | RobotisLDS-01, 5 Hz, 360° |
| IMU | Accelerometer + Gyro + InertialUnit, 20 Hz |

---

## Usage

### Simulation only (no navigation)
#### Turtlebot
```bash
ros2 launch webots_diffdrive turtlebot_multi_launch.launch.py
```
#### CustomBot
```bash
ros2 launch webots_diffdrive custom_bot_launch.launch.py 
```

### Multiple robots (simulation only)
#### Turtlebot
```bash
ros2 launch webots_diffdrive turtlebot_multi_launch.launch.py num_robots:=3
```
#### CustomBot
```bash
ros2 launch webots_diffdrive custom_bot_launch.launch.py num_robots:=3
```

Robots spawn at `x=6.36 m`, separated by `0.5 m` on the Y axis:
- `robot1` → `y=0.0`
- `robot2` → `y=0.5`
- `robot3` → `y=1.0`

### Simulation + SLAM (Cartographer)
#### Turtlebot
```bash
ros2 launch webots_diffdrive turtlebot_multi_launch.launch.py num_robots:=3 slam:=true
```
#### CustomBot
```bash
ros2 launch webots_diffdrive custom_bot_launch.launch.py num_robots:=3 slam:=true
```

### Simulation + Navigation (Nav2 with pre-built map)
#### Turtlebot
```bash
ros2 launch webots_diffdrive turtlebot_multi_launch.launch.py num_robots:=3 nav:=true 
```
#### CustomBot
```bash
ros2 launch webots_diffdrive custom_bot_launch.launch.py num_robots:=3 nav:=true
```

### All launch arguments

| Argument | Default | Description |
|---|---|---|
| `num_robots` | `1` | Number of robots to spawn |
| `nav` | `false` | Launch Nav2 for each robot |
| `slam` | `false` | Launch Cartographer SLAM for each robot |
| `use_sim_time` | `true` | Use Webots simulation clock |
| `world` | `custom_world.wbt` | World file (from `worlds/`) |
| `mode` | `realtime` | Webots startup mode (`realtime`, `fast`, `pause`) |

---

## Multi-Robot Architecture

Each robot `i` gets a fully isolated ROS 2 namespace `/robot{i}` containing:

- **TF tree**: `robot{i}/map → robot{i}/odom → robot{i}/base_link → robot{i}/LDS-01`
- **Topics**: `/robot{i}/scan`, `/robot{i}/cmd_vel`, `/robot{i}/odom`, `/robot{i}/imu`
- **Nav2 stack**: independent AMCL, costmaps, planner, controller, and BT navigator
- **ros2_control**: independent `diffdrive_controller` and `joint_state_broadcaster`

All per-robot configuration files (URDF, ros2_control YAML, Nav2 YAML, Lua, RViz) are generated from single template files at launch time by substituting the `ROBOT_NS` placeholder with `robot{i}`. Temporary files are written to `/tmp` and cleaned up by the OS after the session.

---

---

## Known Limitations

- Nav2 and SLAM cannot be run simultaneously (`nav:=true slam:=true` is not supported; use one at a time)
- Cartographer SLAM quality depends on LiDAR range and environment complexity; the LDS-01 has a 3.5 m max range which limits large open spaces
