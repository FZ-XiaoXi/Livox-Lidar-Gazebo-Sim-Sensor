<!-- SPDX-FileCopyrightText: 2026 FZ_XiaoXi -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

<div align="center">

# Livox Lidar Gazebo Sim Sensor

**English | [简体中文](README.md)**

A Livox lidar simulation plugin for ROS 2 Jazzy and Gazebo Harmonic.

<p>
  <a href="https://ubuntu.com/"><img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04"></a>
  <a href="https://www.ros.org/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"></a>
  <a href="https://gazebosim.org/"><img src="https://img.shields.io/badge/Gazebo-Harmonic-FFB81C?logo=gazebo&logoColor=111827" alt="Gazebo Harmonic"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0%20%2B%20MIT-64748b" alt="Apache-2.0 and MIT"></a>
</p>

</div>

This project adapts scan-pattern data and ideas from [Livox laser simulation](https://github.com/Livox-SDK/livox_laser_simulation) for Gazebo Harmonic. The repository is a ROS 2 workspace containing a reusable Gazebo Sim plugin and a runnable example.

![Livox Gazebo simulation](resources/1.jpg)

> The maintainer currently has access to a Mid-70 only. Other models have not been validated against physical devices; their dimensions, mass, meshes, CSV scan patterns, and simulated output are provided for reference. Horizon and Mid-40 are no longer supported on the manufacturer's website, so this project provides no default mesh for them and does not guarantee their visual geometry, mass, collision shape, or point-cloud results.

## Quick Links

| Topic | Section |
| --- | --- |
| Build and run | [User Guide](#user-guide) |
| Model macros, parameters, and multi-lidar setup | [Using the Plugin](#using-the-plugin) |
| Interactive two-lidar scene | [Example](#example) |
| Plugin architecture | [Developer Guide](#developer-guide) |
| Known limitations | [Conventions and Limitations](#conventions-and-limitations) |

## Repository Layout

```text
gz_ros_ws/
├── README.md
├── README.en.md
├── LICENSE
├── NOTICE
└── src/
    ├── livox_lidar_gz_sensor/      # Gazebo Sim plugin package
    │   ├── include/                # Headers
    │   ├── src/                    # Plugin and sensor implementation
    │   ├── urdf/                   # Sensor xacro macros
    │   ├── scan_mode/              # Ray scan-pattern data
    │   └── meshes/                 # Lidar model meshes
    └── livox_lidar_example/        # Simulation example package
        ├── worlds/                 # Two example robots, walls, and ground
        ├── config/                 # Bridge and optional RViz configuration
        ├── urdf/                   # Optional RobotModel description
        └── launch/                 # ROS 2 launch file
```

# User Guide

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ament_cmake`, `colcon`, and `xacro`
- Example runtime packages: `ros_gz_sim`, `ros_gz_bridge`, `rviz2`, and `robot_state_publisher`
- Raycast physics backend: the plugin currently uses DART with Bullet. Gazebo ODE does not support the raycast query used by this plugin.

## Build

Run these commands from the workspace root:

```bash
cd ~/gz_ros_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic

CCACHE_DISABLE=1 colcon build --symlink-install \
  --packages-select livox_lidar_gz_sensor livox_lidar_example

source install/setup.bash
```

`CCACHE_DISABLE=1` is only needed if the local ccache directory is not writable; omit it in a normal environment.

## Example

The example in [`src/livox_lidar_example`](src/livox_lidar_example) demonstrates plugin loading, ray hits, dynamic TF, and the ROS bridge. It includes:

- Two cone-shaped example robots, each with a lidar;
- One model-scoped `LivoxLidarSystem` plugin instance per robot;
- Four walls and a ground plane;
- `ros_gz_bridge` mappings for Gazebo and ROS 2 point clouds and TF;
- Gravity disabled, so interactive entities do not fall and can be moved to test raycasts.

The optional RViz RobotModel and meshes help visualize model placement. They are not required by the plugin and are not part of its sensor algorithm.

Start the example:

```bash
./src/livox_lidar_example/run_livox_example.sh
```

Launch arguments:

```bash
./src/livox_lidar_example/run_livox_example.sh gz_verbosity:=4
```

| Argument | Default | Description |
| --- | --- | --- |
| `gz_verbosity` | `3` | Gazebo log level; `4` enables more detailed output |

Example ROS topics and TF frames:

```text
/livox_left/points       # Mid-360 point cloud
/livox_right/points      # Mid-70 point cloud
world -> livox_left_frame
world -> livox_right_frame
```

## Using the Plugin

The plugin library is:

```text
liblivox_lidar_gz_sensor.so
```

Minimal SDF plugin element:

```xml
<plugin
  filename="liblivox_lidar_gz_sensor.so"
  name="livox_lidar_system::LivoxLidarSystem"/>
```

The plugin can be attached to a `<model>` or a `<world>`:

- A model-scoped plugin only processes Livox custom sensors owned by that model and its descendants. This is useful for configuring each robot independently.
- A world-scoped plugin processes Livox custom sensors across the world and is useful for centralized management.
- Do not let both a world-scoped and a model-scoped plugin own the same sensor; that can result in duplicate sensor instances, raycasts, and publications.
- The environment hook only configures `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_SIM_RESOURCE_PATH`. It does not insert a plugin element into SDF automatically.

### Model Macros

Every macro requires a unique `link_name`. The sensor name, frame, and point-cloud topic are derived from it:

```text
sensor name = ${link_name}_sensor
frame_id    = ${link_name}_frame
topic       = /${link_name}/points
```

| Model | Xacro file | Link macro | Default mesh | Status |
| --- | --- | --- | --- | --- |
| Mid-70 | `urdf/livox_mid70.xacro` | `livox_mid70_link` | `mid70.dae` | Available for validation |
| Mid-360 | `urdf/livox_mid360.xacro` | `livox_mid360_link` | `mid360.dae` | Reference only; not physically validated |
| Avia | `urdf/livox_avia.xacro` | `livox_avia_link` | `avia.dae` | Reference only; not physically validated |
| HAP | `urdf/livox_HAP.xacro` | `livox_hap_link` | `HAP.dae` | Reference only; not physically validated |
| Tele | `urdf/livox_tele.xacro` | `livox_tele_link` | `tele.dae` | Reference only; not physically validated |
| Horizon | `urdf/livox_horizon.xacro` | `livox_horizon_link` | None | No longer supported on the manufacturer's website; geometry, mass, collision, and point-cloud results are not guaranteed |
| Mid-40 | `urdf/livox_mid40.xacro` | `livox_mid40_link` | None | No longer supported on the manufacturer's website; geometry, mass, collision, and point-cloud results are not guaranteed |

### Frames, Poses, and Mounting Features

Sensor poses in xacro are defined relative to the lidar point-cloud frame:

- `link_pose` is the pose of the lidar link in its parent model. With the default zero `sensor_pose`, the link origin, Gazebo custom-sensor origin, and point-cloud frame origin coincide.
- A non-zero `sensor_pose` offsets the custom sensor and point-cloud origin from the link. The TF follows the sensor entity.
- `mesh_pose` moves only the visual mesh; it does not change ray coordinates, TF, or the point-cloud frame.
- Use the relevant model manual to set `link_pose`, `sensor_pose`, or `mesh_pose` for offsets between mounting holes, the device base, housing, and point-cloud origin.

Do not assume that the visual mesh center is the point-cloud origin. Rays and point clouds are expressed in `${link_name}_frame`; the plugin publishes `world -> ${link_name}_frame` using the sensor entity's world pose.

### Common Xacro Parameters

| Parameter | Default / type | Description |
| --- | --- | --- |
| `link_name` | Required string | Naming root for the link, sensor, frame, and topic; must be globally unique |
| `link_pose` | `x y z roll pitch yaw` | Point-cloud link origin in the parent model |
| `sensor_pose` | `x y z roll pitch yaw` | Custom sensor pose relative to the link |
| `mesh_file` | Model-specific path or empty | Optional Gazebo visual mesh |
| `mesh_scale` | `1 1 1` | Gazebo mesh scale |
| `mesh_pose` | Zero pose | Visual mesh offset relative to the link |
| `scan_file` | Model-specific CSV | Scan trajectory with `Time/s,Azimuth/deg,Zenith/deg` columns |
| `update_rate` | Hz | Point-cloud scan publication rate |
| `range_min` / `range_max` | Meters | Minimum and maximum ray range |
| `downsample` | Positive integer | Keeps every Nth CSV point. For example, `2` halves the ray count of a 200,000-point-per-second scan |
| `noise_mean` / `noise_stddev` | Meters | Gaussian range noise along each ray |

Example:

```xml
<xacro:include filename="$(find livox_lidar_gz_sensor)/urdf/livox_mid70.xacro"/>

<xacro:livox_mid70_link
  link_name="front_mid70"
  link_pose="0.25 0 1.20 0 0 0"
  update_rate="20"
  range_min="0.1"
  range_max="70.0"
  downsample="2"
  noise_stddev="0.01"/>
```

### Multiple Lidar Models

Include multiple model files and invoke their link macros in the same model. Each `link_name` and fixed-joint child must be unique; a single System plugin instance can manage multiple sensors:

```xml
<xacro:include filename="$(find livox_lidar_gz_sensor)/urdf/livox_mid70.xacro"/>
<xacro:include filename="$(find livox_lidar_gz_sensor)/urdf/livox_avia.xacro"/>

<xacro:livox_mid70_link
  link_name="robot1_top_mid70"
  link_pose="0 0 1.0 0 0 0"/>
<xacro:livox_avia_link
  link_name="robot1_front_avia"
  link_pose="0.3 0 0.8 0 0 0"/>

<joint name="robot1_top_mid70_joint" type="fixed">
  <parent>base_link</parent>
  <child>robot1_top_mid70</child>
</joint>
<joint name="robot1_front_avia_joint" type="fixed">
  <parent>base_link</parent>
  <child>robot1_front_avia</child>
</joint>

<plugin
  filename="liblivox_lidar_gz_sensor.so"
  name="livox_lidar_system::LivoxLidarSystem"/>
```

For multiple robots, include a robot prefix in each name, such as `robot1_top_mid70` and `robot2_top_mid70`. Reusing a `link_name` across robots causes point-cloud topic and TF frame conflicts. Multiple model-scoped plugin instances can coexist. A world-scoped plugin can manage all robots, but do not also attach plugins to those models for the same sensors.

### Gazebo Meshes and RViz Point-Cloud Display

This project focuses on the sensor plugin, CSV scan trajectories, ray queries, and point-cloud publication. The `mesh_file` in SDF/xacro controls only optional visual geometry for the sensor link. It does not participate in scan generation, raycasts, physics hits, or point-cloud packing. Gazebo uses `<geometry><mesh><uri>...</uri><scale>...</scale></mesh></geometry>`; the default paths expand from `$(find livox_lidar_gz_sensor)/meshes/*.dae` and are resolved through Gazebo's resource path.

RViz can display the bridged `sensor_msgs/msg/PointCloud2` without `robot_state_publisher`, RobotModel, or any lidar mesh. Set the PointCloud2 display topic to `/${link_name}/points`. For point-cloud-only inspection, set Fixed Frame to `${link_name}_frame`; to display it in `world` or `map`, configure the required TF. The plugin sets the cloud `frame_id` to `${link_name}_frame`, so a RobotModel is not required to validate point-cloud output.

The example's `src/livox_lidar_example/urdf/livox_display.urdf.xacro` and RobotModel configuration are optional visualization helpers. They use URDF `package://...` mesh URIs resolved by `robot_state_publisher` and RViz. They do not change the Gazebo link, sensor pose, or point-cloud coordinates. Mesh placement, scale, coordinate origin, and synchronization between Gazebo and RViz are not calibrated or guaranteed by this project; users should validate these independently for their models.

The current Gazebo example primarily uses COLLADA `.dae` files. RViz mesh support depends on its renderer and URDF parsing chain. If a mesh does not display, check the Gazebo resource path or the ROS package share path as appropriate. A mesh-loading failure does not prevent the plugin from publishing point clouds.

# Developer Guide

## Sensor and System Responsibilities

`LivoxLidarSystem` is a Gazebo Sim System:

- `Configure` stores the owning entity and creates an owner-scoped TF publisher.
- `PreUpdate` discovers new sensors owned by the System with `EachNew<CustomSensor, ParentEntity>`.
- Sensors whose `gz:type` starts with `livox_` are instantiated as `LivoxLidarSensor` objects.
- The System generates rays for the current scan window and writes them to `components::RaycastData` on the sensor entity.
- `PostUpdate` reads physics raycast results, passes them to the Sensor, and publishes point clouds and `world -> frame_id` TF.
- `EachRemoved` clears scan timing, raycast, and instance state for removed sensors.

`LivoxLidarSensor` derives from `gz::sensors::Sensor`:

- `Load` parses `gz:livox_config`, the CSV file, range, downsampling, and noise parameters.
- `GenerateRays` creates sensor-frame rays for a CSV time window.
- `Update` packs raycast hits into `gz.msgs.PointCloudPacked` and publishes them.
- SDF `update_rate` determines the scan-window period; the CSV time span and point rate determine the number of rays in each window.

Processing flow:

```text
SDF / xacro
  -> CustomSensor entity
  -> LivoxLidarSystem::PreUpdate
  -> CSV time window + GenerateRays
  -> components::RaycastData
  -> Gazebo Physics / Bullet
  -> LivoxLidarSystem::PostUpdate
  -> PointCloudPacked + owner-scoped TF
  -> ros_gz_bridge
  -> PointCloud2 + /tf
  -> Optional RViz2 PointCloud2 display
```

Ray hits and point clouds remain in the sensor frame. Do not apply another inverse world-to-sensor transform before publication, or the coordinates will be transformed twice.

## Conventions and Limitations

- `gz:type` uses `livox_<model>`. The System discovers supported custom sensors by prefix, so a new model does not require a new System class.
- CSV files must have three columns and non-decreasing timestamps. Duplicate HAP timestamps are preserved as multiple rays at the same time.
- `intensity` is fixed at `1.0`; no material reflectivity model is implemented.
- Multiple returns, alternative scan modes, IMU data, vendor packet protocols, return intensity, and other device-specific features are not simulated.
- The current scan logic follows CSV trajectories and periodic time windows; it does not reproduce all real-device timing, returns, or internal scheduling.
- At most one complete scan window is submitted per physics step. If simulation time advances across multiple `update_rate` periods in one step, queued windows are processed in later physics steps and may publish back-to-back. Avoid simulation steps or pause/resume jumps longer than the scan period.
- Ray queries are not skipped when there are no point-cloud subscribers. Multiple high-rate lidars can add substantial CPU/physics load; tune `downsample`, `update_rate`, and sensor count.
- Legacy fields such as `visualize`, regular ray FOV, angular resolution, `samples`, and `resolution` were not fabricated during migration.
- The verified point-cloud message is `gz.msgs.PointCloudPacked`. ROS 2 output requires `ros_gz_bridge`; the plugin itself does not depend on ROS 2 message libraries.

## Build and Inspect

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
CCACHE_DISABLE=1 colcon build --symlink-install
source install/setup.bash

# XML / xacro
xmllint --noout src/livox_lidar_gz_sensor/urdf/livox_mid70.xacro
xacro src/livox_lidar_example/worlds/livox_example.world.xacro \
  -o /tmp/livox_example.sdf
gz sdf -k /tmp/livox_example.sdf

# Plugin interface
gz plugin -i -p \
  install/livox_lidar_gz_sensor/lib/liblivox_lidar_gz_sensor.so

# Runtime data
gz topic -e -t /livox_example_robot_1/tf
ros2 topic echo /livox_left/points --once
```

## Licensing and Release Note

Repository-original integration code is distributed under Apache-2.0. The upstream CSV reader and scan data listed in [`NOTICE`](NOTICE) retain their MIT terms. Third-party DAE meshes are not covered by either license; their sources and redistribution terms have not been verified. Resolve mesh permissions before redistributing them or including them in a release package.
