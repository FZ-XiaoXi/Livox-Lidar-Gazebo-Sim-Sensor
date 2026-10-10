<!-- SPDX-FileCopyrightText: 2026 FZ_XiaoXi -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

<div align="center">

# Livox 激光雷达 Gazebo Sim 仿真工作区

面向 ROS 2 Jazzy、Gazebo Harmonic 的 CSV 扫描模式 Livox 激光雷达插件与示例工程。

<p>
  <a href="https://example.invalid/PROJECT_URL"><img src="https://img.shields.io/badge/project-Livox%20Gazebo%20Sim-0f766e?logo=gazebo&logoColor=white" alt="Project"></a>
  <a href="https://example.invalid/PROJECT_URL"><img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04"></a>
  <a href="https://example.invalid/PROJECT_URL"><img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"></a>
  <a href="https://example.invalid/PROJECT_URL"><img src="https://img.shields.io/badge/Gazebo-Harmonic-FFB81C?logo=gazebo&logoColor=111827" alt="Gazebo Harmonic"></a>
  <a href="https://example.invalid/PROJECT_URL"><img src="https://img.shields.io/badge/license-Apache--2.0%20%2B%20MIT-64748b" alt="Apache-2.0 and MIT"></a>
</p>

</div>

本项目基于 [Livox laser simulation](https://example.invalid/UPSTREAM_REPOSITORY_URL) 的扫描模式数据和实现思路，迁移到现代 Gazebo Sim system plugin API。仓库包含一个可复用的传感器插件，以及一个可以直接启动和拖动实体的最简示例。

## 快速导航

| 内容 | 入口 |
| --- | --- |
| 用户快速开始 | [构建与启动](#用户部分) |
| 插件 API 与参数 | [插件使用](#插件使用) |
| 最简双雷达示例 | [示例工程](#示例工程) |
| 开发者架构 | [开发部分](#开发部分) |
| 参数兼容性与已知限制 | [兼容性与限制](#兼容性与限制) |
| 许可与发布风险 | [许可审查与待办](#许可审查与待办) |
| 详细发布审查 | [`RELEASE_AUDIT.md`](RELEASE_AUDIT.md) |

## 仓库结构

```text
gz_ros_ws/
├── README.md
├── src/
│   ├── livox_lidar_gz_sensor/       # 可复用 Gazebo Sim 插件
│   │   ├── include/                 # Sensor / System 头文件
│   │   ├── src/                     # 射线、点云、TF、生命周期实现
│   │   ├── urdf/                    # 各型号 link + custom sensor xacro
│   │   ├── scan_mode/               # CSV 扫描模式
│   │   └── meshes/                  # 官方下载后手动平移的型号 mesh
│   ├── livox_lidar_example/         # 最简双雷达示例
│   │   ├── worlds/                  # 两个可拖动机器人、四面墙、固定地面
│   │   ├── config/                  # bridge 与 RViz 配置
│   │   └── launch/                  # ROS 2 启动文件
└── RELEASE_AUDIT.md                 # 发布前代码与许可审查
```

# 用户部分

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8
- `ament_cmake`、`colcon`、`xacro`

## 构建

```bash
cd ~/gz_ros_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic

CCACHE_DISABLE=1 colcon build --symlink-install \
  --packages-select livox_lidar_gz_sensor livox_lidar_example

source install/setup.bash
```

`CCACHE_DISABLE=1` 只用于本机 ccache 目录不可写时的构建规避；正常环境可以省略。

## 示例工程

插件 API 与参数说明见[插件使用](#插件使用)；插件仓库中的示例入口见[示例工程](#示例工程)。

示例位于 [`src/livox_lidar_example`](src/livox_lidar_example)，包括：

- 两个动态圆锥机器人，底座 frame 为 `base_link_left`、`base_link_right`；
- 每个机器人顶部一个独立 Livox 实例；
- 四面可交互墙壁和一个固定地面；
- 世界级 `UserCommands`，支持 Gazebo GUI Transform Control；
- ROS 2 bridge、RViz2 双点云显示和动态 TF；
- 一键脚本 [`run_livox_example.sh`](src/livox_lidar_example/run_livox_example.sh)。

启动：

```bash
./src/livox_lidar_example/run_livox_example.sh
```

启动参数：

```bash
./src/livox_lidar_example/run_livox_example.sh rviz:=false
./src/livox_lidar_example/run_livox_example.sh gz_verbosity:=4
```

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `rviz` | `true` | 是否启动 RViz2 |
| `gz_verbosity` | `3` | Gazebo 日志等级，`4` 可显示更详细调试信息 |

示例点云 topic：

```text
/livox_left/points
/livox_right/points
```

对应 TF：

```text
world -> livox_left_frame
world -> livox_right_frame
```

传感器通过 fixed joint 安装在机器人上。拖动机器人模型会带动雷达、射线结果和 RViz 点云；墙壁可以单独拖动。示例关闭重力，动态实体不会自行掉落。

## 插件使用

双雷达可运行场景和启动方法见[示例工程](#示例工程)。

插件库：

```text
liblivox_lidar_gz_sensor.so
```

加载插件的最小 SDF：

```xml
<plugin
  filename="liblivox_lidar_gz_sensor.so"
  name="livox_lidar_system::LivoxLidarSystem"/>
```

TF 默认使用 system owner 的 scoped topic，例如 `/livox_example_robot_1/tf`。需要自定义时，可在该 system plugin 内传入 `<tf_topic>/my_namespace/tf</tf_topic>`；多个插件实例应使用不同 Gazebo topic。示例 bridge 将各 scoped topic 合并到 ROS 标准 `/tf`，RViz 的 TF 显示显式订阅 `/tf`。

加载工作区后，环境 hook 会设置 `GZ_SIM_SYSTEM_PLUGIN_PATH`。插件必须实际写入 world 或 model 的 `<plugin>`，hook 不会自动实例化插件。

### 型号宏

所有新型号都采用与 Mid-70 相同的公共接口。`link_name` 必填；sensor、frame 和 topic 自动生成。

| 型号 | xacro 文件 | link 宏 |
| --- | --- | --- |
| HAP | `urdf/livox_HAP.xacro` | `livox_hap_link` |
| Avia | `urdf/livox_avia.xacro` | `livox_avia_link` |
| Horizon | `urdf/livox_horizon.xacro` | `livox_horizon_link` |
| Mid-360 | `urdf/livox_mid360.xacro` | `livox_mid360_link` |
| Mid-40 | `urdf/livox_mid40.xacro` | `livox_mid40_link` |
| Mid-70 | `urdf/livox_mid70.xacro` | `livox_mid70_link` |
| Tele | `urdf/livox_tele.xacro` | `livox_tele_link` |

从旧 xacro 对照并迁移的默认参数：

| 型号 | CSV 点数 / 时间跨度 | CSV 点率 | 频率 | 量程 | 安装偏移 |
| --- | ---: | ---: | ---: | ---: | --- |
| HAP | 1,048,575 / 2.3136 s | 约 453 k点/s | 10 Hz | 0.1--200 m | `0 0 1.6` |
| Avia | 960,000 / 4 s | 240 k点/s | 10 Hz | 0.1--200 m | `0 0 0` |
| Horizon | 960,000 / 4 s | 240 k点/s | 10 Hz | 0.1--200 m | `0 0 0` |
| Mid-360 | 800,000 / 4 s | 200 k点/s | 10 Hz | 0.1--200 m | `0 1.0 0.2` |
| Mid-40 | 400,000 / 4 s | 100 k点/s | 10 Hz | 0.1--200 m | `0 0 0` |
| Mid-70 | 400,000 / 4 s | 100 k点/s | 10 Hz | 0.1--200 m | `0 0 0` |
| Tele | 960,000 / 4 s | 240 k点/s | 10 Hz | 0.1--200 m | `0 0 0` |

表中是旧 xacro 的迁移值。Mid-70 宏仍采用项目既有默认值 20 Hz / 0.1--70 m；如需与旧项目一致，在调用时传入 `update_rate="10"`、`range_max="200"`。CSV 点率与消息频率独立，例如 Mid-360 在 10 Hz 下每帧约 20,000 条射线。

调用示例：

```xml
<xacro:include filename="$(find livox_lidar_gz_sensor)/urdf/livox_avia.xacro"/>

<xacro:livox_avia_link
  link_name="front_avia"
  link_pose="0 0 1.0 0 0 0"
  update_rate="10"
  downsample="2"
  noise_stddev="0.01"/>
```

自动生成：

```text
sensor name = front_avia_sensor
frame_id    = front_avia_frame
topic       = /front_avia/points
```

### 公共参数

| 参数 | 默认/类型 | 作用 |
| --- | --- | --- |
| `link_name` | 必填字符串 | link、sensor、frame、topic 的命名根 |
| `link_pose` | `x y z roll pitch yaw` | 传感器 link 在父模型中的位姿 |
| `sensor_pose` | `x y z roll pitch yaw` | custom sensor 在 link 中的位姿 |
| `mesh_file` | 型号 mesh 路径 | visual mesh URI |
| `mesh_scale` | `1 1 1` | mesh 缩放 |
| `mesh_pose` | 零位姿 | mesh visual 相对 link 的位姿 |
| `scan_file` | 型号 CSV | 扫描时间、方位角、天顶角 |
| `update_rate` | Hz | 扫描窗口发布频率 |
| `range_min` / `range_max` | 米 | 射线起止距离 |
| `downsample` | 正整数 | 保留 CSV 的第 `0、N、2N...` 行 |
| `noise_mean` / `noise_stddev` | 米 | 沿射线方向的高斯距离噪声 |

配置文件不再接收 `sensor_name` 或 `frame_id` 参数。`link_name` 必须在同一 Gazebo transport 域内全局唯一，因为点云 topic 和 TF frame 均由它派生；多机器人可用 `robot1_livox_front`、`robot2_livox_front` 这样的名称。

# 开发部分

## Sensor 与 System 分工

`LivoxLidarSystem` 是挂在 model 或 world 层的 Gazebo Sim system：

- `Configure`：保存 owner entity，创建 Gazebo TF publisher；
- `PreUpdate`：发现 owner 下的 `CustomSensor`，创建 `LivoxLidarSensor`，提交 `RaycastData`；
- `PostUpdate`：读取物理射线结果，更新 sensor，发布点云和 `world -> frame_id` TF。

`LivoxLidarSensor` 继承 `gz::sensors::Sensor`：

- `Load` 读取 `gz:livox_config`、CSV、量程、降采样和噪声；
- `GenerateRays` 用 CSV 时间窗口生成传感器坐标系射线；
- `Update` 将命中结果打包为 `gz.msgs.PointCloudPacked`；
- topic、frame 和消息序号在 Sensor 内维护。

## 运行流程

```text
SDF / xacro
  -> CustomSensor entity
  -> LivoxLidarSystem::PreUpdate
  -> CSV 时间窗口 + GenerateRays
  -> components::RaycastData
  -> Gazebo Physics / Bullet
  -> LivoxLidarSystem::PostUpdate
  -> PointCloudPacked + owner-scoped TF topic
  -> ros_gz_bridge
  -> PointCloud2 + /tf
  -> RViz2
```

射线与命中点保持在 sensor frame。不要在发布前再次套用 world 到 sensor 的逆变换，否则会出现重复变换。插件直接使用传感器 entity 的 world pose 发布与点云相同的 `frame_id`，不需要额外创建同名空 link。

## 代码与资源约定

- 新型号配置应复制 Mid-70 的 link/sensor 结构，只替换型号、CSV、mesh 和已确认的安装偏移。
- `gz:type` 使用 `livox_<model>`，System 通过 `livox_` 前缀发现，不需要为每个型号新增一套 system 类。
- CSV 必须是三列：`Time/s,Azimuth/deg,Zenith/deg`，时间非递减；HAP 数据有重复时间戳，重复项作为同一时刻的多束射线保留。
- 真正 mesh 替换时，保持 xacro 参数中的 mesh URI 与 `GZ_SIM_RESOURCE_PATH` 可解析。
- ROS bridge 的每个点云 topic 必须与 SDF `<topic>` 完全一致。

## 构建与验证

插件构建：

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
CCACHE_DISABLE=1 colcon build --symlink-install \
  --packages-select livox_lidar_gz_sensor
```

建议的静态检查：

```bash
xmllint --noout src/livox_lidar_gz_sensor/urdf/livox_avia.xacro
gz sdf -k <expanded_world.sdf>
gz plugin -i -p install/livox_lidar_gz_sensor/lib/liblivox_lidar_gz_sensor.so
```

运行检查：

```bash
gz topic -e -t /livox_example_robot_1/tf
gz topic -e -t /front_avia/points
ros2 topic echo /front_avia/points --once
```

## 兼容性与限制

- 旧配置中的 `visualize`、规则 ray FOV、角分辨率、`samples`、`resolution` 没有迁移到当前 custom sensor；当前实现使用 CSV 真实扫描轨迹。
- 当前 `intensity` 字段固定为 `1.0`，尚未依据目标材质或 Livox 反射率模型计算；CSV 也不包含反射率数据。
- Mid-360 CSV 原始时间序号已全部除以 200000，800,000 行时间范围为 5 us 至 4 s；插件载入时归一化为 0 至 3.999995 s，并以 4 s 为周期。帧窗口采用 `[start, end)`，边界点只属于一帧。
- 各型号 CSV 点率已核对：HAP 约 453 k点/s（含重复时间戳），Avia/Horizon/Tele 240 k点/s，Mid-360 200 k点/s，Mid-40/Mid-70 100 k点/s。
- `meshes/` 下的模型来自 Livox 官方下载，并经过手动坐标平移；其独立再分发许可尚未核验。
- 点云发布使用 `gz.msgs.PointCloudPacked`，ROS 2 输出依赖 `ros_gz_bridge`，插件本身不直接依赖 ROS 2 消息库。
- TF topic 按 system owner 隔离，例如 `/livox_example_robot_1/tf`；示例 bridge 合并到 ROS `/tf`。

## 开源许可证

最终采用文件范围明确的混合许可证策略：

| 文件范围 | 许可证 | 版权与来源 |
| --- | --- | --- |
| 本仓库原创插件、示例、xacro、launch、构建配置和文档 | Apache-2.0 | `SPDX-License-Identifier: Apache-2.0`；版权 `FZ_XiaoXi`，2026 |
| 上游扫描 CSV 与修改过的 CSV | MIT | 上游版权 `livox`，2021；清单及完整许可文本见 [`NOTICE`](NOTICE) |
| `include/csv_reader.hpp` | MIT | 保留 `livox` 上游版权，并标注 `FZ_XiaoXi` 修改；见文件 SPDX 头与 [`NOTICE`](NOTICE) |
| `meshes/*.dae` | 未确定 | 属于单独下载的第三方模型，不由本仓库 Apache-2.0 授权 |

根目录 [`LICENSE`](LICENSE) 是原创代码的 Apache-2.0 正文；它不会把 MIT 文件或第三方 mesh 转授权为 Apache-2.0。插件包 manifest 同时列出 `Apache-2.0` 与 `MIT`，表示包内存在这两类文件；文件级范围以 SPDX 头和 `NOTICE` 为准。Example 包和其原创文件使用 Apache-2.0。

`scan_mode/mid360.csv` 的时间列经过转换，但数据来源仍是上游，继续保留 MIT 条件。`meshes/` 的下载来源已在 `NOTICE` 记录，但独立再分发许可与商标条件尚未核验；获得权利依据前，不应将 mesh 纳入公开二进制或资源包发布。

发布前许可证待办：

- [x] 将上游 MIT 版权与许可声明放入根目录第三方声明，并标注受影响文件；
- [ ] 确认官方 mesh 和 Livox 商标/型号资料的分发权限；
- [x] 统一原创文件 SPDX 头、插件和 example manifest 的许可证字段及文件级发布策略；
- [ ] 补充各第三方依赖的许可证清单和版本；
- [ ] 发布前由项目维护者或法律顾问复核 NOTICE、版权头和再分发义务。
- [ ] 为 `intensity` 实现有依据的反射率输出；当前值固定为 `1.0`。

以上是基于当前工作区文件的工程审查，不构成法律意见。

## 外部链接占位符

- 项目主页：`https://example.invalid/PROJECT_URL`
- 上游 Livox 项目：`https://github.com/Livox-SDK/livox_laser_simulation`
- Gazebo Sim system plugin 文档：`https://example.invalid/GAZEBO_SYSTEM_PLUGIN_DOC_URL`
- ROS 2 Jazzy 文档：`https://example.invalid/ROS2_JAZZY_DOC_URL`
- Gazebo Harmonic 文档：`https://example.invalid/GAZEBO_HARMONIC_DOC_URL`
