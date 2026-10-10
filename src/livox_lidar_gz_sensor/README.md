<div align="center">

<h1>Gazebo Sim Livox 激光雷达传感器插件</h1>

<p>面向 ROS 2 Jazzy 和 Gazebo Harmonic 的 Livox 激光雷达传感器插件</p>

<p>
  <a href="https://ubuntu.com/desktop"><img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04"></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"></a>
  <a href="https://gazebosim.org/docs/harmonic/"><img src="https://img.shields.io/badge/Gazebo-Harmonic-FFB81C?logo=gazebo&logoColor=111827" alt="Gazebo Harmonic"></a>
  <a href="https://cmake.org/"><img src="https://img.shields.io/badge/CMake-3.22%2B-064F8C?logo=cmake&logoColor=white" alt="CMake 3.22 或更高版本"></a>
  <a href="https://www.apache.org/licenses/LICENSE-2.0.html"><img src="https://img.shields.io/badge/license-Apache--2.0-green" alt="Apache 2.0 许可证"></a>
</p>

</div>

本项目基于 [Livox laser simulation](https://github.com/Livox-SDK/livox_laser_simulation) 重新开发。原项目面向较旧的 ROS 和 Gazebo Classic 版本，当前项目针对以下环境进行了迁移和重构：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic（Gazebo Sim 8）
- ament_cmake
- 现代 Gazebo system plugin API

> **项目状态**
>
> 当前已经完成 ROS 2 包结构、共享库插件、Gazebo 环境 hook、CSV 非重复扫描、物理射线检测和 Gazebo PointCloudPacked 发布。ROS 2 点云输出通过 ros_gz_bridge 转换为 sensor_msgs/msg/PointCloud2。

## 项目背景

原项目提供了有价值的 Livox 扫描模式数据，但其构建方式和接口无法直接适配 ROS 2 Jazzy 与 Gazebo Harmonic。本项目保留这些扫描模式数据，并将 Livox 激光雷达模拟逻辑迁移到 Gazebo Harmonic 和 ROS 2 工作空间规范。

## 当前功能

| 模块 | 当前支持情况 |
| --- | --- |
| 构建系统 | 基于 ament_cmake 的 ROS 2 包 |
| Gazebo API | Gazebo Sim 8 system plugin API |
| 插件库 | liblivox_lidar_gz_sensor.so |
| 自动发现 | 通过 GZ_SIM_SYSTEM_PLUGIN_PATH 环境 hook |
| 生命周期诊断 | ISystemConfigure 启动日志和 gzmsg 更新日志 |
| 扫描模式数据 | HAP、Avia、Horizon、Mid-360、Mid-40、Mid-70、Tele CSV 文件 |
| 扫描窗口 | 按 `<update_rate>` 以 `[上次更新时间, 当前更新时间)` 发布完整窗口 |
| 降采样 | `<downsample>N</downsample>` 均匀保留 CSV 第 `0、N、2N...` 个样本 |
| Gazebo 点云发布 | `gz.msgs.PointCloudPacked`，topic 使用 `<topic>` |
| ROS 点云发布 | 通过 ros_gz_bridge 转换为 `sensor_msgs/msg/PointCloud2` |
| 测距噪声 | 沿每条射线加入可配置的高斯距离噪声，单位为米 |
| Livox link 模型 | `livox_mid70_link` 统一生成 pose、惯性、mesh visual、碰撞体和 sensor |

## 目录结构

~~~text
livox_lidar_gz_sensor/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── livox_lidar_system.hpp
├── src/
│   └── livox_lidar_system.cpp
├── scan_mode/
│   ├── mid70.csv
│   ├── mid40.csv
│   ├── mid360.csv
│   └── ...
└── hooks/
    ├── livox_lidar_gz_sensor.dsv.in
    └── livox_lidar_gz_sensor.sh.in
~~~

扫描模式文件使用三列数据：

~~~text
Time/s,Azimuth/deg,Zenith/deg
~~~

例如，scan_mode/mid70.csv 保存了描述 Mid-70 扫描模式的时间序列、方位角和天顶角数据。

### 更新周期与降采样

`<update_rate>10</update_rate>` 表示每 `T = 1 / 10 = 0.1 s` 发布一帧。Mid-70 的 CSV 约为每秒 100000 个样本，因此 `downsample=1` 时每帧约生成 10000 条射线：

~~~xml
<xacro:livox_mid70_link
  link_name="livox_mid_70"
  update_rate="10"
  downsample="2"/>
~~~

`downsample=2` 保留第 `0、2、4...` 个 CSV 样本，射线数量约减半；`downsample=10` 约保留十分之一。时间窗口仍按原始 CSV 的时间轴计算，因此降采样不会改变扫描模式的播放速度。

### 高斯噪声

`noise_mean` 和 `noise_stddev` 表示沿射线方向的距离噪声，单位为米。Mid-70 xacro 默认使用 `stddev=0.01`，可以将其设为 `0` 关闭噪声：

~~~xml
<xacro:livox_mid70_link
  link_name="livox_mid_70"
  noise_mean="0.0"
  noise_stddev="0.01"/>
~~~

## 环境要求

请先安装 ROS 2 Jazzy 和 Gazebo Harmonic，然后在构建终端中加载 ROS 2 并显式指定 Gazebo 版本：

~~~bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
~~~

本包依赖 Jazzy/Harmonic 环境提供的 Gazebo 开发包，主要包括 gz-sim8、gz-plugin2 和 gz-common5。

## 编译

在包含本包的工作空间中执行：

~~~bash
cd ~/gz_ros_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic

# 当本机 ccache 目录只读时，禁用 ccache 可以避免构建失败。
CCACHE_DISABLE=1 colcon build --symlink-install --packages-select livox_lidar_gz_sensor

source install/setup.bash
~~~

共享库安装位置为：

~~~text
install/livox_lidar_gz_sensor/lib/liblivox_lidar_gz_sensor.so
~~~

## 插件自动发现

加载 install/setup.bash 后，本包的环境 hook 会把插件目录加入 GZ_SIM_SYSTEM_PLUGIN_PATH：

~~~bash
echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
~~~

因此，SDF 中可以直接使用库名，而不需要写绝对路径：

~~~xml
<plugin
  filename="liblivox_lidar_gz_sensor.so"
  name="livox_lidar_system::LivoxLidarSystem" />
~~~

插件仍然必须写入 SDF world 或 model，Gazebo 才会实例化它。环境 hook 只负责让 Gazebo 找到共享库，不会自动加载插件。

## 启动示例工作空间

如果使用当前工作空间中的示例 bringup 包，可以按下面的方式启动：

~~~bash
cd ~/gz_ros_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
source install/setup.bash

ros2 launch ros_gz_example_bringup diff_drive.launch.py
~~~

工作空间还提供了一个启动脚本，会自动完成环境设置并固定使用 Gazebo Harmonic：

~~~bash
cd ~/gz_ros_ws
bash ./run_diff_drive.sh
~~~

插件通过 std::cerr 输出启动确认，因此这条日志不依赖 Gazebo verbosity：

~~~text
[livox_lidar_gz_sensor] LivoxLidarSystem started on entity <id>
~~~

PostUpdate 中的诊断信息使用 gzmsg。如果需要在终端显示这类消息，可以使用 ROS 2 launch 参数语法：

~~~bash
bash ./run_diff_drive.sh gz_verbosity:=3
~~~

不要把 -v 3 直接传给 ros2 launch；-v 属于底层 gz sim 命令的参数。

## 检查共享库

可以使用 Gazebo 插件工具检查共享库中注册的类和接口，不需要启动仿真世界：

~~~bash
source /opt/ros/jazzy/setup.bash
gz plugin -i -p install/livox_lidar_gz_sensor/lib/liblivox_lidar_gz_sensor.so
~~~

预期可以看到以下接口：

~~~text
livox_lidar_system::LivoxLidarSystem
gz::sim::v8::System
gz::sim::v8::ISystemConfigure
gz::sim::v8::ISystemPostUpdate
~~~

## 后续开发方向

下一阶段需要实现完整的 Livox 传感器处理流程：

1. 加载指定的 CSV 扫描模式，例如 mid70.csv。
2. 将每个方位角和天顶角样本转换为传感器坐标系中的射线。
3. 通过 Gazebo Sim 的传感器或物理组件提交射线。
4. 获取距离回波，并应用传感器位姿和时间模型。
5. 通过明确的 Gazebo Transport 或 ROS 2 bridge 接口发布点云。

这样的模块划分可以分别测试插件加载、扫描模式数据、射线生成和 ROS 消息传输。

## 参考资料

- [原始 Livox laser simulation 项目](https://github.com/Livox-SDK/livox_laser_simulation)
- [Gazebo Sim system plugin 官方文档](https://gazebosim.org/api/sim/8/createsystemplugins.html)
- [ROS 2 Jazzy 官方文档](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic 官方文档](https://gazebosim.org/docs/harmonic/)

## 许可证

本项目使用 Apache License 2.0。包的许可证声明位于 package.xml 中。
