# livox_lidar_example

这是 `livox_lidar_gz_sensor` 的最简示例工程，包含：

- 两个静态 `base_link` 圆锥形机器人底座；
- 每个圆锥顶部各安装一个独立的 Mid-70 实例；
- 四周的静态墙壁和地面；
- 唯一的自定义用户插件 `liblivox_lidar_gz_sensor.so`；
- Gazebo 点云到 ROS 2 `sensor_msgs/msg/PointCloud2` 的 bridge；
- RViz2 点云显示配置；
- 一键启动脚本。

## 构建

```bash
cd ~/gz_ros_ws
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
CCACHE_DISABLE=1 colcon build --symlink-install \
  --packages-select livox_lidar_gz_sensor livox_lidar_example
source install/setup.bash
```

## 启动

```bash
cd ~/gz_ros_ws
./src/livox_lidar_example/run_livox_example.sh
```

不启动 RViz：

```bash
./src/livox_lidar_example/run_livox_example.sh rviz:=false
```

两个雷达分别发布到：

```text
/livox_left/points
/livox_right/points
```

启动文件分别发布 `world -> livox_left_frame` 和 `world -> livox_right_frame` 静态 TF，与两个传感器在世界中的位置和朝向对应。Gazebo GUI 用于查看圆锥和墙壁，RViz2 分别显示两路点云。
