# livox_lidar_example

这是 `livox_lidar_gz_sensor` 的最简示例工程，包含：

- 两个可拖动的圆锥形机器人底座（`base_link_left` 和 `base_link_right`）；
- 每个圆锥顶部各安装一个独立的 Mid-70 实例；
- 四周的可交互墙壁和固定地面；
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

世界关闭重力，圆锥和墙壁都是动态实体，地面保持固定，可以在 Gazebo GUI 中拖动。自定义插件直接发布 `world -> livox_left_frame` 和 `world -> livox_right_frame` 动态 TF，拖动后 RViz2 中的两路点云会跟随更新。
