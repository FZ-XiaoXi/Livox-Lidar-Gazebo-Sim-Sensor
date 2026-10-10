<!-- SPDX-FileCopyrightText: 2026 FZ_XiaoXi -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# 发布前审查报告

审查范围：仓库内 Livox Gazebo Sim 插件、型号 xacro、CSV 资源、双雷达示例、构建安装规则、ROS/Gazebo bridge、RViz 配置和仓库内可见许可证信息。

审查环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic / Gazebo Sim 8。报告记录的是当前工作区状态，不替代正式法律意见。

仓库边界注意：`/home/seaber/gz_ros_ws` 是包含插件包和 example 包的 Git 根仓库，根目录文档与许可证随该仓库发布。

## 结论

代码可以在当前环境完成构建，示例 SDF 和七种型号 xacro 可以展开，插件能够加载两个不同型号实例并发布点云与 TF。Mid-360 周期回归已经完成；发布前仍有许可证清理、仓库边界处理和自动化测试待办：

1. 原创代码的 Apache-2.0 策略、上游 MIT 文件范围和 SPDX/manifest 标识已统一；官方 mesh 的独立再分发许可仍待核验。
2. 当前没有自动化单元测试；本次验证使用构建、xacro/SDF、插件加载和 Gazebo server smoke test。

## 已修复问题

- 删除传感器实体后，`LivoxLidarSystem` 现在清理 sensor、扫描时间和 pending raycast 映射，避免删除模型后残留状态。
- 加入 `range_min` / `range_max` 有限性和大小关系校验。
- 高斯噪声分布移到每帧循环外，避免每个点重复构造分布对象。
- CSV 行必须解析为三列；非法行会使加载失败，不再静默跳过并生成不完整模式。
- 插件成功初始化日志改为 `gzmsg`；完整扫描窗口诊断改为 `gzdbg`，避免正常运行持续刷错误级别日志。
- 安装 CMake 头文件，包声明 Gazebo vendor 依赖和 xacro 运行依赖。
- 统一原创源码与配置的 `SPDX-FileCopyrightText: 2026 FZ_XiaoXi` / `SPDX-License-Identifier: Apache-2.0` 头；上游 CSV reader 保留 `livox` MIT 并注明本项目修改。
- 根 LICENSE、NOTICE、插件与 example manifest、README 均采用同一文件级许可证策略；mesh 明确排除于根级 Apache-2.0 授权之外。
- 修正示例动态墙的惯性张量，使其与 wall box 尺寸和质量一致。
- 统一 HAP、Avia、Horizon、Mid-360、Mid-40、Mid-70、Tele 的 xacro 接口：`link_name` 派生 sensor 名、frame 和 topic。
- 删除 xacro 中无物理意义的同名空 frame link；TF 由 system 根据 sensor entity 的 world pose 发布，避免引入无质量固定 link。
- 删除源码树下所有子目录 README，内容集中到根目录 [`README.md`](README.md)。

## 架构审查

`LivoxLidarSystem` 负责发现 owner entity 下的 `CustomSensor`、创建传感器对象、提交 `RaycastData`、读取物理结果和发布动态 TF。`LivoxLidarSensor` 负责读取配置和 CSV、生成传感器坐标系射线、应用距离噪声、打包 `gz.msgs.PointCloudPacked`。

当前 TF 和 topic 约定是一致的：对 `link_name=front_avia`，sensor 名为 `front_avia_sensor`，点云 `frame_id=front_avia_frame`，topic 为 `/front_avia/points`，插件 TF 为 `world -> front_avia_frame`。model 级 system 的默认 Gazebo TF topic 为 `/model_name/tf`；示例 bridge 将多个 scoped topic 合并到 ROS `/tf`。多个 sensor 可以共享一个 system 实例；多个 model 级 system 实例也会按 owner 过滤实体。

射线和命中点保持在 sensor frame，避免二次变换。物理射线通过 `SetComponentData<components::RaycastData>` 更新，满足 Gazebo Sim 8 物理系统对组件变更的要求。

## 型号审查

旧 Livox xacro 中的 CSV、默认频率、量程、降采样和噪声已迁移到同格式的新 xacro。旧配置里的 `visualize`、规则 ray 的 FOV、角分辨率、规则 ray `samples` 和 `resolution` 没有当前 CSV 驱动 custom sensor 的等价字段，因此明确没有伪造迁移。

Mid-360 原始时间列为 `1..800000`，已按 200000 点/秒缩放到约 `0.000005..4` 秒，使单位与插件一致。Gazebo 实测 `update_rate=10` 时每个 `[0.1s)` 窗口为 20,000 条射线，完整运行跨过 `[3.9, 4.0)` 后继续输出 `[4.0, 4.1)`，回绕没有重复或漏点；CSV 静态检查覆盖完整 4 秒和 800,000 行。

当前各型号 mesh 根据维护者说明来自 Livox 官方下载并经过手动坐标平移；它们不能在许可证确认前视为可自由再分发资产，也不能作为真实外观和尺寸的验证依据。

## 验证记录

已执行并通过：

- `CCACHE_DISABLE=1 colcon build --symlink-install`，工作区两个包构建成功；
- 所有型号 xacro XML 语法检查；
- 六个新增型号与 Mid-70 共七种 xacro 文件解析，且示例 world 展开成功；
- `gz sdf -k` SDF 合法性检查；
- 生成 SDF 中 sensor 名、`gz:type`、CSV、mesh、frame 和 topic 一一对应；
- Gazebo server smoke test：`UserCommands` 服务创建，两个 Livox sensor 成功加载，射线结果和点云持续发布；
- owner-scoped TF 消息检查，确认 `/livox_example_robot_1/tf`、`/livox_example_robot_2/tf` 存在，且 `frame_id=world`、`child_frame_id=livox_left_frame/livox_right_frame`；
- 10 秒 server-only Gazebo 回归，确认 Mid-360 连续输出 20,000 rays / 0.1 s，Horizon 连续输出 24,000 rays / 0.1 s，并无插件错误；
- `git diff --check` 和 launch Python 语法检查。

未覆盖：

- 没有自动化单元测试或 launch_testing 测试；
- 没有在所有支持的 Gazebo physics backend 上验证，当前射线验证依赖 DART + Bullet；
- 没有完成 RViz GUI 像素级截图测试；
- 没有在模型删除、运行时动态生成 sensor、多个 world 共用同一 transport domain 的场景中做完整回归。

## 全量代码审查发现

| 严重度 | 发现 | 影响 / 处理 |
| --- | --- | --- |
| 阻断发布 | 官方 mesh 的独立再分发许可未知 | `NOTICE` 已说明来源与处理；发布前需核验授权 |
| 中 | 点云 `intensity` 固定写成 `1.0` | 不表达材质反射率或真实 Livox intensity；当前文档已明确限制 |
| 中 | `frame_id` 与点云 topic 只根据 `link_name` 派生 | 多机器人使用同名传感器会产生 topic/frame 冲突；README 要求同一 transport 域内全局唯一 |
| 中 | 没有提交的自动化测试，运行回归目前依赖人工 Gazebo smoke test | 加入 CSV、窗口端点、实体删除和多实例测试后再做发布候选 |
| 中 | 除 Mid-70 外各型号 mesh 是 Mid-70 模型副本 | 仅能验证资源安装和渲染，不代表真实尺寸、外观或碰撞外形 |

修复的代码问题：射线窗口原先实现为 `(start, end]`，与注释目标相反；现已改为 `[start, end)`。噪声标准差为零时不再构造非法的零方差正态分布。HAP 的重复时间戳由非递减 CSV 校验保留；空 frame link、未使用的 `model` 配置和 ray normal 缓存已移除。

## 发布前待办

- [x] 完成 Mid-360 时间换算、4 秒静态周期和 Gazebo 窗口边界回归；
- [ ] 为 owner-scoped TF topic 增加多 world 运行回归；
- [ ] 增加自动化测试：CSV 校验、射线窗口边界、实体删除、多 sensor topic/TF 唯一性；
- [ ] 确认官方坐标平移 mesh 和 Livox 商标/型号资料的再分发权限；
- [x] 为仓库补充根级 Apache-2.0 LICENSE 和包含上游 MIT 全文的 NOTICE；
- [x] 在 NOTICE 中明确上游 MIT 范围为 `scan_mode/*.csv` 与 `include/csv_reader.hpp`；
- [x] 统一原创代码 SPDX 版权头、包 manifest 许可证字段和文件级发布策略；
- [x] 清理 launch 生成的 `__pycache__`，并确认源码、build/install/log 中均无 diff-drive 或 radar_test 遗留；build/install/log 由 `.gitignore` 排除。

## 许可证风险

审查到的事实：

- `/home/seaber/livox_laser_simulation/LICENSE` 是 MIT License，版权标识为 `livox`；
- `livox_lidar_gz_sensor/package.xml` 声明包内 Apache-2.0 与 MIT 文件，`livox_lidar_example/package.xml` 声明 Apache-2.0；
- 原创 C++、头文件、xacro、launch、CMake 与 example 配置带 Apache-2.0 SPDX 标识；`csv_reader.hpp` 带 MIT SPDX 标识与双版权归属；CSV 的范围由 NOTICE 列出；
- 根目录 `LICENSE` 是 Apache License 2.0；根目录 `NOTICE` 完整保留上游 MIT 声明，并限定到 `scan_mode/*.csv` 与 `include/csv_reader.hpp`。
- 根据维护者说明，`meshes/` 来自 Livox 官方下载并经过手动坐标平移；mesh 不纳入上游 MIT 清单，其独立许可待核验。

因此按文件范围识别混合许可：Apache-2.0 是本仓库原创代码的默认许可证，MIT 只适用于 NOTICE 和 SPDX 标识的上游文件；不得把 package manifest 的多许可证列表解释成包内每个文件都双重许可。mesh 属于单独下载并修改坐标的第三方资产，仍需核验再分发权。发布前应由项目维护者确认 mesh 权利和 Livox 商标范围，必要时让法律顾问复核。

## 风险分级

| 等级 | 项目 | 处理状态 |
| --- | --- | --- |
| 已完成/待确认 | SPDX 头、manifest 与文件级 Apache/MIT 策略已统一；mesh 许可仍未核验 | mesh 授权是发布阻断项 |
| 已关闭 | Mid-360 时间轴和 0.1 秒窗口边界 | 4 秒 CSV 静态检查和 Gazebo server 回归通过 |
| 高 | 没有自动化回归测试 | 发布前建议补充 |
| 中 | 多 world 下 scoped TF topic 尚未回归 | 示例 bridge 已按 model owner 隔离 |
| 中 | mesh 坐标已手动平移，需保留来源和处理记录 | NOTICE 已记录，待补充授权凭据 |
| 低 | ROS/Gazebo 端到端 RViz 画面未做 GUI 像素级验证 | TF 与点云 topic 数据已验证，仍建议在发布机做一次人工显示检查 |
