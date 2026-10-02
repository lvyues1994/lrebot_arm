# lrebot_arm

通用机械臂框架 `larm`，第一个落地机器人是 reBot Arm B601-RS。设计见 [docs/architecture.md](docs/architecture.md)。

## 准备

依赖 Ubuntu 24.04、ROS 2 Jazzy（Pinocchio、Ruckig 取自其安装目录）、`libyaml-cpp-dev`、`libexpected-dev`、`libgtest-dev`、`libeigen3-dev`、CMake 3.25+、Ninja。lexec、co2、lrclexec、lqtexec 在配置时按固定提交自动拉取。Studio 另需 Qt 6（Widgets、OpenGL），在 X11 下运行官方 Qt 还需要 `sudo apt install libxcb-cursor0`；找不到 Qt 6 时跳过 Studio，其余照常构建。

```bash
tools/fetch_mujoco.sh                                  # MuJoCo 3.8.0 → .deps/
python3 robots/rebot_b601/scripts/generate_description.py   # URDF/MJCF → robots/rebot_b601/generated/
```

## 核心模块：CMake 预设

不含 ROS 节点，用于开发和测试核心模块、仿真与运行时。

```bash
source /opt/ros/jazzy/setup.bash
cd larm
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

另有 `release`、`asan`、`tsan` 预设；后两者使用 Clang。

不经 ROS 在仿真中运行一条轨迹：

```bash
./build/debug/apps/sim_cli/larm_sim_cli \
  --profile ../robots/rebot_b601/config/rebot_b601_rs.yaml \
  --target 0.8,1.0,1.4,-0.5,0.4,1.0,0.04
```

输出 JSON 汇总（状态、跟踪误差、最终误差、单周期计算耗时、最终状态摘要）。`--realtime` 按墙钟节流，`--cancel-after SEC` 中途取消，`--trace FILE` 逐周期输出 JSONL。

## ROS 2 工作区：colcon

在仓库根目录：

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=$HOME/Qt/6.8.3/gcc_64:$CMAKE_PREFIX_PATH   # 使用官方 Qt 时，用于构建 Studio
colcon build --base-paths larm larm_msgs robots
colcon test --base-paths larm larm_msgs robots && colcon test-result --verbose
source install/setup.bash
ros2 launch rebot_b601 sim.launch.py          # rviz:=false 不开 RViz；real_time_factor:=2.0 加速
```

另开终端操作：

```bash
ros2 service call /larm_runtime/enable std_srvs/srv/Trigger
ros2 action send_goal /larm_runtime/arm/move_to_joints larm_msgs/action/MoveToJoints \
  "{positions: [0.6, 1.0, 1.4, -0.5, 0.4, 1.0], speed: 0.5}"
ros2 action send_goal /larm_runtime/gripper/gripper_command control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 10.0}}"
ros2 topic echo /larm_runtime/status
```

其余接口：`/larm_runtime/arm/follow_joint_trajectory`（`control_msgs/FollowJointTrajectory`）、`/larm_runtime/arm/move_to_pose`，服务 `disable`、`park`、`reset_fault`、`emergency_stop`。机械臂没有抱闸，`disable` 只在停放姿态下成功，先调用 `park`。

## Studio

仿真加 Studio，关闭 Studio 即结束整个 launch：

```bash
ros2 launch rebot_b601 studio.launch.py       # rviz:=true 同时开 RViz
```

也可以连接一个已经在运行的运行时节点：

```bash
ros2 run larm larm_studio --profile $(ros2 pkg prefix rebot_b601)/share/rebot_b601/config/rebot_b601_rs.yaml
```

右侧是会话面板（使能、停放、失能、复位、急停）和关节、笛卡尔、路径、夹爪面板；每个面板的"Stop"停止它正在执行的操作。视口左键旋转、右键平移、滚轮缩放，黄色标记是笛卡尔面板的目标位姿。

`--script [--screenshots DIR]` 用真实按钮自动走完主要流程并检查结果（`colcon test` 中的 `larm_studio_script` 默认在 `offscreen` 平台下运行它；设 `QT_QPA_PLATFORM=xcb` 可得到渲染后的截图）。
