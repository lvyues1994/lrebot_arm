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

## 真机：RobStride over SocketCAN（尚未在真机上验证）

所有带 `--simulated` 或 `backend:=robstride_simulated` 的命令都只在模拟电机上运行，不接触硬件，可以先用来演练。

准备 CAN 接口（PCAN-USB，1 Mbit/s）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

调试探针 `larm_driver_probe`（`ros2 run larm larm_driver_probe ...`，或预设构建目录下的 `apps/driver_probe/larm_driver_probe`），输出 JSON Lines：

```bash
P=$(ros2 pkg prefix rebot_b601)/share/rebot_b601/config/rebot_b601_rs.yaml
larm_driver_probe --profile $P scan                       # 只读：ping、run_mode、zero_sta、电压、位置
larm_driver_probe --profile $P monitor 10                 # 只读：10 s 位置采样（可手动推动关节核对方向）
larm_driver_probe --profile $P hold 5 --confirm-power     # 在停放姿态使能、保持 5 s、失能
larm_driver_probe --profile $P jog joint1 0.1 --confirm-motion --output jog.jsonl
```

`hold` 和 `jog` 在机械臂不在停放姿态、反馈不全或已有故障时拒绝执行。Ctrl+C 中断运动后，探针会把机械臂停放并失能；这一收尾不再被打断，紧急情况用硬件急停。

运行时节点接真机：`ros2 launch rebot_b601 robot.launch.py`（`rt_priority:=80` 使用 SCHED_FIFO，需要相应权限；`rviz:=true` 打开 RViz）。Studio 可以用 `ros2 run larm larm_studio --profile $P` 连接。

上真机按以下顺序逐级进行，每一级都需要操作者确认后才进入：

1. 只读：`scan` 全部应答，`run_mode` 为 0，`zero_sta` 为 1，电压正常；用 MotorBridge Studio 确认固件版本（旧固件的 Kp/Kd 有 1.4167 倍的换算错误）。
2. 只读：`monitor` 下手动推动每个关节，核对方向、零点，以及夹爪传动（7.353 mm/rad，方向待定）。
3. 使能保持：机械臂在停放姿态，`hold 5 --confirm-power`；核对重力前馈下的反馈力矩、`missed_replies`、`missed_periods`，用 `canbusload` 实测总线负载。
4. 单关节小幅运动：`jog` 逐个关节，从 0.05 rad 开始；在机械臂离开桌面的姿态下，比较反馈速度与位置差分。
5. 慢速轨迹：`robot.launch.py` 加 Studio 或命令行，低速走完关节运动、停放、失能。

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
