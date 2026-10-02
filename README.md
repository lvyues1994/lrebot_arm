# lrebot_arm

通用机械臂框架 `larm`，第一个落地机器人是 reBot Arm B601-RS。设计见 [docs/architecture.md](docs/architecture.md)。

## 构建与测试

依赖 Ubuntu 24.04、ROS 2 Jazzy（Pinocchio、Ruckig 取自其安装目录）、`libyaml-cpp-dev`、`libexpected-dev`、`libgtest-dev`、`libeigen3-dev`、CMake 3.25+、Ninja。

```bash
tools/fetch_mujoco.sh                                                        # MuJoCo 3.8.0 → .deps/
python3 robots/rebot_b601/rebot_b601_description/scripts/generate_description.py  # URDF/MJCF → generated/

source /opt/ros/jazzy/setup.bash
cd larm
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

另有 `release`、`asan`、`tsan` 预设；后两者使用 Clang。

## 仿真中运行一条轨迹

```bash
./build/debug/apps/sim_cli/larm_sim_cli \
  --profile ../robots/rebot_b601/rebot_b601_bringup/config/rebot_b601_rs.yaml \
  --target 0.8,1.0,1.4,-0.5,0.4,1.0,0.04
```

从配置里的停放姿态使能，按点到点轨迹走到目标，输出 JSON 汇总（状态、跟踪误差、最终误差、单周期计算耗时、最终状态摘要）。`--realtime` 按墙钟节流，`--cancel-after SEC` 中途取消，`--trace FILE` 逐周期输出 JSONL。
