# rebotarm_control_rt

> For the English documentation, see [README.md](README.md)

**reBot Arm B601** 控制库。所有性能关键层均为原生实现，**Python 仅作上层接口**。

| 层 | 语言 | 职责 |
|---|---|---|
| `_native`（actuator） | **Rust / PyO3** | 电机控制。直接以 Cargo `path` 依赖 [motorbridge](../motorbridge) 的 vendor crates——无 `ctypes` / C-ABI 中转。 |
| `_math`（kinematics / dynamics / trajectory / controllers） | **C++ / pybind11** | 直接链接 **Pinocchio C++**（非 Python 绑定），摆脱了 Python `pinocchio` 依赖。 |

接口与 `reBotArm_control_py` 保持一致，可直接替换。

---

## 项目结构

```
rebotarm_control_rt/
├── src/                                  # Rust actuator 内核 → _native.so
│   ├── lib.rs
│   ├── config.rs
│   ├── vendor.rs
│   ├── arm.rs
│   └── gripper.rs
├── cpp/                                  # C++ 数学层（Pinocchio）→ _math.so
│   ├── CMakeLists.txt
│   ├── include/rebotarm/                 #   robot_model, dynamics, trajectory, arm_endpos, se3_conv (.hpp)
│   └── src/                              #   bindings, robot_model, dynamics, trajectory, arm_endpos (.cpp)
├── python/rebotarm_control_rt/
│   ├── actuator/                         #   re-export _native（RobotArm / Gripper / ...）
│   ├── kinematics/                       #   re-export _math（纯绑定）
│   ├── dynamics/
│   ├── trajectory/
│   ├── controllers/
│   ├── config/                           #   随包安装
│   └── urdf/
├── example/
│   ├── python/                          # Python 示例和 MeshCat 仿真
│   └── rust/                            # 使用 motorbridge + C++ math C ABI 的 Rust 示例
├── tests/
└── build.sh
```

---

## 构建与安装

全新机器一键部署（两阶段；详见 [INSTALL.md](INSTALL.md)）：

```bash
# 阶段 1：建环境 + Pinocchio C++ 3.x（--conda 亦可；可加 python 版本）
bash ./setup_env.sh --mamba rebot 3.10

# 阶段 2：激活后安装（rust/maturin → build.sh --wheel → 自检）
mamba activate rebot          # 或：conda activate rebot
bash ./setup_env.sh --install
```

### 构建流程

`build.sh` 先用 CMake 编出 `librebotarm_math.so` 和 `_math.so`（链接 Pinocchio C++）落入包目录，
再用 maturin 把 `_native`（Rust）+ `_math.so` + Python 打进同一个 wheel 并安装。Rust 示例需要
FK、IK 或重力补偿时，会通过 C ABI 直接动态加载 `librebotarm_math.so`。

- **Pinocchio C++ 前缀自动探测**：
  `-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `/usr/local` > `/usr`。
  自动适配 `lib` 与 `lib/x86_64-linux-gnu`，自动定位 Eigen。**不依赖 ROS。**
- 运行期 RPATH 已写入 Pinocchio 库目录，无需 `LD_LIBRARY_PATH`。
- 从仓库根目录运行 example 时，脚本会自动把本地 `python/` 源码树加入 `sys.path`，
  因此安装 wheel 之前也可以先测试示例。

> ⚠️ 用 conda-forge 的 **Pinocchio 3.x**（`pinocchio>=3.2,<4`）。4.0 重排了头文件布局，
> 当前代码按 3.x 编写。

### 构建检查

执行 `setup_env.sh --install` 或 `./build.sh --wheel` 后，先激活安装时使用的同一个环境，再检查两个
native module 是否都能导入：

```bash
conda activate rebot
python -c "import rebotarm_control_rt._math, rebotarm_control_rt._native; print('ok')"
```

### 实时性说明

RT 循环是 **Rust 软实时**（`std::thread` + 绝对节拍 + overrun 计数 + 尽力 `SCHED_FIFO`）。
它全程释放 Python GIL，比 Python 线程稳定得多——但**不是** Flexiv RDK 那种硬实时栈。
要接近硬实时：在 PREEMPT_RT 内核上以 root 运行并设 `rt_priority`/`cpu`，
并监控 `arm.rt_send_overruns` / `arm.rt_read_overruns`。

## 示例程序

示例按语言拆分，并在各自目录内维护具体用法：

- Python 示例：[example/python/README.zh.md](example/python/README.zh.md)
- Rust 示例：[example/rust/README.zh.md](example/rust/README.zh.md)

英文示例文档见 [example/python/README.md](example/python/README.md) 和
[example/rust/README.md](example/rust/README.md)。

激活环境后，从仓库根目录运行示例。Python 示例从仓库根目录运行时会自动把本地 `python/` 源码树加入 `sys.path`。

## 用法

```python
import numpy as np
from rebotarm_control_rt.actuator import RobotArm
from rebotarm_control_rt.kinematics import RobotModel, compute_ik
from rebotarm_control_rt.dynamics import compute_generalized_gravity, load_dynamics_model
from rebotarm_control_rt.controllers import ArmEndPos

arm = RobotArm()                      # 默认读取包内 config/arm.yaml
arm.connect(); arm.enable(); arm.mode_mit()
```

### 控制循环

```python
# 兼容模式：Python 回调控制循环（与 reBotArm_control_py 一致）
arm.start_control_loop(lambda a, dt: a.mit(np.zeros(arm.num_joints)))

# RT 原生模式：控制循环跑在 Rust 线程，全程释放 GIL。
#   - 未先 set_targets 时，自动读取当前关节位置作为 hold 目标（不会拉向 0 位姿）。
#   - request_feedback 默认 False；常规使用 motorbridge 后台缓存反馈。
#   - command_gap_us 可在逐关节控制帧之间插入短延时，必要时降低总线压力。
#   - rt_priority>0：尽力申请 SCHED_FIFO（需 root/CAP_SYS_NICE；本机为 PREEMPT_RT 内核）。
#   - cpu：可选 CPU 亲和性。
arm.mode_pos_vel()
arm.start_rt_loop(rate=150.0, rt_priority=0, cpu=None, command_gap_us=0)
arm.set_targets(pos=np.zeros(arm.num_joints))   # 之后随时更新目标
print("send/read overruns:", arm.rt_send_overruns, arm.rt_read_overruns)

arm.stop_control_loop(); arm.disconnect()
```

如果需要在 motorbridge 后台 polling 之外主动请求反馈，显式开启可选反馈线程：

```python
arm.start_rt_loop(rate=150.0, request_feedback=True, feedback_rate=60.0)
```

### 末端位置编排

```python
# IK + 轨迹：计算在 C++，驱动 Rust arm。
with ArmEndPos(arm) as ep:
    ep.move_to_ik(x=0.3, y=0.0, z=0.3)
    ep.move_to_traj(x=0.3, y=0.0, z=0.3, pitch=0.4, duration=2.0)
```

位姿在 Python 边界统一为 **4×4 numpy 齐次矩阵**（不暴露 `pinocchio.SE3`）。

---

## 厂商支持（actuator）

- **Damiao** —— 主路径，串口桥 `/dev/tty*` 921600。
- **MyActuator / RobStride / HighTorque** —— CAN。

状态归一化、模式映射、控制指令派发均与 `motor_abi` 的 C-ABI 实现保持一致。

## 致谢

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm.git)：提供了本仓库内置 URDF、mesh、运动学、动力学和仿真示例使用的 B601 机械臂模型资源。
- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py.git)：提供了本仓库保持兼容的 Python API 形态和示例流程参考，本仓库在此基础上替换为原生 RT actuator 与 C++ 数学后端。
- [motorbridge](https://github.com/motorbridge/motorbridge.git)：提供了 actuator 层直接依赖的 Rust 电机厂商 crate，用于 Damiao、MyActuator、RobStride、HighTorque 的通信和控制指令派发。
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git)：提供了 `_math` 链接使用的 C++ 运动学与动力学后端；本仓库直接使用 C++ 库，不依赖 Python binding。
