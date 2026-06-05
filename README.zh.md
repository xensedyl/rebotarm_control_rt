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

`build.sh` 先用 CMake 编出 `_math.so`（链接 Pinocchio C++）落入包目录，再用 maturin 把
`_native`（Rust）+ `_math.so` + Python 打进同一个 wheel 并安装。

- **Pinocchio C++ 前缀自动探测**：
  `-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `/usr/local` > `/usr`。
  自动适配 `lib` 与 `lib/x86_64-linux-gnu`，自动定位 Eigen。**不依赖 ROS。**
- 运行期 RPATH 已写入 Pinocchio 库目录，无需 `LD_LIBRARY_PATH`。
- 从仓库根目录运行 example 时，脚本会自动把本地 `python/` 源码树加入 `sys.path`，
  因此安装 wheel 之前也可以先测试示例。

> ⚠️ 用 conda-forge 的 **Pinocchio 3.x**（`pinocchio>=3.2,<4`）。4.0 重排了头文件布局，
> 当前代码按 3.x 编写。

### 实时性说明

RT 循环是 **Rust 软实时**（`std::thread` + 绝对节拍 + overrun 计数 + 尽力 `SCHED_FIFO`）。
它全程释放 Python GIL，比 Python 线程稳定得多——但**不是** Flexiv RDK 那种硬实时栈。
要接近硬实时：在 PREEMPT_RT 内核上以 root 运行并设 `rt_priority`/`cpu`，
并监控 `arm.rt_send_overruns` / `arm.rt_read_overruns`。

## 示例程序

激活 Python 环境后，在仓库根目录运行：

```bash
cd rebotarm_control_rt
conda activate rebot
```

所有真机示例都支持 `--config/-c` 指定其他 arm/gripper YAML。从仓库根目录运行时，示例会自动
把本地 `python/` 源码树加入 `sys.path`。

### 调试工具

#### 1. 单电机控制台（`0x01damiao_test.py`、`1_damiao_text.py`）

交互式单关节终端。它通过 `RobotArm` 控制一个指定关节，同时让其他关节保持当前位置。
`1_damiao_text.py` 保留为与 `reBotArm_control_py` 同名的兼容入口。

**运行**：

```bash
python example/0x01damiao_test.py --joint 0
python example/1_damiao_text.py --joint joint1
```

**交互命令**：

| 命令 | 说明 |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | 给选中关节发送 MIT 目标 |
| `posvel <pos_deg> [vlim]` | 给选中关节发送 POS_VEL 目标 |
| `vel <vel_rad_s>` | 给选中关节发送速度指令 |
| `mode <mit|posvel|vel>` | 切换控制模式 |
| `enable` / `disable` | 使能或失能机械臂 |
| `set_zero` | 设置选中关节零点 |
| `state` | 打印选中关节的位置、速度、力矩 |

---

#### 2. 零点校准与状态监控（`2_zero_and_read.py`）

打印实时关节位置。若不加 `--skip-zero`，脚本会先要求确认，然后把当前姿态设为零点。

**运行**：

```bash
python example/2_zero_and_read.py --skip-zero
python example/2_zero_and_read.py
```

---

### 关节控制示例

#### 3. RT 原生 MIT 控制（`3_mit_control.py`）

以 MIT 模式启动 Rust RT 循环。Python 只通过 `set_targets` 更新目标缓存，Rust 线程按设定频率下发电机帧。

**运行**：

```bash
python example/3_mit_control.py --rate 150
```

**输入格式**：

```text
q1 q2 q3 q4 q5 q6 [kp kd]     # 关节角度单位为度
state                         # 打印当前状态和 RT overrun
q                             # 退出
```

---

#### 4. RT 原生 POS_VEL 控制（`4_pos_vel_control.py`）

以 POS_VEL 模式启动 Rust RT 循环。输入末尾可附加 `vlim`，用于覆盖本次命令的所有关节速度上限。

**运行**：

```bash
python example/4_pos_vel_control.py --rate 150
```

**输入格式**：

```text
q1 q2 q3 q4 q5 q6 [vlim]      # 关节角度单位为度，vlim 单位为 rad/s
state                         # 打印当前状态和 RT overrun
q                             # 退出
```

---

### 运动学测试

#### 5. 正运动学测试（`5_fk_test.py`）

根据 6 个关节角计算末端位姿。不连接硬件。

**运行**：

```bash
python example/5_fk_test.py
> 0 0 0 0 0 0
> 45 -30 15 -60 90 180
```

**输出**：

- 末端位置 `(x, y, z)`，单位米
- 旋转矩阵
- roll / pitch / yaw，单位度

---

#### 6. 逆运动学测试（`6_ik_test.py`）

根据目标末端位姿求解关节角。不连接硬件。

**输入格式**：

```text
x y z                         # 米，保持零位 FK 的姿态
x y z roll pitch yaw          # 米 + 度
```

**运行**：

```bash
python example/6_ik_test.py
> 0.2603 0.0 0.1917
> 0.2603 0.0 0.1917 0 0 0
```

---

### 真机控制

运行真机示例前，先检查设备权限：

```bash
# 达妙串口桥。先查看 USB 适配器实际枚举出来的 tty。
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/

# 然后修改 python/rebotarm_control_rt/config/arm.yaml 和 gripper.yaml：
#   channel: /dev/ttyACM0    # 或 /dev/ttyACM1、/dev/ttyACM2 等
sudo chmod 666 /dev/ttyACM0
# 或者直接
sudo chmod 666 /dev/ttyACM*

# 如果使用 SocketCAN
sudo ip link set can0 up type can bitrate 500000
```

#### 7. 末端 IK 控制（`7_arm_ik_control.py`）

使用 `ArmEndPos`：C++ 求解 IK，Rust RT 循环执行求解出的关节目标。

**运行**：

```bash
python example/7_arm_ik_control.py
> 0.3 0.0 0.2
> 0.3 0.1 0.25 0 0.5 0
```

**交互命令**：

| 命令 | 说明 |
|---|---|
| `x y z [roll pitch yaw]` | 目标末端位姿，姿态单位为弧度 |
| `state` | 打印当前关节状态和 RT overrun |
| `pos` / `end_state` | 打印当前末端位姿 |
| `q` / `quit` / `exit` | 退出 |

---

#### 8. 末端轨迹控制（`8_arm_traj_control.py`）

使用 `ArmEndPos` 轨迹模式。C++ 规划并跟踪笛卡尔轨迹，Rust RT 循环执行流式关节目标。

**输入格式**：

```text
x y z [roll pitch yaw] [duration]
```

**运行**：

```bash
python example/8_arm_traj_control.py
> 0.3 0.0 0.3 0 0.4 0 2.0
```

`7_arm_ik_control.py` 和 `8_arm_traj_control.py` 退出时会调用 `ArmEndPos.end()`，
也就是先执行 `safe_home()`，再断开连接。

---

#### 9. 重力补偿（`9_gravity_compensation.py`）

使用 C++ dynamics 模型计算重力前馈力矩，并通过 Python 回调循环发送 MIT 指令。

**控制律**：

```text
tau = g(q)
pos = 当前关节位置
循环指令中 kp = 0, kd = 1
```

**运行**：

```bash
python example/9_gravity_compensation.py --rate 200
```

默认 `use_gripper=true`：动力学模型会把 URDF 中固定在末端的 `end_link` 负载计入重力补偿，
并使用实测效果较好的 `0.7` 负载缩放。若机械臂未安装夹爪或末端负载，显式关闭：

```bash
python example/9_gravity_compensation.py --rate 200 --use_gripper=false
```

按 `Ctrl+C` 停止并断开。

---

#### 10. 带速度锁止的重力补偿（`10_gravity_compensation_lock.py`）

在重力补偿基础上加入末端速度锁止。TCP 速度低于阈值时锁定关节目标；用力推动超过阈值后更新锁定姿态。

**运行**：

```bash
python example/10_gravity_compensation_lock.py --rate 200
```

常用参数：

| 参数 | 说明 |
|---|---|
| `--vel-threshold` | TCP 线速度阈值 |
| `--w-threshold` | TCP 角速度阈值 |
| `--kp`, `--kd` | MIT 锁止刚度和阻尼 |
| `--integral-limit` | 积分力矩限幅 |
| `--use_gripper=true/false` | 是否计入固定 `end_link` 夹爪负载；默认 `true`，并使用标定后的 `0.7` 负载缩放 |

---

#### 11. 夹爪控制台（`gripper_test.py`）

夹爪交互终端，用于设零、切换模式、发送 MIT/POS_VEL/VEL 指令。

**运行**：

```bash
python example/gripper_test.py
```

**交互命令**：

| 命令 | 说明 |
|---|---|
| `z` | 将当前夹爪位置设为零点 |
| `m` | 切换 MIT / POS_VEL / VEL 模式 |
| `c` | 发送或更新控制指令 |
| `s` | 打印夹爪位置、速度、力矩 |
| `q` | 停止循环、失能并断开 |

---

### MeshCat 仿真

可选仿真示例位于 `example/sim/`。它们只是在可视化层需要 Python `meshcat` 和 Python
`pinocchio`；运动学和轨迹计算仍然走本包的 C++ 绑定。

```bash
pip install meshcat
conda install -c conda-forge "pinocchio>=3.2,<4"
```

如果当前 shell source 过 ROS，运行仿真示例时同时清掉 ROS 的 Python 路径和动态库路径。

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/sim/fk_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/sim/ik_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/sim/traj_sim.py
```

---

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
