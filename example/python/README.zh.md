# Python 示例

> For the English documentation, see [README.md](README.md)

这些示例使用 `rebotarm_control_rt` 的 Python API。激活环境后，从仓库根目录运行：

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
conda activate rebot
```

从仓库根目录运行时，脚本会自动把本地 `python/` 源码树加入 `sys.path`，因此安装 wheel 之前也可以先测试示例。

## 硬件准备

所有真机示例都支持 `--config/-c` 指定其他 arm/gripper YAML。

达妙串口桥运行前先确认实际端口，再按需修改随包配置：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/

# 然后修改 python/rebotarm_control_rt/config/arm.yaml 和 gripper.yaml:
#   channel: /dev/ttyACM0    # 或 /dev/ttyACM1、/dev/ttyACM2 等
sudo chmod 666 /dev/ttyACM*
```

SocketCAN 使用前：

```bash
sudo ip link set can0 up type can bitrate 500000
```

## 调试工具

### 1. 单电机控制台

`0x01damiao_test.py` 是交互式单关节终端。它通过 `RobotArm` 控制一个指定关节，同时让其他关节保持当前位置。
`1_damiao_text.py` 保留为与 `reBotArm_control_py` 同名的兼容入口。

```bash
python example/python/0x01damiao_test.py --joint 0
python example/python/1_damiao_text.py --joint joint1
```

交互命令：

| 命令 | 说明 |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | 给选中关节发送 MIT 目标 |
| `posvel <pos_deg> [vlim]` | 给选中关节发送 POS_VEL 目标 |
| `vel <vel_rad_s>` | 给选中关节发送速度指令 |
| `mode <mit|posvel|vel>` | 切换控制模式 |
| `enable` / `disable` | 使能或失能机械臂 |
| `set_zero` | 设置选中关节零点 |
| `state` | 打印选中关节的位置、速度、力矩 |
| `q` | 停止并断开 |

### 2. 零点校准与状态监控

`2_zero_and_read.py` 打印实时关节位置。若不加 `--skip-zero`，脚本会先要求确认，然后把当前姿态设为零点。

```bash
python example/python/2_zero_and_read.py --skip-zero
python example/python/2_zero_and_read.py
```

### 3. 达妙 POS_VEL 参数寄存器读取

`0x02_read_damiao_pd.py` 通过 SDK 直接读取达妙电机当前 POS_VEL 增益寄存器。`--default-bi` 会从端口临时生成 B601 配置，不依赖 LeRobot 生成的缓存 yaml。

达妙寄存器对应关系：

| 寄存器 | 名称 |
|---|---|
| `25` | `vel_kp` / `KP_ASR` |
| `26` | `vel_ki` / `KI_ASR` |
| `27` | `pos_kp` / `KP_APR` |
| `28` | `pos_ki` / `KI_APR` |

运行方式：

```bash
# 双 B601，默认读取 /dev/ttyACM0 和 /dev/ttyACM1
python example/python/0x02_read_damiao_pd.py --default-bi --timeout-ms 300

# 双 B601，显式指定左右端口
python example/python/0x02_read_damiao_pd.py \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# 读取一个或多个指定端口
python example/python/0x02_read_damiao_pd.py --port /dev/ttyACM0
python example/python/0x02_read_damiao_pd.py --port /dev/ttyACM0 --port /dev/ttyACM1

# 通过指定 arm YAML 读取
python example/python/0x02_read_damiao_pd.py --config python/rebotarm_control_rt/config/arm.yaml
```

## 关节控制

### 4. RT 原生 MIT 控制

`3_mit_control.py` 以 MIT 模式启动 Rust RT 循环。Python 只通过 `set_targets` 更新目标缓存，Rust 线程按设定频率下发电机帧。

```bash
python example/python/3_mit_control.py --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [kp kd]     # 关节角度单位为度
state                         # 打印当前状态和 RT overrun
q                             # 退出
```

### 5. RT 原生 POS_VEL 控制

`4_pos_vel_control.py` 以 POS_VEL 模式启动 Rust RT 循环。输入末尾可附加 `vlim`，用于覆盖本次命令的所有关节速度上限。

```bash
python example/python/4_pos_vel_control.py --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [vlim]      # 关节角度单位为度，vlim 单位为 rad/s
state                         # 打印当前状态和 RT overrun
q                             # 退出
```

## 运动学测试

### 6. 正运动学

`5_fk_test.py` 根据 6 个关节角计算末端位姿。不连接硬件。

```bash
python example/python/5_fk_test.py
```

示例输入：

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

输出内容：

- 末端位置 `(x, y, z)`，单位米
- 旋转矩阵
- roll / pitch / yaw，单位度

### 7. 逆运动学

`6_ik_test.py` 根据目标末端位姿求解关节角。不连接硬件。

```bash
python example/python/6_ik_test.py
```

输入格式：

```text
x y z                         # 米，保持零位 FK 的姿态
x y z roll pitch yaw          # 米 + 度
```

示例输入：

```text
0.2603 0.0 0.1917
0.2603 0.0 0.1917 0 0 0
```

## 真机末端控制

### 8. 末端 IK 控制

`7_arm_ik_control.py` 使用 `ArmEndPos`：C++ 求解 IK，Rust RT 循环执行求解出的关节目标。

```bash
python example/python/7_arm_ik_control.py
```

示例输入：

```text
0.3 0.0 0.2
0.3 0.1 0.25 0 0.5 0
```

交互命令：

| 命令 | 说明 |
|---|---|
| `x y z [roll pitch yaw]` | 目标末端位姿，姿态单位为弧度 |
| `state` | 打印当前关节状态和 RT overrun |
| `pos` / `end_state` | 打印当前末端位姿 |
| `q` / `quit` / `exit` | 退出 |

### 9. 末端轨迹控制

`8_arm_traj_control.py` 使用 `ArmEndPos` 轨迹模式。C++ 规划并跟踪笛卡尔轨迹，Rust RT 循环执行流式关节目标。

```bash
python example/python/8_arm_traj_control.py
```

输入格式：

```text
x y z [roll pitch yaw] [duration]
```

示例输入：

```text
0.3 0.0 0.3 0 0.4 0 2.0
```

`7_arm_ik_control.py` 和 `8_arm_traj_control.py` 退出时会调用 `ArmEndPos.end()`，也就是先执行 `safe_home()`，再断开连接。

## 重力补偿

### 10. 基础重力补偿

`9_gravity_compensation.py` 使用 C++ dynamics 模型计算重力前馈力矩，并通过 Python 回调循环发送 MIT 指令。

```bash
python example/python/9_gravity_compensation.py --rate 200
```

控制律：

```text
tau = g(q)
pos = 当前关节位置
循环指令中 kp = 0, kd = 1
```

默认 `use_gripper=true`：动力学模型会把 URDF 中固定在末端的 `end_link` 负载计入重力补偿，并使用当前 B601 夹爪配置下标定过的负载缩放。若机械臂未安装夹爪或等效末端负载，显式关闭：

```bash
python example/python/9_gravity_compensation.py --rate 200 --use_gripper=false
```

按 `Ctrl+C` 停止并断开。

### 11. 带速度锁止的重力补偿

`10_gravity_compensation_lock.py` 在重力补偿基础上加入末端速度锁止。TCP 速度低于阈值时锁定关节目标；用力推动超过阈值后更新锁定姿态。

```bash
python example/python/10_gravity_compensation_lock.py --rate 200
```

常用参数：

| 参数 | 说明 |
|---|---|
| `--vel-threshold` | TCP 线速度阈值 |
| `--w-threshold` | TCP 角速度阈值 |
| `--kp`, `--kd` | MIT 锁止刚度和阻尼 |
| `--integral-limit` | 积分力矩限幅 |
| `--use_gripper=true/false` | 是否计入固定 `end_link` 夹爪负载；默认 `true` |

## 夹爪控制台

`gripper_test.py` 是夹爪交互终端，用于设零、切换模式、发送 MIT/POS_VEL/VEL 指令。

```bash
python example/python/gripper_test.py
```

交互命令：

| 命令 | 说明 |
|---|---|
| `z` | 将当前夹爪位置设为零点 |
| `m` | 切换 MIT / POS_VEL / VEL 模式 |
| `c` | 发送或更新控制指令 |
| `s` | 打印夹爪位置、速度、力矩 |
| `q` | 停止循环、失能并断开 |

## MeshCat 仿真

可选仿真示例位于 `example/python/sim/`。它们只是在可视化层需要 Python `meshcat` 和 Python `pinocchio`；运动学和轨迹计算仍然走本包的 C++ 绑定。

```bash
pip install meshcat
conda install -c conda-forge "pinocchio>=3.2,<4"
```

如果当前 shell source 过 ROS，运行仿真示例时同时清掉 ROS 的 Python 路径和动态库路径：

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/fk_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/ik_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/traj_sim.py
```
