# Python 示例

> For the English documentation, see [README.md](README.md)

这些示例使用 `rebotarm_control_rt` 的 Python API。激活环境后，从仓库根目录运行：

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
conda activate rebot
```

从仓库根目录运行时，脚本会自动把本地 `python/` 源码树加入 `sys.path`，因此安装 wheel 之前也可以先测试示例。

## 硬件准备

所有真机示例都支持 `--port` 在运行时覆盖 YAML 里的 `channel`，也支持
`--config/-c` 指定其他 arm/gripper YAML。

达妙串口桥运行前先确认实际端口，然后在命令里传 `--port`：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/

sudo chmod 666 /dev/ttyACM*

# 单臂示例：
#   --port /dev/ttyACM0
#
# 双臂通常使用不同端口，例如：
#   left:  --port /dev/ttyACM0
#   right: --port /dev/ttyACM1
```

如果希望不传 `--port` 也能用随包默认配置，再手动修改
`python/rebotarm_control_rt/config/arm.yaml` 和 `gripper.yaml`：
`channel: /dev/ttyACM0`。

灵足 / RobStride 通过 PCAN-USB 接入时，先按 1 Mbps 启动 SocketCAN，并使用
RobStride YAML 配置：

```bash
sudo modprobe peak_usb
ip -br link

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

灵足机械臂真机示例统一传 `--config python/rebotarm_control_rt/config/arm_rs.yaml --port can0`。灵足夹爪示例传
`--config python/rebotarm_control_rt/config/gripper_rs.yaml --port can0`。运动学和仿真示例不需要 `--port`，但如果要使用
灵足 URDF 和 `gripper_end` 末端 frame，仍然传 `--config python/rebotarm_control_rt/config/arm_rs.yaml`。

## 调试工具

### 1. 单电机控制台

`0x01damiao_test.py` 是交互式单关节终端。它通过 `RobotArm` 控制一个指定关节，同时让其他关节保持当前位置。
`1_damiao_text.py` 保留为与 `reBotArm_control_py` 同名的兼容入口。

```bash
python example/python/0x01damiao_test.py --port /dev/ttyACM0 --joint 0
python example/python/1_damiao_text.py --port /dev/ttyACM0 --joint joint1
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

### 1b. 灵足（RobStride）单电机控制台

`0x03robstride_test.py` 是单关节终端的灵足（RobStride）版本。默认加载随包的
`arm_rs.yaml`（灵足电机 + CAN 总线），启动时自动开启主动状态上报以获得持续反馈，
并在常规 MIT / POS_VEL / VEL 控制之外提供灵足底层命令。

```bash
python example/python/0x03robstride_test.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0 --joint joint1
```

首次小角度测试建议按顺序输入：

```text
ping
enable
state
mode posvel
posvel 3 0.3
state
posvel 0 0.3
disable
q
```

相比达妙控制台新增的交互命令：

| 命令 | 说明 |
|---|---|
| `ping` | ping 选中电机（type-0 GET_DEVICE_ID） |
| `clear_error` | 清除电机故障状态 |
| `csp <pos_deg> [vlim]` | 灵足原生 CSP 位置模式（run_mode=5），只驱动选中关节 |
| `report <on\|off>` | 开/关主动状态上报 |
| `read_param <id> [type]` | 读取 0x7000 参数表，例如 `read_param 0x7019` |
| `write_param <id> <value> [type]` | 写入参数，例如 `write_param 0x701E 13.0` |
| `save_params` | 保存参数（断电保持，type-22） |

POS_VEL 模式下，YAML 中的环路增益会在切换 `run_mode` 前写入灵足参数表
（`0x7017 limit_spd`、`0x701F spd_kp`、`0x7020 spd_ki`、`0x701E loc_kp`）；
灵足没有独立的位置环 Ki，`pos_ki` 字段被忽略。

### 2. 零点校准与状态监控

`2_zero_and_read.py` 打印实时关节位置。若不加 `--skip-zero`，脚本会先要求确认，然后把当前姿态设为零点。

```bash
python example/python/2_zero_and_read.py --port /dev/ttyACM0 --skip-zero
python example/python/2_zero_and_read.py --port /dev/ttyACM0

# 灵足 / RobStride
python example/python/2_zero_and_read.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0 --skip-zero
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
python example/python/3_mit_control.py --port /dev/ttyACM0 --rate 150

# 灵足 / RobStride
python example/python/3_mit_control.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0 --rate 150
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
python example/python/4_pos_vel_control.py --port /dev/ttyACM0 --rate 150

# 灵足 / RobStride
python example/python/4_pos_vel_control.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0 --rate 150
```

输入格式：bnvb

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

# 灵足 URDF / gripper_end 末端 frame
python example/python/5_fk_test.py --config python/rebotarm_control_rt/config/arm_rs.yaml
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

# 灵足 URDF / gripper_end 末端 frame
python example/python/6_ik_test.py --config python/rebotarm_control_rt/config/arm_rs.yaml
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
python example/python/7_arm_ik_control.py --port /dev/ttyACM0

# 灵足 / RobStride
python example/python/7_arm_ik_control.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0
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
python example/python/8_arm_traj_control.py --port /dev/ttyACM0

# 灵足 / RobStride
python example/python/8_arm_traj_control.py --config python/rebotarm_control_rt/config/arm_rs.yaml --port can0
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
python example/python/9_gravity_compensation.py --port /dev/ttyACM0 --rate 200

# 灵足 / RobStride
python example/python/9_gravity_compensation.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --port can0 \
  --rate 200 \
  --use_gripper=false
```

控制律：

```text
tau = g(q)
pos = 当前关节位置
循环指令中 kp = 0, kd = 1
```

默认 `use_gripper=true`：动力学模型会把 URDF 中固定在末端的 `end_link` 负载计入重力补偿，并使用当前 B601 夹爪配置下标定过的负载缩放。若机械臂未安装夹爪或等效末端负载，显式关闭：

```bash
python example/python/9_gravity_compensation.py --port /dev/ttyACM0 --rate 200 --use_gripper=false

# 灵足 / RobStride 机械臂本体模型
python example/python/9_gravity_compensation.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --port can0 \
  --rate 200 \
  --use_gripper=false
```

按 `Ctrl+C` 停止并断开。

### 11. 带速度锁止的重力补偿

`10_gravity_compensation_lock.py` 在重力补偿基础上加入末端速度锁止。TCP 速度低于阈值时锁定关节目标；用力推动超过阈值后更新锁定姿态。

```bash
python example/python/10_gravity_compensation_lock.py --port /dev/ttyACM0 --rate 200

# 灵足 / RobStride
python example/python/10_gravity_compensation_lock.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --port can0 \
  --rate 200 \
  --use_gripper=false
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
python example/python/gripper_test.py --port /dev/ttyACM0

# 灵足 / RobStride 夹爪
python example/python/gripper_test.py --config python/rebotarm_control_rt/config/gripper_rs.yaml --port can0
```

交互命令：

| 命令 | 说明 |
|---|---|
| `z` | 将当前夹爪位置设为零点 |
| `m` | 切换 MIT / POS_VEL / VEL 模式 |
| `c` | 发送或更新控制指令 |
| `s` | 打印夹爪位置、速度、力矩 |
| `q` | 停止循环、失能并断开 |

## 工具 TCP 标定

`tool_calibration.py` 用于标定新夹爪/工具相对法兰的 TCP 位姿，默认法兰 frame 为
`link6`。脚本会开启重力补偿自由拖动，采集至少 4 个固定点触碰姿态。默认只标定 TCP
平移，并沿用原 URDF 里 `end_joint` 的姿态。

```bash
python example/python/tool_calibration.py \
  --port /dev/ttyACM0 \
  --samples 4 \
  --kd 0.5 \
  --gravity-scale 1.0

# 灵足 / RobStride，默认会更新固定关节 j_gripper_end
python example/python/tool_calibration.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --port can0 \
  --frame link6 \
  --samples 4 \
  --kd 0.5 \
  --gravity-scale 1.0
```

输出内容：

- `calibration/tool_calibration.yaml`：数值标定结果
- `calibration/tool_calibration.urdf`：输入 URDF 的副本，只更新 `end_joint origin`
- `xyz_m` / `xyz_mm`：TCP 在法兰系下的平移
- `rpy_deg`：TCP 在法兰系下的姿态
- `T_flange_tool`：完整 4x4 变换
- `residual_mm`：4 点法残差

文件保存完成后，脚本会继续保持重力补偿自由拖动循环，不会马上下使能，避免机械臂突然失去支撑。
准备退出时按 `Ctrl+C`，脚本会停止控制循环、下使能、断开并退出。

使用生成的 URDF 加载模型：

```python
from rebotarm_control_rt.kinematics import load_robot_model

default_model = load_robot_model()                  # 原始 reBot-DevArm_fixend.urdf
tool_model = load_robot_model("tool_calibration.urdf")
```

这样原始 URDF 不会被修改。只传 `tool_calibration.urdf` 文件名时，`load_robot_model`
会自动到项目/本地 `calibration/` 目录下找同名文件，因此这个模型里的 `end_link`
会变成标定后的 TCP。

如果还需要标定工具姿态，额外加 `--calibrate-orientation`。四点平移标定后，脚本会要求
采集 +Z 和 +X 两个方向姿态。这两个方向必须不同；如果 +X 和 +Z 几乎一样，姿态无法求解，
脚本会退回保存只包含平移的结果。

自由拖动调参：

| 参数 | 说明 |
|---|---|
| `--gravity-scale` | 重力前馈力矩倍率，`tau = gravity_scale * g(q)`。先从 `1.0` 开始；如果机械臂往下沉，略微增大；如果自己往上飘，略微减小。建议每次按 `0.02` 到 `0.05` 小步调整。 |
| `--kd` | 自由拖动时 MIT 阻尼。值越大越稳、越不松；值越小越轻，但可能更容易抖。 |

## 动力学参数辨识

当前推荐用重力补偿自由拖动录轨迹，再用 POS_VEL 回放这条真实手拖轨迹采集
`q/dq/ddq/tau`。轨迹已经由人手在真实环境里检查过，比离线随机生成轨迹更安全。

`11_record_gravity_trajectory.py` 启动后会进入重力补偿。手拖到起点后按 Enter 开始录制，再按
Enter 停止录制并保存；保存后仍保持重力补偿，手拖回零点或安全姿态后按 `Ctrl+C` 退出。
如果机械臂下沉，略微增大 `--gravity-scale`；如果上飘，略微减小。

辨识数据 CSV 列格式固定为：

```text
time,q1,q2,q3,q4,q5,q6,dq1,...,dq6,ddq1,...,ddq6,tau1,...,tau6
```

单位分别是 rad、rad/s、rad/s^2、Nm。实际使用分成下面两套完整流程。

### 方案 A：先机械臂，再机械臂加夹爪

这个方案用于尽量把机械臂本体和新夹爪分开。第一阶段拆掉夹爪或确保末端不带额外负载；第二阶段装上新夹爪，只辨识 `end_link` 负载。这个方案最适合调重力补偿手感。

灵足 / RobStride 使用时，在录制和回放命令里加 `--config python/rebotarm_control_rt/config/arm_rs.yaml --port can0`。
灵足 URDF 使用 `gripper_end` / `j_gripper_end`，不是 B601 的 `end_link` 负载约定；
建议先做机械臂本体辨识。只有当你已经给灵足 URDF 添加了工具负载 inertial 时，再显式选择对应 payload link。

#### A1. 录制机械臂本体轨迹

```bash
python example/python/11_record_gravity_trajectory.py \
  --output calibration/arm_only_trajectory.csv \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0 \
  --use_gripper=false

# 灵足 / RobStride
python example/python/11_record_gravity_trajectory.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --output calibration/lingzu_arm_only_trajectory.csv \
  --port can0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0 \
  --use_gripper=false
```

#### A2. 预览机械臂本体轨迹

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/arm_only_trajectory.csv \
  --output calibration/id_data_arm_only.csv \
  --port /dev/ttyACM0

# 灵足 / RobStride 预览
python example/python/12_collect_identification_data.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --trajectory calibration/lingzu_arm_only_trajectory.csv \
  --output calibration/id_data_lingzu_arm_only.csv \
  --port can0
```

#### A3. 回放并采集机械臂本体数据

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/arm_only_trajectory.csv \
  --output calibration/id_data_arm_only.csv \
  --port /dev/ttyACM0 \
  --execute

# 灵足 / RobStride 回放
python example/python/12_collect_identification_data.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --trajectory calibration/lingzu_arm_only_trajectory.csv \
  --output calibration/id_data_lingzu_arm_only.csv \
  --port can0 \
  --execute
```

#### A4. 辨识机械臂本体

```bash
python example/python/13_identify_dynamics.py \
  --data calibration/id_data_arm_only.csv \
  --mode full \
  --ignore-payload-link end_link \
  --output calibration/identified_arm.yaml \
  --urdf-output calibration/identified_arm.urdf

# 灵足 / RobStride 机械臂本体
python example/python/13_identify_dynamics.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --data calibration/id_data_lingzu_arm_only.csv \
  --mode full \
  --output calibration/identified_lingzu_arm.yaml \
  --urdf-output calibration/identified_lingzu_arm.urdf
```

`--ignore-payload-link end_link` 会在辨识时临时移除 `end_link` 的 inertial，避免旧夹爪参数污染机械臂本体结果。

#### A5. 装上新夹爪后录制轨迹

```bash
python example/python/11_record_gravity_trajectory.py \
  --output calibration/arm_with_gripper_trajectory.csv \
  --urdf calibration/tool_calibration.urdf \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0
```

#### A6. 预览机械臂加夹爪轨迹

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/arm_with_gripper_trajectory.csv \
  --output calibration/id_data_arm_with_gripper.csv \
  --port /dev/ttyACM0
```

#### A7. 回放并采集机械臂加夹爪数据

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/arm_with_gripper_trajectory.csv \
  --output calibration/id_data_arm_with_gripper.csv \
  --port /dev/ttyACM0 \
  --execute
```

#### A8. 只辨识夹爪负载

```bash
python example/python/13_identify_dynamics.py \
  --data calibration/id_data_arm_with_gripper.csv \
  --urdf calibration/identified_arm.urdf \
  --mode payload \
  --payload-link end_link \
  --payload-parameters 4 \
  --output calibration/identified_gripper_payload.yaml \
  --urdf-output calibration/identified_arm_with_gripper.urdf
```

如果轨迹里速度和加速度激励足够，也可以把最后一条命令中的 `--payload-parameters 4` 改成
`--payload-parameters 10`。4 参数只辨识质量和质心，更适合先修重力补偿；10 参数会辨识完整惯量，但更容易受数据质量影响。

#### A9. 用辨识后的 URDF 测试重力补偿

```bash
python example/python/9_gravity_compensation.py \
  --port /dev/ttyACM0 \
  --rate 200 \
  --kd 1.0 \
  --urdf calibration/identified_arm_with_gripper.urdf

# 灵足 / RobStride 辨识后的机械臂模型
python example/python/9_gravity_compensation.py \
  --config python/rebotarm_control_rt/config/arm_rs.yaml \
  --port can0 \
  --rate 200 \
  --kd 1.0 \
  --urdf calibration/identified_lingzu_arm.urdf
```

### 方案 B：机械臂加夹爪一起辨识

这个方案更快，适合你认为机械臂和夹爪都不准，并且接受 `link6` 和 fixed `end_link` 参数强耦合的情况。它能直接得到一个整体拟合模型，但物理参数分配不一定唯一。

#### B1. 录制机械臂加夹爪轨迹

```bash
python example/python/11_record_gravity_trajectory.py \
  --output calibration/all_with_gripper_trajectory.csv \
  --urdf calibration/tool_calibration.urdf \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0
```

#### B2. 预览轨迹

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/all_with_gripper_trajectory.csv \
  --output calibration/id_data_all_with_gripper.csv \
  --port /dev/ttyACM0
```

#### B3. 回放并采集数据

```bash
python example/python/12_collect_identification_data.py \
  --trajectory calibration/all_with_gripper_trajectory.csv \
  --output calibration/id_data_all_with_gripper.csv \
  --port /dev/ttyACM0 \
  --execute
```

#### B4. 机械臂和夹爪一起辨识

```bash
python example/python/13_identify_dynamics.py \
  --data calibration/id_data_all_with_gripper.csv \
  --urdf calibration/tool_calibration.urdf \
  --mode full \
  --output calibration/identified_dynamics_all.yaml \
  --urdf-output calibration/identified_dynamics_all.urdf
```

#### B5. 用整体辨识 URDF 测试重力补偿

```bash
python example/python/9_gravity_compensation.py \
  --port /dev/ttyACM0 \
  --rate 200 \
  --kd 1.0 \
  --urdf calibration/identified_dynamics_all.urdf
```

常用参数：

| 参数 | 说明 |
|---|---|
| `--mode full` | 机械臂和夹爪一起辨识完整动态参数向量，可以写回 URDF。 |
| `--mode base` | 使用 QR 选择独立最小参数集，数值上通常更稳，但不能唯一写回每个 link 的 URDF 惯量。 |
| `--mode payload` | 固定机械臂 URDF，只辨识一个固定负载 link，默认是 `end_link`。适合两阶段方案的第二步。 |
| `--urdf-output` | 基于输入 URDF 输出一份写入辨识惯量的新 URDF。`full` 和 `payload` 可用。`base` 不能唯一写回 URDF。 |
| `--payload-parameters 4` | 只辨识夹爪质量和质心，推荐先用这个修重力补偿。 |
| `--payload-parameters 10` | 辨识夹爪完整 10 参数惯量，需要更丰富的动态激励。 |
| `--ignore-payload-link end_link` | 机械臂本体辨识时临时忽略末端固定负载。 |
| `--no-friction` | 只辨识刚体惯性参数。默认包含粘性摩擦和光滑库伦摩擦列。 |
| `--no-model-prior` | full 模式下使用纯最小范数最小二乘。默认会让不可辨识零空间保持接近输入 URDF，更适合写回 URDF。 |

`14_verify_identification.py` 只支持 `full/base` 结果。如果你有另一段验证数据，可以验证方案 B 或方案 A 的机械臂本体结果：

```bash
python example/python/14_verify_identification.py \
  --data calibration/id_data_verify.csv \
  --params calibration/identified_dynamics_all.yaml
```

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

# 灵足 URDF
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/fk_sim.py --config python/rebotarm_control_rt/config/arm_rs.yaml
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/ik_sim.py --config python/rebotarm_control_rt/config/arm_rs.yaml
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/traj_sim.py --config python/rebotarm_control_rt/config/arm_rs.yaml
```
