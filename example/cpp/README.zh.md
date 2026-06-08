# C++ 示例

> For the English documentation, see [README.md](README.md)

这些示例是 Python / Rust 示例的 C++ 版本，不导入 Python。电机通信使用 `motorbridge`
C++ binding 和 `motor_abi`；FK、IK、轨迹和重力补偿使用 `librebotarm_math.so` 中的
原生 C++/Pinocchio 后端。

从仓库根目录运行：

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
conda activate rebot
```

## 构建

先构建数学后端：

```bash
PY=/home/xense/miniforge3/envs/rebot/bin/python ./build.sh
```

构建硬件示例需要的 `motor_abi`：

```bash
cargo build --manifest-path ../motorbridge/motor_abi/Cargo.toml --release
```

再构建 C++ 示例：

```bash
cmake -S example/cpp -B example/cpp/build \
  -DPINOCCHIO_PREFIX=/home/xense/miniforge3/envs/rebot
cmake --build example/cpp/build -j"$(nproc)"
```

如果 conda 环境已经激活，且 `CONDA_PREFIX` 指向同一个环境，`-DPINOCCHIO_PREFIX=...`
可以省略。

## 硬件准备

Damiao 串口桥先确认 USB 设备名：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
sudo chmod 666 /dev/ttyACM*
```

每个硬件示例都通过 `--port` 指定端口：

```bash
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0 --skip-zero
```

如果使用 SocketCAN，也可以传 CAN 通道：

```bash
sudo ip link set can0 up type can bitrate 500000
./example/cpp/build/2_zero_and_read --port can0 --skip-zero
```

## 调试工具

### 1. 单电机控制台

`0x01damiao_test` 是交互式单关节终端。它打开一条 B601 机械臂，选择一个关节，然后发送
MIT、POS_VEL、速度和 FORCE_POS 指令。`1_damiao_text` 保留为兼容文件名入口。

```bash
./example/cpp/build/0x01damiao_test --port /dev/ttyACM0 --joint 0
./example/cpp/build/1_damiao_text --port /dev/ttyACM0 --joint joint1
```

交互命令：

| 命令 | 作用 |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | 给选中关节发送 MIT 目标 |
| `posvel <pos_deg> [vlim]` | 给选中关节发送 POS_VEL 目标 |
| `vel <vel_rad_s>` | 给选中关节发送速度命令 |
| `forcepos <pos_deg> [vlim ratio]` | FORCE_POS 命令，适合夹爪限力位置控制 |
| `mode <mit|posvel|vel|forcepos>` | 切换选中关节控制模式 |
| `enable` / `disable` | 使能或失能机械臂 |
| `set_zero` | 确认后给选中关节置零 |
| `state` | 打印关节位置、速度、力矩和状态 |
| `q` | 停止并断开 |

### 2. 置零和状态读取

`2_zero_and_read` 打印当前关节状态。如果不传 `--skip-zero`，会先要求确认，然后把当前姿态
设置为所有 B601 电机的零点。

```bash
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0 --skip-zero
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0
```

### 3. Damiao POS_VEL 增益寄存器读取

`0x02_read_damiao_pd` 直接从电机读取当前 Damiao POS_VEL 增益寄存器。它不会使能电机，也不会
发送运动命令。

Damiao 寄存器：

| 寄存器 | 名称 |
|---|---|
| `25` | `vel_kp` / `KP_ASR` |
| `26` | `vel_ki` / `KI_ASR` |
| `27` | `pos_kp` / `KP_APR` |
| `28` | `pos_ki` / `KI_APR` |

用法：

```bash
# 双 B601，默认读取 /dev/ttyACM0 和 /dev/ttyACM1
./example/cpp/build/0x02_read_damiao_pd --default-bi --timeout-ms 300

# 双 B601，显式指定左右端口
./example/cpp/build/0x02_read_damiao_pd \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# 读取一个或多个任意端口
./example/cpp/build/0x02_read_damiao_pd --port /dev/ttyACM0
./example/cpp/build/0x02_read_damiao_pd --port /dev/ttyACM0 --port /dev/ttyACM1
```

## 关节控制

### 4. MIT 控制

`3_mit_control` 启动一个 C++ 线程，按设定频率持续发送 MIT 指令；终端只更新目标缓冲区。

```bash
./example/cpp/build/3_mit_control --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [gripper] [kp kd]     # 角度单位为度
state                                    # 打印当前状态
q                                        # 退出
```

### 5. POS_VEL 控制

`4_pos_vel_control` 启动一个 C++ 线程，按设定频率持续发送 POS_VEL 指令。

```bash
./example/cpp/build/4_pos_vel_control --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [gripper] [vlim]       # 角度单位为度，vlim 单位 rad/s
state                                    # 打印当前状态
q                                        # 退出
```

## 运动学测试

### 6. 正运动学

`5_fk_test` 根据 6 个关节角计算末端位姿，不连接硬件。

```bash
./example/cpp/build/5_fk_test
```

示例输入：

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

### 7. 逆运动学

`6_ik_test` 求解 IK。输入 `x y z` 时保持 neutral 姿态的朝向；输入
`x y z roll pitch yaw` 时姿态单位为弧度。不连接硬件。

```bash
./example/cpp/build/6_ik_test
```

示例输入：

```text
0.2603 0.0 0.1917
0.2603 0.0 0.1917 0 0 0
```

## 真机末端控制

### 8. 末端 IK 控制

`7_arm_ik_control` 读取当前关节位置，用 C++/Pinocchio IK 求解目标位置，然后用 POS_VEL
流式执行关节空间运动。

```bash
./example/cpp/build/7_arm_ik_control --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
x y z                         # 单位 m
state
q
```

### 9. 末端轨迹控制

`8_arm_traj_control` 用 C++/Pinocchio IK 求解目标位置，并按指定时长做关节空间插值。

```bash
./example/cpp/build/8_arm_traj_control --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
x y z [duration]              # 单位 m、秒
state
q
```

## 重力补偿

### 10. 基础重力补偿

`9_gravity_compensation` 用 C++/Pinocchio 动力学后端计算广义重力项，并发送 MIT 力矩前馈。

```bash
./example/cpp/build/9_gravity_compensation \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true
```

重要参数：

| 参数 | 说明 |
|---|---|
| `--use_gripper=true/false` | `true` 会给夹爪发送 MIT hold，并使用 `end_link` 惯量缩放 `0.7`；`false` 不控制夹爪，并从重力模型中移除 `end_link` 惯量 |
| `--urdf <path>` | 可选 URDF 路径，默认使用包内 B601 fixed-end URDF |

### 11. 带速度锁的重力补偿

`10_gravity_compensation_lock` 在重力补偿上加速度锁。当测得关节速度低于
`--vel-threshold` 时，锁定的关节目标保持不变；推动机械臂超过阈值后更新锁定姿态。

```bash
./example/cpp/build/10_gravity_compensation_lock \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true
```

重要参数：

| 参数 | 说明 |
|---|---|
| `--vel-threshold` | 关节速度阈值，单位 rad/s |
| `--lock-kp`, `--lock-kd` | MIT 锁定刚度和阻尼 |
| `--use_gripper=true/false` | 是否包含固定 `end_link` 夹爪负载，默认 `true` |

## 夹爪控制台

`gripper_test` 是交互式夹爪终端，默认使用 FORCE_POS。

```bash
./example/cpp/build/gripper_test --port /dev/ttyACM0
```

交互命令：

| 命令 | 作用 |
|---|---|
| `open` | 命令夹爪到 0 deg |
| `close` | 命令夹爪到 -270 deg |
| `pos <deg>` | FORCE_POS 目标，使用默认速度和力矩比例 |
| `forcepos <deg> [vlim] [ratio]` | FORCE_POS 目标 |
| `posvel <deg> [vlim]` | POS_VEL 目标 |
| `state` | 打印机械臂当前状态 |
| `q` | 退出 |

## 参数

所有使用数学后端的示例都支持指定 URDF：

```bash
./example/cpp/build/5_fk_test --urdf /path/to/reBot-DevArm_fixend.urdf
```

如果在不同源码路径下运行，设置：

```bash
export REBOTARM_CONTROL_RT_ROOT=/path/to/rebotarm_control_rt
```
