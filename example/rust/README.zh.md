# Rust 示例

> For the English documentation, see [README.md](README.md)

这些示例的电机通信直接使用 Rust Damiao vendor crate，不导入 Python，也不经过
`rebotarm_control_rt` 的 PyO3 层。FK、IK 和重力补偿示例会通过 `librebotarm_math.so`
调用同一套 C++/Pinocchio 数学后端。

从仓库根目录运行：

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
```

或者进入当前目录运行：

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt/example/rust
```

## 目录结构

```text
example/rust/
├── Cargo.toml
├── README.md
├── README.zh.md
└── src/
    ├── common.rs
    └── bin/
        ├── 0x01damiao_test.rs
        ├── 0x02_read_damiao_pd.rs
        ├── 1_damiao_text.rs
        ├── 2_zero_and_read.rs
        ├── 3_mit_control.rs
        ├── 4_pos_vel_control.rs
        ├── 5_fk_test.rs
        ├── 6_ik_test.rs
        ├── 7_arm_ik_control.rs
        ├── 8_arm_traj_control.rs
        ├── 9_gravity_compensation.rs
        ├── 10_gravity_compensation_lock.rs
        ├── gripper_test.rs
        └── read_damiao_pd.rs
```

`src/common.rs` 放了 B601 电机映射、默认增益、硬件辅助函数，以及 C++/Pinocchio 数学后端的 C ABI 加载逻辑。

## 构建数学后端

运行 FK、IK 或重力补偿示例前，先构建一次 C++ 后端：

```bash
PY=/home/xense/miniforge3/envs/rebot/bin/python ./build.sh
```

构建后会生成 `python/rebotarm_control_rt/librebotarm_math.so`。如果在源码树外运行 Rust 示例，
用 `REBOTARM_MATH_LIB` 指向这个 shared library。

## 硬件准备

达妙串口桥运行前先确认实际 USB 设备名：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
sudo chmod 666 /dev/ttyACM*
```

每个真机示例都用 `--port` 指定端口：

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0 --skip-zero
```

如果使用 SocketCAN：

```bash
sudo ip link set can0 up type can bitrate 500000
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port can0 --skip-zero
```

## 运行方式

从仓库根目录运行：

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 5_fk_test
cargo run --manifest-path example/rust/Cargo.toml --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
```

从 `example/rust` 目录运行：

```bash
cargo run --bin 5_fk_test
cargo run --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
```

## 调试工具

### 1. 单电机控制台

`0x01damiao_test.rs` 是交互式单关节终端。它打开一台 B601，选择一个关节，然后可以发送
MIT、POS_VEL、速度和 FORCE_POS 指令。`1_damiao_text.rs` 保留为同名兼容入口。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
cargo run --manifest-path example/rust/Cargo.toml --bin 1_damiao_text -- --port /dev/ttyACM0 --joint joint1
```

交互命令：

| 命令 | 说明 |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | 给选中关节发送 MIT 目标 |
| `posvel <pos_deg> [vlim]` | 给选中关节发送 POS_VEL 目标 |
| `vel <vel_rad_s>` | 给选中关节发送速度指令 |
| `forcepos <pos_deg> [vlim ratio]` | FORCE_POS 指令，适合夹爪限力位置控制 |
| `mode <mit|posvel|vel|forcepos>` | 切换选中关节控制模式 |
| `enable` / `disable` | 使能或失能机械臂 |
| `set_zero` | 确认后给选中关节设置零点 |
| `state` | 打印位置、速度、力矩和状态 |
| `q` | 停止并断开 |

### 2. 零点校准与状态监控

`2_zero_and_read.rs` 打印实时关节位置。若不加 `--skip-zero`，会先要求确认，然后把当前姿态设为所有 B601 电机零点。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0 --skip-zero
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0
```

### 3. 达妙 POS_VEL 参数寄存器读取

`0x02_read_damiao_pd.rs` 直接读取电机当前 Damiao POS_VEL 增益寄存器。它不会使能电机，也不会发送运动指令。
`read_damiao_pd.rs` 保留为短名称别名。

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
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --default-bi --timeout-ms 300

# 双 B601，显式指定左右端口
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# 读取一个或多个指定端口
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --port /dev/ttyACM0
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --port /dev/ttyACM0 --port /dev/ttyACM1
```

## 关节控制

### 4. MIT 控制

`3_mit_control.rs` 启动一个 Rust 线程按设定频率发送 MIT 指令，终端只负责更新目标缓存。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 3_mit_control -- --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [gripper] [kp kd]     # 角度单位为度
state                                    # 打印当前状态
q                                        # 退出
```

### 5. POS_VEL 控制

`4_pos_vel_control.rs` 启动一个 Rust 线程按设定频率发送 POS_VEL 指令。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 4_pos_vel_control -- --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
q1 q2 q3 q4 q5 q6 [gripper] [vlim]       # 角度单位为度，vlim 单位为 rad/s
state                                    # 打印当前状态
q                                        # 退出
```

## 运动学测试

### 6. 正运动学

`5_fk_test.rs` 根据 6 个关节角计算末端位姿。不连接硬件。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 5_fk_test
```

示例输入：

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

### 7. 逆运动学

`6_ik_test.rs` 求解只包含位置的 IK 目标。不连接硬件。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 6_ik_test
```

输入格式：

```text
x y z                         # 米
```

这个示例通过 `librebotarm_math.so` 使用与 Python 示例相同的 C++/Pinocchio IK 后端。

## 真机末端控制

### 8. 末端 IK 控制

`7_arm_ik_control.rs` 读取当前关节角，用 C++/Pinocchio IK 后端求解目标位置，然后用 POS_VEL 发送关节空间移动。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 7_arm_ik_control -- --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
x y z                         # 米
state
q
```

### 9. 末端轨迹控制

`8_arm_traj_control.rs` 用 C++/Pinocchio IK 后端求目标关节角，并按指定时间做关节空间插值。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 8_arm_traj_control -- --port /dev/ttyACM0 --rate 150
```

输入格式：

```text
x y z [duration]              # 米，秒
state
q
```

Rust 轨迹示例目前是在 IK 后做关节空间插值。更完整的笛卡尔轨迹生成请使用 Python 示例或 LeRobot follower。

## 重力补偿

### 10. 基础重力补偿

`9_gravity_compensation.rs` 使用与 Python 示例相同的 C++/Pinocchio 动力学后端计算广义重力力矩，
然后用 MIT 指令发送力矩前馈。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 9_gravity_compensation -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true
```

重要参数：

| 参数 | 说明 |
|---|---|
| `--use_gripper=true/false` | `true` 给夹爪发送 MIT hold，并使用 `end_link` 惯量缩放 `0.7`；`false` 不给夹爪发指令，并从重力模型中移除 `end_link` 惯量 |
| `--urdf <path>` | 可选 URDF 路径；默认使用包内 B601 fixed-end URDF |

### 11. 带速度锁止的重力补偿

`10_gravity_compensation_lock.rs` 在同一个 C++/Pinocchio 重力力矩上增加关节速度锁止。
测得关节速度超过 `--vel-threshold` 时，锁止目标跟随当前姿态；否则保持上一次锁止姿态。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 10_gravity_compensation_lock -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true
```

重要参数：

| 参数 | 说明 |
|---|---|
| `--vel-threshold` | 关节速度阈值，单位 rad/s，默认 `0.04` |
| `--lock-kp`, `--lock-kd` | MIT 锁止刚度和阻尼 |
| `--use_gripper=true/false` | `true` 给夹爪发送 MIT hold，并使用 `end_link` 惯量缩放 `0.7`；`false` 不给夹爪发指令，并从重力模型中移除 `end_link` 惯量 |
| `--urdf <path>` | 可选 URDF 路径；默认使用包内 B601 fixed-end URDF |

## 夹爪控制台

`gripper_test.rs` 是交互式夹爪终端，用于 FORCE_POS 和 POS_VEL 指令。

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin gripper_test -- --port /dev/ttyACM0
```

交互命令：

| 命令 | 说明 |
|---|---|
| `open` | 用 FORCE_POS 打开到 `0 deg` |
| `close` | 用 FORCE_POS 闭合到 `-270 deg` |
| `pos <deg>` | FORCE_POS 目标，使用默认速度和力矩比例 |
| `forcepos <deg> [vlim ratio]` | FORCE_POS 目标 |
| `posvel <deg> [vlim]` | POS_VEL 目标 |
| `state` | 打印机械臂和夹爪状态 |
| `q` | 停止并断开 |

## 构建检查

如果依赖已经在本机缓存，可以用离线模式检查：

```bash
cargo check --manifest-path example/rust/Cargo.toml --offline
```

格式化 Rust 示例：

```bash
cargo fmt --manifest-path example/rust/Cargo.toml
```

## 注意事项

- `/dev/ttyACM*` 和 `/dev/ttyUSB*` 在 USB 重新插拔后可能变化，运行真机示例前先检查设备列表。
- 示例当前把 `/dev/*` 目标按达妙串口桥处理，波特率为 `921600`；非 `/dev` 目标按 SocketCAN channel 名处理。
- FK、IK 和重力补偿示例需要 `librebotarm_math.so`；先运行 `./build.sh`，或手动设置 `REBOTARM_MATH_LIB`。
