# C++ Examples

> 中文文档见 [README.zh.md](README.zh.md)

These examples are the C++ equivalents of the Python and Rust example sets. They do not import
Python. Motor communication uses the `motorbridge` C++ binding plus `motor_abi`, while FK, IK,
trajectory, and gravity compensation use the native C++/Pinocchio backend in
`librebotarm_math.so`.

Run all commands from the repository root:

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
conda activate rebot
```

## Build

Build the math backend first:

```bash
PY=/home/xense/miniforge3/envs/rebot/bin/python ./build.sh
```

Build `motor_abi`, which is required by all hardware examples:

```bash
cargo build --manifest-path ../motorbridge/motor_abi/Cargo.toml --release
```

Then build the C++ examples:

```bash
cmake -S example/cpp -B example/cpp/build \
  -DPINOCCHIO_PREFIX=/home/xense/miniforge3/envs/rebot
cmake --build example/cpp/build -j"$(nproc)"
```

If the conda environment is active and `CONDA_PREFIX` points to the same environment, the explicit
`-DPINOCCHIO_PREFIX=...` argument is optional.

## Hardware Setup

For the Damiao serial bridge, check the actual USB device name first:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
sudo chmod 666 /dev/ttyACM*
```

Pass the selected port to each hardware example:

```bash
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0 --skip-zero
```

SocketCAN can also be used by passing a CAN channel:

```bash
sudo ip link set can0 up type can bitrate 500000
./example/cpp/build/2_zero_and_read --port can0 --skip-zero
```

## Debug Tools

### 1. Single Motor Console

`0x01damiao_test` is an interactive single-joint terminal. It opens one B601 arm, selects one
joint, and lets you send MIT, POS_VEL, velocity, and FORCE_POS commands. `1_damiao_text` is kept
as a filename-compatible entry point.

```bash
./example/cpp/build/0x01damiao_test --port /dev/ttyACM0 --joint 0
./example/cpp/build/1_damiao_text --port /dev/ttyACM0 --joint joint1
```

Interactive commands:

| Command | Description |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | MIT target for the selected joint |
| `posvel <pos_deg> [vlim]` | POS_VEL target for the selected joint |
| `vel <vel_rad_s>` | Velocity command for the selected joint |
| `forcepos <pos_deg> [vlim ratio]` | FORCE_POS command, useful for gripper force-limited position |
| `mode <mit|posvel|vel|forcepos>` | Switch selected joint control mode |
| `enable` / `disable` | Enable or disable the arm |
| `set_zero` | Set zero for the selected joint after confirmation |
| `state` | Print joint position, velocity, torque, and status |
| `q` | Stop and disconnect |

### 2. Zero Calibration and State Monitor

`2_zero_and_read` prints live joint positions. If `--skip-zero` is omitted, it asks for
confirmation and then sets the current pose as zero for all B601 motors.

```bash
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0 --skip-zero
./example/cpp/build/2_zero_and_read --port /dev/ttyACM0
```

### 3. Damiao POS_VEL Gain Register Reader

`0x02_read_damiao_pd` reads current Damiao POS_VEL gain registers directly from the motors. It
does not enable motors and does not send motion commands.

Damiao registers:

| Register | Name |
|---|---|
| `25` | `vel_kp` / `KP_ASR` |
| `26` | `vel_ki` / `KI_ASR` |
| `27` | `pos_kp` / `KP_APR` |
| `28` | `pos_ki` / `KI_APR` |

Usage:

```bash
# Dual B601 arms, defaults to /dev/ttyACM0 and /dev/ttyACM1
./example/cpp/build/0x02_read_damiao_pd --default-bi --timeout-ms 300

# Dual B601 arms with explicit ports
./example/cpp/build/0x02_read_damiao_pd \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# Read one or more arbitrary ports
./example/cpp/build/0x02_read_damiao_pd --port /dev/ttyACM0
./example/cpp/build/0x02_read_damiao_pd --port /dev/ttyACM0 --port /dev/ttyACM1
```

## Joint Control

### 4. MIT Control

`3_mit_control` starts a C++ thread that sends MIT commands at the configured rate. The terminal
only updates the target buffer.

```bash
./example/cpp/build/3_mit_control --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [gripper] [kp kd]     # angles in degrees
state                                    # print current state
q                                        # quit
```

### 5. POS_VEL Control

`4_pos_vel_control` starts a C++ thread that sends POS_VEL commands at the configured rate.

```bash
./example/cpp/build/4_pos_vel_control --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [gripper] [vlim]       # angles in degrees, vlim in rad/s
state                                    # print current state
q                                        # quit
```

## Kinematics Tests

### 6. Forward Kinematics

`5_fk_test` computes the end-effector pose from six joint angles. It does not connect to hardware.

```bash
./example/cpp/build/5_fk_test
```

Example input:

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

### 7. Inverse Kinematics

`6_ik_test` solves an IK target. `x y z` keeps the neutral orientation; `x y z roll pitch yaw`
uses radians for orientation. It does not connect to hardware.

```bash
./example/cpp/build/6_ik_test
```

Example input:

```text
0.2603 0.0 0.1917
0.2603 0.0 0.1917 0 0 0
```

## Real Machine End-effector Control

### 8. End-effector IK Control

`7_arm_ik_control` reads the current joint pose, solves a position target with C++/Pinocchio IK,
and streams a joint-space POS_VEL move.

```bash
./example/cpp/build/7_arm_ik_control --port /dev/ttyACM0 --rate 150
```

Input format:

```text
x y z                         # meters
state
q
```

### 9. End-effector Trajectory Control

`8_arm_traj_control` solves a position target with C++/Pinocchio IK and interpolates the joint
move over the requested duration.

```bash
./example/cpp/build/8_arm_traj_control --port /dev/ttyACM0 --rate 150
```

Input format:

```text
x y z [duration]              # meters, seconds
state
q
```

## Gravity Compensation

### 10. Basic Gravity Compensation

`9_gravity_compensation` computes generalized gravity with the C++/Pinocchio dynamics backend and
sends MIT torque feed-forward commands.

```bash
./example/cpp/build/9_gravity_compensation \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true \
  --kp 0.0 \
  --kd 1.0
```

Important options:

| Option | Description |
|---|---|
| `--kp`, `--kd` | MIT command stiffness and damping for arm joints; defaults are `0.0` and `1.0` for compliant gravity compensation |
| `--gripper-kp`, `--gripper-kd` | MIT command stiffness and damping for the extra gripper joint; defaults are `0.0` and `1.0` |
| `--use_gripper=true/false` | `true` sends gripper MIT hold and uses `end_link` inertial scale `0.7`; `false` skips gripper motor command and removes `end_link` inertial from the gravity model |
| `--urdf <path>` | Optional URDF path; defaults to the packaged B601 fixed-end URDF |

### 11. Gravity Compensation with Velocity Lock

`10_gravity_compensation_lock` adds a velocity-based lock. When the measured joint velocity is
below `--vel-threshold`, the locked joint target stays fixed; moving the arm fast enough updates
the locked pose.

```bash
./example/cpp/build/10_gravity_compensation_lock \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true \
  --lock-kp 8.0 \
  --lock-kd 1.0
```

Important options:

| Option | Description |
|---|---|
| `--vel-threshold` | Joint velocity threshold in rad/s |
| `--lock-kp`, `--lock-kd` | MIT lock stiffness and damping; defaults are `8.0` and `1.0`. `--kp` / `--kd` are accepted as aliases |
| `--use_gripper=true/false` | Include or exclude the fixed `end_link` gripper load; default is `true` |

## Gripper Console

`gripper_test` is an interactive gripper terminal using FORCE_POS by default.

```bash
./example/cpp/build/gripper_test --port /dev/ttyACM0
```

Interactive commands:

| Command | Description |
|---|---|
| `open` | Command gripper to 0 deg |
| `close` | Command gripper to -270 deg |
| `pos <deg>` | FORCE_POS target with default speed and torque ratio |
| `forcepos <deg> [vlim] [ratio]` | FORCE_POS target |
| `posvel <deg> [vlim]` | POS_VEL target |
| `state` | Print current arm state |
| `q` | Quit |

## Options

All math-backed examples accept an optional URDF path:

```bash
./example/cpp/build/5_fk_test --urdf /path/to/reBot-DevArm_fixend.urdf
```

If running from a different checkout path, set:

```bash
export REBOTARM_CONTROL_RT_ROOT=/path/to/rebotarm_control_rt
```
