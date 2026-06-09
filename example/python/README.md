# Python Examples

> 中文文档见 [README.zh.md](README.zh.md)

These examples exercise the Python API of `rebotarm_control_rt`. Run them from the repository
root after activating the Python environment:

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
conda activate rebot
```

When run from this repository, the scripts add the local `python/` source tree to `sys.path`
automatically, so they can be used before installing a wheel.

## Hardware Setup

All hardware examples accept `--port` to override the YAML `channel` at runtime, and
`--config/-c` to load a different arm or gripper YAML.

For the Damiao serial bridge, first check the actual USB device name and pass it with `--port`:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/

sudo chmod 666 /dev/ttyACM*

# Single arm example:
#   --port /dev/ttyACM0
#
# Dual arms usually use different ports, for example:
#   left:  --port /dev/ttyACM0
#   right: --port /dev/ttyACM1
```

If you want the packaged YAML defaults to work without `--port`, update `python/rebotarm_control_rt/config/arm.yaml` and `gripper.yaml` manually: `channel: /dev/ttyACM0`.

For SocketCAN:

```bash
sudo ip link set can0 up type can bitrate 500000
```

## Debug Tools

### 1. Single Motor Console

`0x01damiao_test.py` is an interactive single-joint terminal. It creates a `RobotArm`, controls
one selected joint, and keeps the other joints at their current positions. `1_damiao_text.py` is
kept as a filename-compatible entry point for workflows copied from `reBotArm_control_py`.

```bash
python example/python/0x01damiao_test.py --port /dev/ttyACM0 --joint 0
python example/python/1_damiao_text.py --port /dev/ttyACM0 --joint joint1
```

Interactive commands:

| Command | Description |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | MIT target for the selected joint |
| `posvel <pos_deg> [vlim]` | POS_VEL target for the selected joint |
| `vel <vel_rad_s>` | Velocity command for the selected joint |
| `mode <mit|posvel|vel>` | Switch control mode |
| `enable` / `disable` | Enable or disable the arm |
| `set_zero` | Set zero for the selected joint |
| `state` | Print selected joint position, velocity, and torque |
| `q` | Stop and disconnect |

### 2. Zero Calibration and State Monitor

`2_zero_and_read.py` prints live joint positions. If `--skip-zero` is omitted, it asks for
confirmation and then sets the current arm pose as zero.

```bash
python example/python/2_zero_and_read.py --port /dev/ttyACM0 --skip-zero
python example/python/2_zero_and_read.py --port /dev/ttyACM0
```

### 3. Damiao POS_VEL Gain Register Reader

`0x02_read_damiao_pd.py` reads the current Damiao POS_VEL gain registers directly through the SDK.
`--default-bi` creates temporary B601 configs from the supplied ports, so it does not depend on
LeRobot-generated cache YAML files.

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
python example/python/0x02_read_damiao_pd.py --default-bi --timeout-ms 300

# Dual B601 arms with explicit ports
python example/python/0x02_read_damiao_pd.py \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# Read one or more arbitrary ports
python example/python/0x02_read_damiao_pd.py --port /dev/ttyACM0
python example/python/0x02_read_damiao_pd.py --port /dev/ttyACM0 --port /dev/ttyACM1

# Read from an explicit arm YAML
python example/python/0x02_read_damiao_pd.py --config python/rebotarm_control_rt/config/arm.yaml
```

## Joint Control

### 4. RT-native MIT Control

`3_mit_control.py` starts the Rust RT loop in MIT mode. Python only updates the target buffers with
`set_targets`; the Rust thread sends motor frames at the configured rate.

```bash
python example/python/3_mit_control.py --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [kp kd]     # joint angles in degrees
state                         # print current state and RT overruns
q                             # quit
```

### 5. RT-native POS_VEL Control

`4_pos_vel_control.py` starts the Rust RT loop in POS_VEL mode. The optional trailing value
overrides `vlim` for all joints in that command.

```bash
python example/python/4_pos_vel_control.py --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [vlim]      # joint angles in degrees, vlim in rad/s
state                         # print current state and RT overruns
q                             # quit
```

## Kinematics Tests

### 6. Forward Kinematics

`5_fk_test.py` computes the end-effector pose from six joint angles. It does not connect to
hardware.

```bash
python example/python/5_fk_test.py
```

Example input:

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

Output:

- End-effector position `(x, y, z)` in meters
- Rotation matrix
- Roll / pitch / yaw in degrees

### 7. Inverse Kinematics

`6_ik_test.py` solves joint angles from a target end-effector pose. It does not connect to
hardware.

```bash
python example/python/6_ik_test.py
```

Input format:

```text
x y z                         # meters, keeps neutral FK orientation
x y z roll pitch yaw          # meters + degrees
```

Example input:

```text
0.2603 0.0 0.1917
0.2603 0.0 0.1917 0 0 0
```

## Real Machine End-effector Control

### 8. End-effector IK Control

`7_arm_ik_control.py` uses `ArmEndPos`: C++ solves IK and the Rust RT loop executes the resulting
joint targets.

```bash
python example/python/7_arm_ik_control.py --port /dev/ttyACM0
```

Example input:

```text
0.3 0.0 0.2
0.3 0.1 0.25 0 0.5 0
```

Interactive commands:

| Command | Description |
|---|---|
| `x y z [roll pitch yaw]` | Target end-effector pose, radians for orientation |
| `state` | Print current joint state and RT overruns |
| `pos` / `end_state` | Print current end-effector pose |
| `q` / `quit` / `exit` | Quit |

### 9. End-effector Trajectory Control

`8_arm_traj_control.py` uses `ArmEndPos` trajectory mode. C++ plans and tracks the Cartesian
trajectory, then the Rust RT loop executes the streamed joint targets.

```bash
python example/python/8_arm_traj_control.py --port /dev/ttyACM0
```

Input format:

```text
x y z [roll pitch yaw] [duration]
```

Example input:

```text
0.3 0.0 0.3 0 0.4 0 2.0
```

`7_arm_ik_control.py` and `8_arm_traj_control.py` call `ArmEndPos.end()` on exit, which runs
`safe_home()` before disconnecting.

## Gravity Compensation

### 10. Basic Gravity Compensation

`9_gravity_compensation.py` computes gravity feedforward torque with the C++ dynamics model and
sends MIT commands from a Python callback loop.

```bash
python example/python/9_gravity_compensation.py --port /dev/ttyACM0 --rate 200
```

Control law:

```text
tau = g(q)
pos = current joint position
kp = 0, kd = 1 in the loop command
```

By default, `use_gripper=true`: the fixed `end_link` load in the URDF is included in the gravity
model with the calibrated load scale used for the tested B601 gripper setup. If the arm is running
without the gripper or equivalent end load, disable it explicitly:

```bash
python example/python/9_gravity_compensation.py --port /dev/ttyACM0 --rate 200 --use_gripper=false
```

Press `Ctrl+C` to stop and disconnect.

### 11. Gravity Compensation with Velocity Lock

`10_gravity_compensation_lock.py` adds an end-effector velocity lock on top of gravity
compensation. When the TCP velocity is below thresholds, the locked joint target stays fixed;
pushing the arm fast enough updates the locked pose.

```bash
python example/python/10_gravity_compensation_lock.py --port /dev/ttyACM0 --rate 200
```

Important options:

| Option | Description |
|---|---|
| `--vel-threshold` | Linear TCP velocity threshold |
| `--w-threshold` | Angular TCP velocity threshold |
| `--kp`, `--kd` | MIT lock stiffness and damping |
| `--integral-limit` | Integral torque clamp |
| `--use_gripper=true/false` | Include or exclude the fixed `end_link` gripper load; default is `true` |

## Gripper Console

`gripper_test.py` is an interactive gripper terminal for zeroing, mode switching, and
MIT/POS_VEL/VEL commands.

```bash
python example/python/gripper_test.py --port /dev/ttyACM0
```

Interactive commands:

| Command | Description |
|---|---|
| `z` | Set current gripper position as zero |
| `m` | Switch MIT / POS_VEL / VEL mode |
| `c` | Send or update the control command |
| `s` | Print gripper position, velocity, and torque |
| `q` | Stop loop, disable, and disconnect |

## MeshCat Simulation

Optional simulation examples live under `example/python/sim/`. They require Python `meshcat` and
Python `pinocchio` for visualization only; kinematics and trajectory calculations still use this
package's C++ bindings.

```bash
pip install meshcat
conda install -c conda-forge "pinocchio>=3.2,<4"
```

If your shell has sourced ROS, clear both ROS Python and library paths when running the simulation
examples:

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/fk_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/ik_sim.py
env -u PYTHONPATH -u LD_LIBRARY_PATH python example/python/sim/traj_sim.py
```
