# rebotarm_control_rt

> 中文文档见 [README.zh.md](README.zh.md)

Control library for the **reBot Arm B601**. Every performance-critical layer is implemented
natively; **Python is only a thin API layer** on top.

| Layer | Language | What it does |
|---|---|---|
| `_native` (actuator) | **Rust / PyO3** | Motor control. Depends directly on the Cargo `path` vendor crates of [motorbridge](../motorbridge) — no `ctypes` / C-ABI hop. |
| `_math` (kinematics / dynamics / trajectory / controllers) | **C++ / pybind11** | Links **Pinocchio C++** directly (not the Python bindings). No Python `pinocchio` dependency. |

The public API matches `reBotArm_control_py`, so this package is a drop-in replacement.

---

## Project layout

```
rebotarm_control_rt/
├── src/                                  # Rust actuator kernel → _native.so
│   ├── lib.rs
│   ├── config.rs
│   ├── vendor.rs
│   ├── arm.rs
│   └── gripper.rs
├── cpp/                                  # C++ math layer (Pinocchio) → _math.so
│   ├── CMakeLists.txt
│   ├── include/rebotarm/                 #   robot_model, dynamics, trajectory, arm_endpos, se3_conv (.hpp)
│   └── src/                              #   bindings, robot_model, dynamics, trajectory, arm_endpos (.cpp)
├── python/rebotarm_control_rt/
│   ├── actuator/                         #   re-exports _native (RobotArm / Gripper / ...)
│   ├── kinematics/                       #   re-exports _math (pure bindings)
│   ├── dynamics/
│   ├── trajectory/
│   ├── controllers/
│   ├── config/                           #   installed with the package
│   └── urdf/
├── example/
├── tests/
└── build.sh
```

---

## Build & install

One-command setup on a fresh machine (two stages; see [INSTALL.md](INSTALL.md) for details):

```bash
# Stage 1: create env + Pinocchio C++ 3.x  (--conda also works; optional python version)
bash ./setup_env.sh --mamba rebot 3.10

# Stage 2: activate, then install (rust/maturin → build.sh --wheel → self-check)
mamba activate rebot          # or: conda activate rebot
bash ./setup_env.sh --install
```

### How the build works

`build.sh` first compiles `_math.so` with CMake (linking Pinocchio C++) into the package
directory, then uses maturin to pack `_native` (Rust) + `_math.so` + Python into a single wheel
and install it.

- **Automatic Pinocchio C++ prefix detection**:
  `-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `/usr/local` > `/usr`.
  Adapts to both `lib` and `lib/x86_64-linux-gnu`, and locates Eigen automatically. **No ROS required.**
- The runtime RPATH is baked with the Pinocchio library directory, so no `LD_LIBRARY_PATH` is needed.
- Example scripts add the local `python/` source tree to `sys.path` automatically when run from
  this repository, so they can be tested before wheel installation.

> ⚠️ Use **Pinocchio 3.x** from conda-forge (`pinocchio>=3.2,<4`). 4.0 reorganized the header
> layout; the current code targets 3.x.

### Real-time note

The RT loop is **soft real-time in Rust** (`std::thread` + absolute tick cadence + overrun
counter + best-effort `SCHED_FIFO`). It releases the Python GIL entirely and is far more stable
than a Python thread — but it is **not** a hard real-time stack like the Flexiv RDK. To get close
to hard real-time, run as root on a PREEMPT_RT kernel with `rt_priority`/`cpu` set, and monitor
`arm.rt_send_overruns` / `arm.rt_read_overruns`.

## Example Programs

Run examples from the repository root after activating the Python environment:

```bash
cd rebotarm_control_rt
conda activate rebot
```

All hardware examples accept `--config/-c` to load another arm or gripper YAML. When running from
this repository, examples add the local `python/` source tree to `sys.path` automatically.

### Debug Tools

#### 1. Single Motor Console (`0x01damiao_test.py`, `1_damiao_text.py`)

Interactive single-joint terminal. It uses `RobotArm`, controls one selected joint, and keeps the
other joints at their current positions. `1_damiao_text.py` is kept for filename compatibility with
`reBotArm_control_py`.

**Usage**:

```bash
python example/0x01damiao_test.py --joint 0
python example/1_damiao_text.py --joint joint1
```

**Interactive commands**:

| Command | Description |
|---|---|
| `mit <pos_deg> [vel kp kd tau]` | MIT target for the selected joint |
| `posvel <pos_deg> [vlim]` | POS_VEL target for the selected joint |
| `vel <vel_rad_s>` | Velocity command for the selected joint |
| `mode <mit|posvel|vel>` | Switch control mode |
| `enable` / `disable` | Enable or disable the arm |
| `set_zero` | Set zero for the selected joint |
| `state` | Print selected joint position, velocity, and torque |

---

#### 2. Zero Calibration & State Monitor (`2_zero_and_read.py`)

Print live joint positions. If `--skip-zero` is omitted, the script asks for confirmation and then
sets the current arm pose as zero.

**Usage**:

```bash
python example/2_zero_and_read.py --skip-zero
python example/2_zero_and_read.py
```

---

### Joint Control Examples

#### 3. RT-native MIT Control (`3_mit_control.py`)

Starts the Rust RT loop in MIT mode. Python only updates target buffers with `set_targets`; the
Rust thread sends motor frames at the configured rate.

**Usage**:

```bash
python example/3_mit_control.py --rate 150
```

**Input format**:

```text
q1 q2 q3 q4 q5 q6 [kp kd]     # joint angles in degrees
state                         # print current state and RT overruns
q                             # quit
```

---

#### 4. RT-native POS_VEL Control (`4_pos_vel_control.py`)

Starts the Rust RT loop in POS_VEL mode. The optional trailing value overrides `vlim` for all
joints in that command.

**Usage**:

```bash
python example/4_pos_vel_control.py --rate 150
```

**Input format**:

```text
q1 q2 q3 q4 q5 q6 [vlim]      # joint angles in degrees, vlim in rad/s
state                         # print current state and RT overruns
q                             # quit
```

---

### Kinematics Tests

#### 5. Forward Kinematics Test (`5_fk_test.py`)

Computes end-effector pose from six joint angles. This does not connect to hardware.

**Usage**:

```bash
python example/5_fk_test.py
> 0 0 0 0 0 0
> 45 -30 15 -60 90 180
```

**Output**:

- End-effector position `(x, y, z)` in meters
- Rotation matrix
- Roll / pitch / yaw in degrees

---

#### 6. Inverse Kinematics Test (`6_ik_test.py`)

Solves joint angles from a target end-effector pose. This does not connect to hardware.

**Input format**:

```text
x y z                         # meters, keeps neutral FK orientation
x y z roll pitch yaw          # meters + degrees
```

**Usage**:

```bash
python example/6_ik_test.py
> 0.2603 0.0 0.1917
> 0.2603 0.0 0.1917 0 0 0
```

---

### Real Machine Control

Before running hardware examples, check device permissions:

```bash
# Damiao serial bridge
sudo chmod 666 /dev/ttyACM0

# SocketCAN, if used
sudo ip link set can0 up type can bitrate 500000
```

#### 7. End-effector IK Control (`7_arm_ik_control.py`)

Uses `ArmEndPos`: C++ solves IK and the Rust RT loop executes the resulting joint targets.

**Usage**:

```bash
python example/7_arm_ik_control.py
> 0.3 0.0 0.2
> 0.3 0.1 0.25 0 0.5 0
```

**Interactive commands**:

| Command | Description |
|---|---|
| `x y z [roll pitch yaw]` | Target end-effector pose, radians for orientation |
| `state` | Print current joint state and RT overruns |
| `pos` / `end_state` | Print current end-effector pose |
| `q` / `quit` / `exit` | Quit |

---

#### 8. End-effector Trajectory Control (`8_arm_traj_control.py`)

Uses `ArmEndPos` trajectory mode. C++ plans and tracks the Cartesian trajectory, then the Rust RT
loop executes the streamed joint targets.

**Input format**:

```text
x y z [roll pitch yaw] [duration]
```

**Usage**:

```bash
python example/8_arm_traj_control.py
> 0.3 0.0 0.3 0 0.4 0 2.0
```

`7_arm_ik_control.py` and `8_arm_traj_control.py` call `ArmEndPos.end()` on exit, which runs
`safe_home()` before disconnecting.

---

#### 9. Gravity Compensation (`9_gravity_compensation.py`)

Computes gravity feedforward torque with the C++ dynamics model and sends MIT commands from a
Python callback loop.

**Control law**:

```text
tau = g(q)
pos = current joint position
kp = 0, kd = 1 in the loop command
```

**Usage**:

```bash
python example/9_gravity_compensation.py --rate 200
```

Press `Ctrl+C` to stop and disconnect.

---

#### 10. Gravity Compensation with Velocity Lock (`10_gravity_compensation_lock.py`)

Adds an end-effector velocity lock on top of gravity compensation. When the TCP velocity is below
thresholds, the locked joint target stays fixed; pushing the arm fast enough updates the locked pose.

**Usage**:

```bash
python example/10_gravity_compensation_lock.py --rate 200
```

Important options:

| Option | Description |
|---|---|
| `--vel-threshold` | Linear TCP velocity threshold |
| `--w-threshold` | Angular TCP velocity threshold |
| `--kp`, `--kd` | MIT lock stiffness and damping |
| `--integral-limit` | Integral torque clamp |

---

#### 11. Gripper Console (`gripper_test.py`)

Interactive gripper terminal for zeroing, mode switching, and MIT/POS_VEL/VEL commands.

**Usage**:

```bash
python example/gripper_test.py
```

**Interactive commands**:

| Command | Description |
|---|---|
| `z` | Set current gripper position as zero |
| `m` | Switch MIT / POS_VEL / VEL mode |
| `c` | Send or update the control command |
| `s` | Print gripper position, velocity, and torque |
| `q` | Stop loop, disable, and disconnect |

---

### MeshCat Simulation

The optional simulation examples live under `example/sim/`. They require Python `meshcat` and
Python `pinocchio` for visualization only; the kinematics and trajectory calculations still use
this package's C++ bindings.

```bash
python example/sim/fk_sim.py
python example/sim/ik_sim.py
python example/sim/traj_sim.py
```

---

## Usage

```python
import numpy as np
from rebotarm_control_rt.actuator import RobotArm
from rebotarm_control_rt.kinematics import RobotModel, compute_ik
from rebotarm_control_rt.dynamics import compute_generalized_gravity, load_dynamics_model
from rebotarm_control_rt.controllers import ArmEndPos

arm = RobotArm()                      # defaults to the packaged config/arm.yaml
arm.connect(); arm.enable(); arm.mode_mit()
```

### Control loops

```python
# Compatibility mode: Python callback control loop (same as reBotArm_control_py)
arm.start_control_loop(lambda a, dt: a.mit(np.zeros(arm.num_joints)))

# Native RT mode: the control loop runs on a Rust thread with the GIL released throughout.
#   - If set_targets has not been called yet, it reads the current joint positions as the
#     hold target (it will NOT pull the arm toward the zero pose).
#   - request_feedback defaults to False; normal operation uses motorbridge's cached feedback.
#   - command_gap_us inserts a small delay between per-joint command frames if the bus needs it.
#   - rt_priority > 0: best-effort SCHED_FIFO (needs root / CAP_SYS_NICE; PREEMPT_RT kernel here).
#   - cpu: optional CPU affinity.
arm.mode_pos_vel()
arm.start_rt_loop(rate=150.0, rt_priority=0, cpu=None, command_gap_us=0)
arm.set_targets(pos=np.zeros(arm.num_joints))   # update the target at any time afterward
print("send/read overruns:", arm.rt_send_overruns, arm.rt_read_overruns)

arm.stop_control_loop(); arm.disconnect()
```

If you need active feedback requests in addition to motorbridge's background polling, enable the
optional feedback thread explicitly:

```python
arm.start_rt_loop(rate=150.0, request_feedback=True, feedback_rate=60.0)
```

### End-effector orchestration

```python
# IK + trajectory: computed in C++, driving the Rust arm.
with ArmEndPos(arm) as ep:
    ep.move_to_ik(x=0.3, y=0.0, z=0.3)
    ep.move_to_traj(x=0.3, y=0.0, z=0.3, pitch=0.4, duration=2.0)
```

Poses are unified at the Python boundary as **4×4 numpy homogeneous matrices**
(`pinocchio.SE3` is not exposed).

---

## Vendor support (actuator)

- **Damiao** — primary path, serial bridge over `/dev/tty*` at 921600.
- **MyActuator / RobStride / HighTorque** — CAN.

State normalization, mode mapping, and control-command dispatch all match the C-ABI
implementation in `motor_abi`.

## Acknowledgments

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm.git): provides the B601 arm model assets used by this package's built-in URDF, meshes, kinematics, dynamics, and simulation examples.
- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py.git): provides the Python API shape and example workflows that this package keeps compatible with while replacing the actuator and math backends with native RT implementations.
- [motorbridge](https://github.com/motorbridge/motorbridge.git): provides the native Rust motor vendor crates used directly by the actuator layer for Damiao, MyActuator, RobStride, and HighTorque communication and command dispatch.
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git): provides the C++ kinematics and dynamics backend linked by `_math`; this package uses the C++ library directly rather than the Python binding.
