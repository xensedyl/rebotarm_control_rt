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
│   ├── python/                          # Python examples and MeshCat simulations
│   ├── rust/                            # Rust examples using motorbridge + C++ math C ABI
│   └── cpp/                             # C++ examples using motorbridge C++ + C++ math backend
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

`build.sh` first compiles `librebotarm_math.so` and `_math.so` with CMake (linking Pinocchio C++)
into the package directory, then uses maturin to pack `_native` (Rust) + `_math.so` + Python into a
single wheel and install it. The Rust examples load `librebotarm_math.so` directly through a C ABI
when they need FK, IK, or gravity compensation. The C++ examples link the same
`librebotarm_math.so` directly and use `motorbridge`'s C++ binding for hardware examples.

- **Automatic Pinocchio C++ prefix detection**:
  `-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `/usr/local` > `/usr`.
  Adapts to both `lib` and `lib/x86_64-linux-gnu`, and locates Eigen automatically. **No ROS required.**
- The runtime RPATH is baked with the Pinocchio library directory, so no `LD_LIBRARY_PATH` is needed.
- Example scripts add the local `python/` source tree to `sys.path` automatically when run from
  this repository, so they can be tested before wheel installation.

> ⚠️ Use **Pinocchio 3.x** from conda-forge (`pinocchio>=3.2,<4`). 4.0 reorganized the header
> layout; the current code targets 3.x.

### Build check

After `setup_env.sh --install` or `./build.sh --wheel`, activate the same environment used for the
install and verify that both native modules can be imported:

```bash
conda activate rebot
python -c "import rebotarm_control_rt._math, rebotarm_control_rt._native; print('ok')"
```

### Real-time note

The RT loop is **soft real-time in Rust** (`std::thread` + absolute tick cadence + overrun
counter + best-effort `SCHED_FIFO`). It releases the Python GIL entirely and is far more stable
than a Python thread — but it is **not** a hard real-time stack. To get close
to hard real-time, run as root on a PREEMPT_RT kernel with `rt_priority`/`cpu` set, and monitor
`arm.rt_send_overruns` / `arm.rt_read_overruns`.

## Example Programs

Examples are split by language and documented in their own directories:

- Python examples: [example/python/README.md](example/python/README.md)
- Rust examples: [example/rust/README.md](example/rust/README.md)
- C++ examples: [example/cpp/README.md](example/cpp/README.md)

Chinese example documentation is available at
[example/python/README.zh.md](example/python/README.zh.md) and
[example/rust/README.zh.md](example/rust/README.zh.md), and
[example/cpp/README.zh.md](example/cpp/README.zh.md).

Run examples from the repository root after activating the environment. The Python examples add the local `python/` source tree to `sys.path` automatically when run from this repository.

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
