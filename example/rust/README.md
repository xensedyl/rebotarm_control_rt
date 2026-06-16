# Rust Examples

> 中文文档见 [README.zh.md](README.zh.md)

These examples use the Rust Damiao vendor crate directly for motor communication. They do not
import Python and do not go through the `rebotarm_control_rt` PyO3 layer. FK, IK, and gravity
compensation examples load the same C++/Pinocchio backend through `librebotarm_math.so`.

Run from the repository root:

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt
```

Or from this directory:

```bash
cd /home/xense/rebot_lerobot/rebotarm_control_rt/example/rust
```

## Layout

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
        ├── 11_record_gravity_trajectory.rs
        ├── 12_collect_identification_data.rs
        ├── 13_identify_dynamics.rs
        ├── 14_verify_identification.rs
        ├── gripper_test.rs
        └── read_damiao_pd.rs
```

`src/common.rs` contains the B601 motor mapping, default gains, hardware helpers, and the C ABI
loader for the C++/Pinocchio math backend.

## Build Math Backend

Run the C++ build once before FK, IK, or gravity compensation examples:

```bash
PY=/home/xense/miniforge3/envs/rebot/bin/python ./build.sh
```

The build creates `python/rebotarm_control_rt/librebotarm_math.so`. If you run the Rust examples
outside this source tree, point `REBOTARM_MATH_LIB` to that shared library.

## Hardware Setup

For the Damiao serial bridge, first check the actual USB device name:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
sudo chmod 666 /dev/ttyACM*
```

Use `--port` in each hardware example:

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0 --skip-zero
```

If you use SocketCAN instead of the Damiao serial bridge:

```bash
sudo ip link set can0 up type can bitrate 500000
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port can0 --skip-zero
```

## Running Examples

From repository root:

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 5_fk_test
cargo run --manifest-path example/rust/Cargo.toml --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
```

From `example/rust`:

```bash
cargo run --bin 5_fk_test
cargo run --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
```

## Debug Tools

### 1. Single Motor Console

`0x01damiao_test.rs` is an interactive single-joint terminal. It opens one B601 arm, selects one
joint, and lets you send MIT, POS_VEL, velocity, and FORCE_POS commands. `1_damiao_text.rs` is kept
as a filename-compatible entry point.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0
cargo run --manifest-path example/rust/Cargo.toml --bin 1_damiao_text -- --port /dev/ttyACM0 --joint joint1
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

`2_zero_and_read.rs` prints live joint positions. If `--skip-zero` is omitted, it asks for
confirmation and then sets the current pose as zero for all B601 motors.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0 --skip-zero
cargo run --manifest-path example/rust/Cargo.toml --bin 2_zero_and_read -- --port /dev/ttyACM0
```

### 3. Damiao POS_VEL Gain Register Reader

`0x02_read_damiao_pd.rs` reads current Damiao POS_VEL gain registers directly from the motors. It
does not enable motors and does not send motion commands. `read_damiao_pd.rs` remains as a shorter
alias.

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
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --default-bi --timeout-ms 300

# Dual B601 arms with explicit ports
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- \
  --default-bi \
  --left-port /dev/ttyACM0 \
  --right-port /dev/ttyACM1

# Read one or more arbitrary ports
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --port /dev/ttyACM0
cargo run --manifest-path example/rust/Cargo.toml --bin 0x02_read_damiao_pd -- --port /dev/ttyACM0 --port /dev/ttyACM1
```

## Joint Control

### 4. MIT Control

`3_mit_control.rs` starts a Rust thread that sends MIT commands at the configured rate. The
terminal only updates the target buffer.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 3_mit_control -- --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [gripper] [kp kd]     # angles in degrees
state                                    # print current state
q                                        # quit
```

### 5. POS_VEL Control

`4_pos_vel_control.rs` starts a Rust thread that sends POS_VEL commands at the configured rate.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 4_pos_vel_control -- --port /dev/ttyACM0 --rate 150
```

Input format:

```text
q1 q2 q3 q4 q5 q6 [gripper] [vlim]       # angles in degrees, vlim in rad/s
state                                    # print current state
q                                        # quit
```

## Kinematics Tests

### 6. Forward Kinematics

`5_fk_test.rs` computes the end-effector pose from six joint angles. It does not connect to
hardware.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 5_fk_test
```

Example input:

```text
0 0 0 0 0 0
45 -30 15 -60 90 180
```

### 7. Inverse Kinematics

`6_ik_test.rs` solves a position-only IK target. It does not connect to hardware.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 6_ik_test
```

Input format:

```text
x y z                         # meters
```

This example uses the same C++/Pinocchio IK backend as the Python examples through
`librebotarm_math.so`.

## Real Machine End-effector Control

### 8. End-effector IK Control

`7_arm_ik_control.rs` reads the current joint pose, solves a position-only target with the
C++/Pinocchio IK backend, and streams a joint-space POS_VEL move.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 7_arm_ik_control -- --port /dev/ttyACM0 --rate 150
```

Input format:

```text
x y z                         # meters
state
q
```

### 9. End-effector Trajectory Control

`8_arm_traj_control.rs` solves a position-only target with the C++/Pinocchio IK backend and
interpolates the joint move over the requested duration.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 8_arm_traj_control -- --port /dev/ttyACM0 --rate 150
```

Input format:

```text
x y z [duration]              # meters, seconds
state
q
```

The Rust trajectory example currently performs joint-space interpolation after IK. For richer
Cartesian trajectory generation, use the Python examples or the LeRobot follower.

## Gravity Compensation

### 10. Basic Gravity Compensation

`9_gravity_compensation.rs` computes generalized gravity with the same C++/Pinocchio dynamics
backend used by Python, then sends MIT torque feed-forward commands.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 9_gravity_compensation -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true \
  --kp 0.0 \
  --kd 1.0
```

Important options:

| Option | Description |
|---|---|
| `--kp`, `--kd` | MIT command stiffness and damping for arm joints; defaults are `0.0` and `1.0` |
| `--gripper-kp`, `--gripper-kd` | MIT command stiffness and damping for the extra gripper joint; defaults are `0.0` and `1.0` |
| `--use_gripper=true/false` | `true` sends gripper MIT hold and uses `end_link` inertial scale `0.7`; `false` skips gripper motor command and removes `end_link` inertial from the gravity model |
| `--urdf <path>` | Optional URDF path; defaults to the packaged B601 fixed-end URDF |

### 11. Gravity Compensation with Velocity Lock

`10_gravity_compensation_lock.rs` adds a simple joint-velocity lock on top of the same
C++/Pinocchio gravity torque. If measured joint velocity is above `--vel-threshold`, the lock
target follows the current pose; otherwise it holds the last locked pose.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 10_gravity_compensation_lock -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --use_gripper=true \
  --lock-kp 8.0 \
  --lock-kd 1.0
```

Important options:

| Option | Description |
|---|---|
| `--vel-threshold` | Joint velocity threshold in rad/s; default `0.04` |
| `--lock-kp`, `--lock-kd` | MIT lock stiffness and damping; defaults are `8.0` and `1.0`. `--kp` / `--kd` are accepted as aliases |
| `--use_gripper=true/false` | `true` sends gripper MIT hold and uses `end_link` inertial scale `0.7`; `false` skips gripper motor command and removes `end_link` inertial from the gravity model |
| `--urdf <path>` | Optional URDF path; defaults to the packaged B601 fixed-end URDF |

## Gripper Console

`gripper_test.rs` is an interactive gripper terminal for FORCE_POS and POS_VEL commands.

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin gripper_test -- --port /dev/ttyACM0
```

Interactive commands:

| Command | Description |
|---|---|
| `open` | Open to `0 deg` using FORCE_POS |
| `close` | Close to `-270 deg` using FORCE_POS |
| `pos <deg>` | FORCE_POS target with default velocity and torque ratio |
| `forcepos <deg> [vlim ratio]` | FORCE_POS target |
| `posvel <deg> [vlim]` | POS_VEL target |
| `state` | Print arm and gripper state |
| `q` | Stop and disconnect |

## Dynamics Identification

The recommended workflow is to record a trajectory by hand while gravity compensation is running,
then replay that real checked trajectory in POS_VEL mode to collect `q/dq/ddq/tau`. This is safer
than generating random offline excitation trajectories.

`11_record_gravity_trajectory.rs` enters gravity compensation immediately. Move the arm to a good
start pose, press Enter to start recording, press Enter again to stop and save, then move the arm
back to zero or a safe pose while gravity compensation is still active. Press `Ctrl+C` to exit.
If the arm sinks, increase `--gravity-scale` slightly; if it floats upward, decrease it slightly.

The identification CSV format is:

```text
time,q1,q2,q3,q4,q5,q6,dq1,...,dq6,ddq1,...,ddq6,tau1,...,tau6
```

Units are rad, rad/s, rad/s^2, and Nm.

### Option A: Arm First, Then Arm With Gripper

Use this when you want to separate the arm body from the new gripper. In the first stage, remove the
gripper or make sure the arm has no extra payload. In the second stage, mount the new gripper and fit
only the `end_link` payload.

#### A1. Record the Arm-Only Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 11_record_gravity_trajectory -- \
  --output calibration/arm_only_trajectory.csv \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0 \
  --use_gripper=false
```

#### A2. Preview the Arm-Only Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/arm_only_trajectory.csv \
  --output calibration/id_data_arm_only.csv \
  --port /dev/ttyACM0
```

#### A3. Replay and Collect Arm-Only Data

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/arm_only_trajectory.csv \
  --output calibration/id_data_arm_only.csv \
  --port /dev/ttyACM0 \
  --execute
```

#### A4. Identify the Arm Body

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 13_identify_dynamics -- \
  --data calibration/id_data_arm_only.csv \
  --mode full \
  --ignore-payload-link end_link \
  --output calibration/identified_arm_rust.yaml \
  --urdf-output calibration/identified_arm_rust.urdf
```

`--ignore-payload-link end_link` temporarily removes the `end_link` inertial during identification
so stale gripper parameters do not pollute the arm-body result.

#### A5. Mount the New Gripper and Record a Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 11_record_gravity_trajectory -- \
  --output calibration/arm_with_gripper_trajectory.csv \
  --urdf calibration/tool_calibration.urdf \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0
```

#### A6. Preview the Arm-With-Gripper Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/arm_with_gripper_trajectory.csv \
  --output calibration/id_data_arm_with_gripper.csv \
  --port /dev/ttyACM0
```

#### A7. Replay and Collect Arm-With-Gripper Data

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/arm_with_gripper_trajectory.csv \
  --output calibration/id_data_arm_with_gripper.csv \
  --port /dev/ttyACM0 \
  --execute
```

#### A8. Identify Only the Gripper Payload

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 13_identify_dynamics -- \
  --data calibration/id_data_arm_with_gripper.csv \
  --urdf calibration/identified_arm_rust.urdf \
  --mode payload \
  --payload-link end_link \
  --payload-parameters 4 \
  --output calibration/identified_gripper_payload_rust.yaml \
  --urdf-output calibration/identified_arm_with_gripper_rust.urdf
```

If the trajectory has enough velocity and acceleration excitation, you can change
`--payload-parameters 4` to `--payload-parameters 10`. Four parameters fit only mass and COM and
are usually better for improving gravity compensation first; ten parameters also fit inertia but
are more sensitive to data quality.

#### A9. Test Gravity Compensation With the Identified URDF

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 9_gravity_compensation -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --kd 1.0 \
  --urdf calibration/identified_arm_with_gripper_rust.urdf
```

### Option B: Identify Arm and Gripper Together

Use this faster workflow when you believe both the arm and gripper parameters are inaccurate and you
accept the coupling between `link6` and fixed `end_link`. It gives a fitted overall model, but the
physical parameter split is not necessarily unique.

#### B1. Record the Arm-With-Gripper Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 11_record_gravity_trajectory -- \
  --output calibration/all_with_gripper_trajectory.csv \
  --urdf calibration/tool_calibration.urdf \
  --port /dev/ttyACM0 \
  --rate 200 \
  --sample-rate 100 \
  --kd 1.0 \
  --gravity-scale 1.0
```

#### B2. Preview the Trajectory

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/all_with_gripper_trajectory.csv \
  --output calibration/id_data_all_with_gripper.csv \
  --port /dev/ttyACM0
```

#### B3. Replay and Collect Data

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 12_collect_identification_data -- \
  --trajectory calibration/all_with_gripper_trajectory.csv \
  --output calibration/id_data_all_with_gripper.csv \
  --port /dev/ttyACM0 \
  --execute
```

#### B4. Identify Arm and Gripper Together

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 13_identify_dynamics -- \
  --data calibration/id_data_all_with_gripper.csv \
  --urdf calibration/tool_calibration.urdf \
  --mode full \
  --output calibration/identified_dynamics_all_rust.yaml \
  --urdf-output calibration/identified_dynamics_all_rust.urdf
```

For full-model URDF write-back, the Rust example folds fixed-child inertials into the corresponding
movable child link and removes the fixed child inertial blocks.

#### B5. Test Gravity Compensation With the Overall URDF

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 9_gravity_compensation -- \
  --port /dev/ttyACM0 \
  --rate 200 \
  --kd 1.0 \
  --urdf calibration/identified_dynamics_all_rust.urdf
```

Use `--mode base` to fit a QR-selected independent parameter set. Base parameters cannot be written
back to URDF uniquely.

`14_verify_identification.rs` supports only `full/base` results. If you have a separate verification
dataset, use it for workflow B or the arm-only part of workflow A:

```bash
cargo run --manifest-path example/rust/Cargo.toml --bin 14_verify_identification -- \
  --data calibration/id_data_verify.csv \
  --params calibration/identified_dynamics_all_rust.yaml
```

## Build Check

Use offline mode if the required crates are already cached:

```bash
cargo check --manifest-path example/rust/Cargo.toml --offline
```

Format the Rust examples:

```bash
cargo fmt --manifest-path example/rust/Cargo.toml
```

## Notes

- `/dev/ttyACM*` and `/dev/ttyUSB*` can change after unplugging USB devices. Check the device list
  before running hardware examples.
- The examples open `/dev/*` targets as Damiao serial bridge at `921600` baud. Non-`/dev` targets
  are treated as SocketCAN channel names.
- FK, IK, and gravity compensation examples require `librebotarm_math.so`; run `./build.sh` first
  or set `REBOTARM_MATH_LIB` manually.
