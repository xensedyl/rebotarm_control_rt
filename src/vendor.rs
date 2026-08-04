//! 厂商统一派发层：在 Rust 内直接调用 motorbridge 的 vendor crates，
//! 镜像 `motor_abi` 的 ControllerInner / MotorHandleInner 枚举派发与状态归一化。
//!
//! 与 motor_abi 的对应：
//!   - lifecycle  ← motor_lifecycle_ffi.rs
//!   - control    ← motor_control_ffi.rs
//!   - register   ← motor_register_ffi.rs
//!   - get_state  ← state_ffi.rs（含 deg→rad、robstride 故障位打包）

use std::f32::consts::PI;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use motor_vendor_damiao::{ControlMode as DamiaoMode, DamiaoController, DamiaoMotor};
use motor_vendor_hightorque::{HightorqueController, HightorqueMotor};
use motor_vendor_myactuator::{MyActuatorController, MyActuatorMotor};
use motor_vendor_robstride::{
    ControlMode as RsMode, ParameterId as RsParameterId, ParameterValue, RobstrideController,
    RobstrideMotor,
};

/// 归一化后的电机状态（位置/速度单位为 rad、rad/s；力矩 N·m）。
#[derive(Clone, Copy, Debug, Default)]
pub struct NormState {
    pub status_code: u8,
    pub pos: f64,
    pub vel: f64,
    pub torq: f64,
}

// ------------------------------------------------------------------
// 模式映射（照搬 motor_abi/src/lib.rs）
// ------------------------------------------------------------------

fn to_damiao_mode(mode: u32) -> Result<DamiaoMode, String> {
    match mode {
        1 => Ok(DamiaoMode::Mit),
        2 => Ok(DamiaoMode::PosVel),
        3 => Ok(DamiaoMode::Vel),
        4 => Ok(DamiaoMode::ForcePos),
        _ => Err("Damiao mode 必须为 1(MIT)/2(POS_VEL)/3(VEL)/4(FORCE_POS)".to_string()),
    }
}

fn to_robstride_mode(mode: u32) -> Result<RsMode, String> {
    match mode {
        1 => Ok(RsMode::Mit),
        2 => Ok(RsMode::Position),
        3 => Ok(RsMode::Velocity),
        5 => Ok(RsMode::PositionCsp),
        _ => Err("RobStride mode 必须为 1(MIT)/2(POSITION)/3(VELOCITY)/5(POSITION-CSP)".to_string()),
    }
}

fn validate_myactuator_mode(mode: u32) -> Result<(), String> {
    match mode {
        1..=3 => Ok(()),
        _ => Err("MyActuator mode 必须为 1(CURRENT)/2(POSITION)/3(VELOCITY)".to_string()),
    }
}

// ------------------------------------------------------------------
// UniController
// ------------------------------------------------------------------

pub enum UniController {
    Damiao(DamiaoController),
    MyActuator(MyActuatorController),
    Robstride(RobstrideController),
    Hightorque(HightorqueController),
}

pub enum UniMotor {
    Damiao(Arc<DamiaoMotor>),
    MyActuator(Arc<MyActuatorMotor>),
    Robstride(Arc<RobstrideMotor>),
    Hightorque(Arc<HightorqueMotor>),
}

impl UniController {
    /// 按 channel 与 vendor 创建控制器。
    /// `/dev/tty*` ⇒ Damiao 串口桥（仅 Damiao 支持）；否则 socketcan。
    pub fn new(channel: &str, vendor: &str) -> Result<Self, String> {
        let is_serial = channel.starts_with("/dev/tty");
        match vendor {
            "damiao" => {
                let c = if is_serial {
                    DamiaoController::new_dm_serial(channel, 921600)
                } else {
                    DamiaoController::new_socketcan(channel)
                }
                .map_err(|e| e.to_string())?;
                Ok(UniController::Damiao(c))
            }
            other if is_serial => Err(format!(
                "串口桥 {channel} 仅支持 damiao；vendor={other} 需使用 CAN 通道"
            )),
            "myactuator" => MyActuatorController::new_socketcan(channel)
                .map(UniController::MyActuator)
                .map_err(|e| e.to_string()),
            "robstride" => RobstrideController::new_socketcan(channel)
                .map(UniController::Robstride)
                .map_err(|e| e.to_string()),
            "hightorque" => HightorqueController::new_socketcan(channel)
                .map(UniController::Hightorque)
                .map_err(|e| e.to_string()),
            other => Err(format!("不支持的 vendor: {other}")),
        }
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<UniMotor, String> {
        match self {
            UniController::Damiao(c) => c
                .add_motor(motor_id, feedback_id, model)
                .map(UniMotor::Damiao)
                .map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c
                .add_motor(motor_id, feedback_id, model)
                .map(UniMotor::MyActuator)
                .map_err(|e| e.to_string()),
            UniController::Robstride(c) => c
                .add_motor(motor_id, feedback_id, model)
                .map(UniMotor::Robstride)
                .map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c
                .add_motor(motor_id, feedback_id, model)
                .map(UniMotor::Hightorque)
                .map_err(|e| e.to_string()),
        }
    }

    pub fn enable_all(&self) -> Result<(), String> {
        match self {
            UniController::Damiao(c) => c.enable_all().map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c.enable_all().map_err(|e| e.to_string()),
            UniController::Robstride(c) => c.enable_all().map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c.enable_all().map_err(|e| e.to_string()),
        }
    }

    pub fn disable_all(&self) -> Result<(), String> {
        match self {
            UniController::Damiao(c) => c.disable_all().map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c.disable_all().map_err(|e| e.to_string()),
            UniController::Robstride(c) => c.disable_all().map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c.disable_all().map_err(|e| e.to_string()),
        }
    }

    pub fn poll_feedback_once(&self) -> Result<(), String> {
        match self {
            UniController::Damiao(c) => c.poll_feedback_once().map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c.poll_feedback_once().map_err(|e| e.to_string()),
            UniController::Robstride(c) => c.poll_feedback_once().map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c.poll_feedback_once().map_err(|e| e.to_string()),
        }
    }

    pub fn shutdown(&self) -> Result<(), String> {
        match self {
            UniController::Damiao(c) => c.shutdown().map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c.shutdown().map_err(|e| e.to_string()),
            UniController::Robstride(c) => c.shutdown().map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c.shutdown().map_err(|e| e.to_string()),
        }
    }

    pub fn close_bus(&self) -> Result<(), String> {
        match self {
            UniController::Damiao(c) => c.close_bus().map_err(|e| e.to_string()),
            UniController::MyActuator(c) => c.close_bus().map_err(|e| e.to_string()),
            UniController::Robstride(c) => c.close_bus().map_err(|e| e.to_string()),
            UniController::Hightorque(c) => c.close_bus().map_err(|e| e.to_string()),
        }
    }
}

// ------------------------------------------------------------------
// UniMotor —— 每电机操作
// ------------------------------------------------------------------

impl UniMotor {
    // enable/disable per-motor 保留以对齐 vendor API（RobotArm 走控制器级 enable_all）。
    #[allow(dead_code)]
    pub fn enable(&self) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.enable().map_err(|e| e.to_string()),
            UniMotor::MyActuator(m) => m.release_brake().map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => m.enable().map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m.enable().map_err(|e| e.to_string()),
        }
    }

    #[allow(dead_code)]
    pub fn disable(&self) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.disable().map_err(|e| e.to_string()),
            UniMotor::MyActuator(m) => m.shutdown_motor().map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => m.disable().map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m.disable().map_err(|e| e.to_string()),
        }
    }

    pub fn set_zero(&self) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.set_zero_position().map_err(|e| e.to_string()),
            UniMotor::MyActuator(_) => {
                Err("MyActuator 不支持 set_zero_position".to_string())
            }
            UniMotor::Robstride(m) => m.set_zero_position().map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m.set_zero_position().map_err(|e| e.to_string()),
        }
    }

    pub fn ensure_mode(&self, mode: u32, timeout_ms: u32) -> Result<(), String> {
        let timeout = Duration::from_millis(timeout_ms as u64);
        match self {
            UniMotor::Damiao(m) => {
                let dm = to_damiao_mode(mode)?;
                m.ensure_control_mode(dm, timeout).map_err(|e| e.to_string())
            }
            UniMotor::MyActuator(_) => validate_myactuator_mode(mode),
            UniMotor::Robstride(m) => {
                let rm = to_robstride_mode(mode)?;
                m.ensure_control_mode(rm, timeout).map_err(|e| e.to_string())
            }
            UniMotor::Hightorque(m) => {
                m.ensure_control_mode(mode, timeout).map_err(|e| e.to_string())
            }
        }
    }

    pub fn ensure_mode_for_control(&self, mode: u32, timeout_ms: u32) -> Result<(), String> {
        self.ensure_mode(mode, timeout_ms)?;
        if matches!(self, UniMotor::Robstride(_)) {
            // RobStride's reliable mode switch path disables torque first; restore
            // enable here so the high-level mode_* APIs remain ready for commands.
            self.enable()?;
        }
        Ok(())
    }

    pub fn send_mit(&self, pos: f32, vel: f32, kp: f32, kd: f32, tau: f32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m
                .send_cmd_mit(pos, vel, kp, kd, tau)
                .map_err(|e| e.to_string()),
            UniMotor::MyActuator(_) => {
                Err("MyActuator 不支持 send_mit；请用 pos_vel 或 vel".to_string())
            }
            UniMotor::Robstride(m) => m
                .send_cmd_mit(pos, vel, kp, kd, tau)
                .map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m
                .send_cmd_mit(pos, vel, kp, kd, tau)
                .map_err(|e| e.to_string()),
        }
    }

    pub fn send_pos_vel(&self, pos: f32, vlim: f32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.send_cmd_pos_vel(pos, vlim).map_err(|e| e.to_string()),
            UniMotor::MyActuator(m) => m
                .send_position_absolute_setpoint(pos * (180.0 / PI), vlim * (180.0 / PI))
                .map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => {
                // mode_pos_vel()/ensure_mode_for_control() owns run_mode switching
                // and enable sequencing. The RT loop must only refresh runtime
                // targets; repeatedly writing run_mode inside the loop can reset
                // RobStride's position controller and make loc_ref look ignored.
                let v = vlim.abs();
                if v.is_finite() && v > 0.0 {
                    m.write_parameter(RsParameterId::VelocityLimit as u16, ParameterValue::F32(v))
                        .map_err(|e| e.to_string())?;
                }
                m.write_parameter(RsParameterId::PositionTarget as u16, ParameterValue::F32(pos))
                    .map_err(|e| e.to_string())
            }
            UniMotor::Hightorque(m) => m.send_cmd_pos_vel(pos, vlim).map_err(|e| e.to_string()),
        }
    }

    pub fn send_force_pos(&self, pos: f32, vlim: f32, torque_limit_ratio: f32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m
                .send_cmd_force_pos(pos, vlim, torque_limit_ratio)
                .map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m
                .send_cmd_force_pos(pos, vlim, torque_limit_ratio)
                .map_err(|e| e.to_string()),
            _ => Err("send_force_pos 仅 Damiao / HighTorque 支持".to_string()),
        }
    }

    pub fn send_vel(&self, vel: f32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.send_cmd_vel(vel).map_err(|e| e.to_string()),
            UniMotor::MyActuator(m) => m
                .send_velocity_setpoint(vel * (180.0 / PI))
                .map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => m.set_velocity_target(vel).map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m.send_cmd_vel(vel).map_err(|e| e.to_string()),
        }
    }

    // 保留以对齐 vendor API（RobotArm 的增益写入统一走 write_pos_vel_gains）。
    #[allow(dead_code)]
    pub fn write_register_f32(&self, rid: u8, value: f32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.write_register_f32(rid, value).map_err(|e| e.to_string()),
            _ => Err("寄存器写入仅 Damiao 支持".to_string()),
        }
    }

    pub fn get_register_f32(&self, rid: u8, timeout_ms: u64) -> Result<f32, String> {
        match self {
            UniMotor::Damiao(m) => m
                .get_register_f32(rid, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string()),
            _ => Err("寄存器读取仅 Damiao 支持".to_string()),
        }
    }

    // --------------------------------------------------------------
    // POS_VEL 增益（按厂商派发）
    // --------------------------------------------------------------

    /// 写入 POS_VEL（位置-速度）模式的环路增益，按厂商映射到各自的寄存器/参数表。
    /// 仅写入 > 0 的项，语义与 reBotArm_control_py 的 `_write_pv_params` 一致：
    ///   - Damiao：寄存器 25(KP_ASR)/26(KI_ASR)/27(KP_APR)/28(KI_APR)
    ///   - 灵足 RobStride：0x7017(limit_spd)=vlim、0x701F(spd_kp)、0x7020(spd_ki)、0x701E(loc_kp)
    ///   - 其余厂商无对应参数表：no-op
    pub fn write_pos_vel_gains(
        &self,
        vel_kp: f32,
        vel_ki: f32,
        pos_kp: f32,
        pos_ki: f32,
        vlim: f32,
    ) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => {
                if vel_kp > 0.0 {
                    m.write_register_f32(25, vel_kp).map_err(|e| e.to_string())?;
                }
                if vel_ki > 0.0 {
                    m.write_register_f32(26, vel_ki).map_err(|e| e.to_string())?;
                }
                if pos_kp > 0.0 {
                    m.write_register_f32(27, pos_kp).map_err(|e| e.to_string())?;
                }
                if pos_ki > 0.0 {
                    m.write_register_f32(28, pos_ki).map_err(|e| e.to_string())?;
                }
                Ok(())
            }
            UniMotor::Robstride(m) => {
                let write = |id: u16, v: f32| -> Result<(), String> {
                    m.write_parameter(id, ParameterValue::F32(v))
                        .map_err(|e| e.to_string())?;
                    thread::sleep(Duration::from_millis(10));
                    Ok(())
                };
                if vlim > 0.0 {
                    write(0x7017, vlim)?; // limit_spd
                }
                if vel_kp > 0.0 {
                    write(0x701F, vel_kp)?; // spd_kp
                }
                if vel_ki > 0.0 {
                    write(0x7020, vel_ki)?; // spd_ki
                }
                if pos_kp > 0.0 {
                    write(0x701E, pos_kp)?; // loc_kp
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// 读取 POS_VEL 模式环路增益 (vel_kp, vel_ki, pos_kp, pos_ki)。
    ///   - Damiao：寄存器 25/26/27/28
    ///   - 灵足 RobStride：0x701F(spd_kp)、0x7020(spd_ki)、0x701E(loc_kp)；无 pos_ki，返回 0.0
    pub fn read_pos_vel_gains(&self, timeout_ms: u64) -> Result<(f32, f32, f32, f32), String> {
        let timeout = Duration::from_millis(timeout_ms);
        match self {
            UniMotor::Damiao(m) => {
                let vel_kp = m.get_register_f32(25, timeout).map_err(|e| e.to_string())?;
                let vel_ki = m.get_register_f32(26, timeout).map_err(|e| e.to_string())?;
                let pos_kp = m.get_register_f32(27, timeout).map_err(|e| e.to_string())?;
                let pos_ki = m.get_register_f32(28, timeout).map_err(|e| e.to_string())?;
                Ok((vel_kp, vel_ki, pos_kp, pos_ki))
            }
            UniMotor::Robstride(m) => {
                let read = |id: u16| -> Result<f32, String> {
                    m.get_parameter_f32(id, timeout).map_err(|e| e.to_string())
                };
                Ok((read(0x701F)?, read(0x7020)?, read(0x701E)?, 0.0))
            }
            _ => Err("read_pos_vel_gains 仅 Damiao / RobStride 支持".to_string()),
        }
    }

    // --------------------------------------------------------------
    // 通用辅助命令
    // --------------------------------------------------------------

    /// 清除电机错误状态（Damiao / 灵足 RobStride / HighTorque 支持）。
    pub fn clear_error(&self) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.clear_error().map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => m.clear_error().map_err(|e| e.to_string()),
            UniMotor::Hightorque(m) => m.clear_error().map_err(|e| e.to_string()),
            UniMotor::MyActuator(_) => Err("MyActuator 不支持 clear_error".to_string()),
        }
    }

    // --------------------------------------------------------------
    // 灵足（RobStride）底层专用接口 —— 镜像 motorbridge 的 robstride_* C ABI
    // --------------------------------------------------------------

    fn as_robstride(&self) -> Result<&Arc<RobstrideMotor>, String> {
        match self {
            UniMotor::Robstride(m) => Ok(m),
            _ => Err("robstride_* 接口仅灵足（RobStride）电机支持".to_string()),
        }
    }

    /// Ping 电机（type-0 GET_DEVICE_ID），返回 (device_id, responder_id)。
    pub fn robstride_ping(&self, timeout_ms: u64) -> Result<(u8, u8), String> {
        let reply = self
            .as_robstride()?
            .ping(Duration::from_millis(timeout_ms))
            .map_err(|e| e.to_string())?;
        Ok((reply.device_id, reply.responder_id))
    }

    /// 开/关电机主动上报（type-24 ACTIVE_REPORT）。
    /// 灵足电机没有单帧状态查询命令，开启主动上报后 motorbridge 的后台轮询
    /// 会持续更新状态缓存，get_state 才能拿到实时反馈。
    pub fn robstride_set_active_report(&self, enabled: bool) -> Result<(), String> {
        self.as_robstride()?
            .set_active_report(enabled)
            .map_err(|e| e.to_string())
    }

    /// 保存参数到电机（type-22 SAVE_PARAMETERS），断电后仍然生效。
    pub fn robstride_save_parameters(&self) -> Result<(), String> {
        self.as_robstride()?
            .save_parameters()
            .map_err(|e| e.to_string())
    }

    pub fn robstride_write_param_f32(&self, param_id: u16, value: f32) -> Result<(), String> {
        self.as_robstride()?
            .write_parameter(param_id, ParameterValue::F32(value))
            .map_err(|e| e.to_string())
    }

    pub fn robstride_write_param_u8(&self, param_id: u16, value: u8) -> Result<(), String> {
        self.as_robstride()?
            .write_parameter(param_id, ParameterValue::U8(value))
            .map_err(|e| e.to_string())
    }

    pub fn robstride_write_param_u16(&self, param_id: u16, value: u16) -> Result<(), String> {
        self.as_robstride()?
            .write_parameter(param_id, ParameterValue::U16(value))
            .map_err(|e| e.to_string())
    }

    pub fn robstride_write_param_u32(&self, param_id: u16, value: u32) -> Result<(), String> {
        self.as_robstride()?
            .write_parameter(param_id, ParameterValue::U32(value))
            .map_err(|e| e.to_string())
    }

    pub fn robstride_get_param_f32(&self, param_id: u16, timeout_ms: u64) -> Result<f32, String> {
        self.as_robstride()?
            .get_parameter_f32(param_id, Duration::from_millis(timeout_ms))
            .map_err(|e| e.to_string())
    }

    pub fn robstride_get_param_u8(&self, param_id: u16, timeout_ms: u64) -> Result<u8, String> {
        match self
            .as_robstride()?
            .get_parameter(param_id, Duration::from_millis(timeout_ms))
            .map_err(|e| e.to_string())?
        {
            ParameterValue::U8(v) => Ok(v),
            other => Err(format!("参数 0x{param_id:04X} 不是 u8 类型: {other:?}")),
        }
    }

    pub fn robstride_get_param_u16(&self, param_id: u16, timeout_ms: u64) -> Result<u16, String> {
        match self
            .as_robstride()?
            .get_parameter(param_id, Duration::from_millis(timeout_ms))
            .map_err(|e| e.to_string())?
        {
            ParameterValue::U16(v) => Ok(v),
            other => Err(format!("参数 0x{param_id:04X} 不是 u16 类型: {other:?}")),
        }
    }

    pub fn robstride_get_param_u32(&self, param_id: u16, timeout_ms: u64) -> Result<u32, String> {
        match self
            .as_robstride()?
            .get_parameter(param_id, Duration::from_millis(timeout_ms))
            .map_err(|e| e.to_string())?
        {
            ParameterValue::U32(v) => Ok(v),
            other => Err(format!("参数 0x{param_id:04X} 不是 u32 类型: {other:?}")),
        }
    }

    /// 灵足原生 CSP 位置模式（run_mode=5）：内部依次 set_mode(CSP)、enable、
    /// 写 0x7017 limit_spd 与 0x7016 loc_ref。
    pub fn robstride_send_pos_vel_csp(&self, pos: f32, vlim: f32) -> Result<(), String> {
        self.as_robstride()?
            .send_cmd_pos_vel_csp(pos, vlim)
            .map_err(|e| e.to_string())
    }

    /// 设置电机侧 CAN 超时看门狗（ms）：超时未收到控制帧则电机自动停机。
    /// 镜像 motor_abi 的 set_can_timeout_ms：Damiao 写寄存器 9（值=ms*20），RobStride 写 0x7028。
    /// timeout_ms=0 表示禁用看门狗。
    pub fn set_can_timeout_ms(&self, timeout_ms: u32) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m
                .write_register_u32(9, timeout_ms.saturating_mul(20))
                .map_err(|e| e.to_string()),
            UniMotor::Robstride(m) => m
                .write_parameter(0x7028, ParameterValue::U32(timeout_ms))
                .map_err(|e| e.to_string()),
            _ => Err("set_can_timeout_ms 仅 Damiao / RobStride 支持".to_string()),
        }
    }

    pub fn request_feedback(&self) -> Result<(), String> {
        match self {
            UniMotor::Damiao(m) => m.request_motor_feedback().map_err(|e| e.to_string()),
            UniMotor::MyActuator(m) => m.request_status().map_err(|e| e.to_string()),
            // RobStride 无单帧状态请求命令；保持 no-op，避免阻塞控制循环。
            UniMotor::Robstride(_) => Ok(()),
            UniMotor::Hightorque(m) => m
                .request_motor_feedback(Duration::from_millis(500))
                .map(|_| ())
                .map_err(|e| e.to_string()),
        }
    }

    /// 读取最新反馈快照并归一化（无数据返回 None）。照搬 state_ffi.rs。
    pub fn get_state(&self) -> Option<NormState> {
        match self {
            UniMotor::Damiao(m) => m.latest_state().map(|s| NormState {
                status_code: s.status_code,
                pos: s.pos as f64,
                vel: s.vel as f64,
                torq: s.torq as f64,
            }),
            UniMotor::MyActuator(m) => m.latest_state().map(|s| NormState {
                status_code: s.command,
                pos: (s.shaft_angle_deg * (PI / 180.0)) as f64,
                vel: (s.speed_dps * (PI / 180.0)) as f64,
                torq: s.current_a as f64,
            }),
            UniMotor::Robstride(m) => m.latest_state().map(|s| {
                let mut status = 0u8;
                if s.uncalibrated {
                    status |= 1 << 5;
                }
                if s.stall {
                    status |= 1 << 4;
                }
                if s.magnetic_encoder_fault {
                    status |= 1 << 3;
                }
                if s.overtemperature {
                    status |= 1 << 2;
                }
                if s.overcurrent {
                    status |= 1 << 1;
                }
                if s.undervoltage {
                    status |= 1;
                }
                NormState {
                    status_code: status,
                    pos: s.position as f64,
                    vel: s.velocity as f64,
                    torq: s.torque as f64,
                }
            }),
            UniMotor::Hightorque(m) => m.latest_state().map(|s| NormState {
                status_code: s.status_code,
                pos: s.pos as f64,
                vel: s.vel as f64,
                torq: s.torq as f64,
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanBus;
    use motor_core::test_support::MockBus;

    #[test]
    fn mode_mapping() {
        assert!(matches!(to_damiao_mode(1), Ok(DamiaoMode::Mit)));
        assert!(matches!(to_damiao_mode(2), Ok(DamiaoMode::PosVel)));
        assert!(matches!(to_damiao_mode(3), Ok(DamiaoMode::Vel)));
        assert!(to_damiao_mode(9).is_err());
        assert!(validate_myactuator_mode(2).is_ok());
        assert!(validate_myactuator_mode(9).is_err());
        assert!(matches!(to_robstride_mode(5), Ok(RsMode::PositionCsp)));
    }

    fn mock_robstride() -> (Arc<MockBus>, UniMotor) {
        let bus = Arc::new(MockBus::new());
        let m = RobstrideMotor::new(1, 0xFD, "rs-06", bus.clone() as Arc<dyn CanBus>)
            .expect("create robstride motor");
        (bus, UniMotor::Robstride(Arc::new(m)))
    }

    fn mock_damiao() -> UniMotor {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let m = DamiaoMotor::new(0x01, 0x11, "4310", bus).expect("create damiao motor");
        UniMotor::Damiao(Arc::new(m))
    }

    #[test]
    fn robstride_param_write_sends_write_parameter_frames() {
        let (bus, motor) = mock_robstride();
        motor
            .robstride_write_param_f32(0x7016, 1.25)
            .expect("write loc_ref");
        motor
            .robstride_write_param_u8(0x7029, 1)
            .expect("write zero_sta");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 2);
        for frame in sent.iter() {
            // 通信类型 18 = WRITE_PARAMETER，见 robstride/protocol.rs
            assert_eq!((frame.arbitration_id >> 24) & 0x1F, 18);
            assert!(frame.is_extended);
        }
        assert_eq!(u16::from_le_bytes([sent[0].data[0], sent[0].data[1]]), 0x7016);
        assert_eq!(
            f32::from_le_bytes([sent[0].data[4], sent[0].data[5], sent[0].data[6], sent[0].data[7]]),
            1.25
        );
    }

    #[test]
    fn robstride_pv_gains_map_to_parameter_table() {
        let (bus, motor) = mock_robstride();
        motor
            .write_pos_vel_gains(12.0, 0.1, 13.0, 0.0, 10.0)
            .expect("write pv gains");
        let sent = bus.sent.lock().expect("sent frames");
        // vlim + vel_kp + vel_ki + pos_kp（pos_ki 灵足无对应参数，不发送）
        let ids: Vec<u16> = sent
            .iter()
            .map(|f| u16::from_le_bytes([f.data[0], f.data[1]]))
            .collect();
        assert_eq!(ids, vec![0x7017, 0x701F, 0x7020, 0x701E]);
    }

    #[test]
    fn robstride_pv_gains_skip_non_positive_values() {
        let (bus, motor) = mock_robstride();
        motor
            .write_pos_vel_gains(0.0, 0.0, 0.0, 0.0, 0.0)
            .expect("no-op gains");
        assert!(bus.sent.lock().expect("sent frames").is_empty());
    }

    #[test]
    fn robstride_send_pos_vel_updates_runtime_targets_without_rewriting_mode() {
        let (bus, motor) = mock_robstride();
        motor
            .send_pos_vel(1.25, 2.5)
            .expect("send robstride pos vel target");
        let sent = bus.sent.lock().expect("sent frames");
        let ids: Vec<u16> = sent
            .iter()
            .map(|f| u16::from_le_bytes([f.data[0], f.data[1]]))
            .collect();
        assert_eq!(ids, vec![0x7017, 0x7016]);
        assert!(!ids.contains(&(RsParameterId::Mode as u16)));
        assert_eq!(
            f32::from_le_bytes([sent[1].data[4], sent[1].data[5], sent[1].data[6], sent[1].data[7]]),
            1.25
        );
    }

    #[test]
    fn robstride_helpers_reject_other_vendors() {
        let motor = mock_damiao();
        assert!(motor.robstride_write_param_f32(0x7016, 0.0).is_err());
        assert!(motor.robstride_get_param_f32(0x7019, 10).is_err());
        assert!(motor.robstride_set_active_report(true).is_err());
        assert!(motor.robstride_save_parameters().is_err());
        assert!(motor.robstride_send_pos_vel_csp(0.0, 1.0).is_err());
        assert!(motor.robstride_ping(10).is_err());
    }

    #[test]
    fn robstride_get_param_times_out_without_reply() {
        let (_bus, motor) = mock_robstride();
        assert!(motor.robstride_get_param_f32(0x7019, 5).is_err());
    }

    #[test]
    fn clear_error_dispatch() {
        let (bus, motor) = mock_robstride();
        // 灵足 clear_error 走 DISABLE(type-4) + data[0]=1，等待状态 ACK 会超时，
        // 但帧必须已发出。
        let _ = motor.clear_error();
        let sent = bus.sent.lock().expect("sent frames");
        assert!(!sent.is_empty());
        assert_eq!((sent[0].arbitration_id >> 24) & 0x1F, 4);
        assert_eq!(sent[0].data[0], 1);
    }
}
