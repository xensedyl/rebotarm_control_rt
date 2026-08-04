use libloading::Library;
use motor_vendor_damiao::{ControlMode as DamiaoMode, DamiaoController, DamiaoMotor};
use motor_vendor_robstride::{
    ControlMode as RobstrideMode, ParameterValue as RobstrideParameterValue, RobstrideController,
    RobstrideMotor,
};
use std::env;
use std::error::Error;
use std::f64::consts::PI;
use std::ffi::{c_char, c_double, c_int, c_void, CStr, CString};
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const ARM_DOF: usize = 6;
pub const ALL_DOF: usize = 7;
pub const DEFAULT_PORT: &str = "/dev/ttyACM0";
pub const DEFAULT_CONFIG_REL: &str = "python/rebotarm_control_rt/config/arm.yaml";
pub const DEFAULT_RATE_HZ: f64 = 150.0;
pub const DEFAULT_URDF_REL: &str =
    "python/rebotarm_control_rt/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf";
pub const END_LINK_LOAD_SCALE_WITH_GRIPPER: f64 = 0.7;

pub const ARM_LIMITS_RAD: [(f64, f64); ARM_DOF] = [
    (-145.0_f64.to_radians(), 145.0_f64.to_radians()),
    (-170.0_f64.to_radians(), 1.0_f64.to_radians()),
    (-200.0_f64.to_radians(), 1.0_f64.to_radians()),
    (-80.0_f64.to_radians(), 90.0_f64.to_radians()),
    (-90.0_f64.to_radians(), 90.0_f64.to_radians()),
    (-90.0_f64.to_radians(), 90.0_f64.to_radians()),
];

static STOP_REQUESTED: AtomicBool = AtomicBool::new(false);

#[cfg(unix)]
extern "C" fn handle_signal(_signal: libc::c_int) {
    STOP_REQUESTED.store(true, Ordering::SeqCst);
}

pub fn install_signal_handler() {
    STOP_REQUESTED.store(false, Ordering::SeqCst);
    #[cfg(unix)]
    unsafe {
        libc::signal(
            libc::SIGINT,
            handle_signal as *const () as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            handle_signal as *const () as libc::sighandler_t,
        );
    }
}

pub fn stop_requested() -> bool {
    STOP_REQUESTED.load(Ordering::SeqCst)
}

#[derive(Clone, Copy, Debug)]
pub struct JointSpec {
    pub name: &'static str,
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: &'static str,
    pub vendor: &'static str,
    pub mit_kp: f32,
    pub mit_kd: f32,
    pub vel_kp: f32,
    pub vel_ki: f32,
    pub pos_kp: f32,
    pub pos_ki: f32,
    pub vlim: f32,
}

pub const B601_JOINTS: [JointSpec; ALL_DOF] = [
    JointSpec {
        name: "shoulder_pan",
        motor_id: 0x01,
        feedback_id: 0x11,
        model: "4340P",
        vendor: "damiao",
        mit_kp: 120.0,
        mit_kd: 8.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "shoulder_lift",
        motor_id: 0x02,
        feedback_id: 0x12,
        model: "4340P",
        vendor: "damiao",
        mit_kp: 120.0,
        mit_kd: 8.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "elbow_flex",
        motor_id: 0x03,
        feedback_id: 0x13,
        model: "4340P",
        vendor: "damiao",
        mit_kp: 120.0,
        mit_kd: 8.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "wrist_flex",
        motor_id: 0x04,
        feedback_id: 0x14,
        model: "4310",
        vendor: "damiao",
        mit_kp: 18.0,
        mit_kd: 2.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "wrist_yaw",
        motor_id: 0x05,
        feedback_id: 0x15,
        model: "4310",
        vendor: "damiao",
        mit_kp: 18.0,
        mit_kd: 2.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "wrist_roll",
        motor_id: 0x06,
        feedback_id: 0x16,
        model: "4310",
        vendor: "damiao",
        mit_kp: 18.0,
        mit_kd: 2.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 2.617_993_8,
    },
    JointSpec {
        name: "gripper",
        motor_id: 0x07,
        feedback_id: 0x17,
        model: "4310",
        vendor: "damiao",
        mit_kp: 8.0,
        mit_kd: 1.0,
        vel_kp: 0.0,
        vel_ki: 0.0,
        pos_kp: 0.0,
        pos_ki: 0.0,
        vlim: 5.235_987_7,
    },
];

#[derive(Debug, Clone, Copy)]
pub enum ControlMode {
    Mit,
    PosVel,
    Vel,
    ForcePos,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorFeedbackState {
    pub status_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
}

#[derive(Clone, Debug)]
pub struct ArmConfig {
    pub channel: String,
    pub urdf_path: Option<PathBuf>,
    pub joints: Vec<JointSpec>,
}

#[derive(Default)]
struct JointBuilder {
    name: Option<String>,
    motor_id: Option<u16>,
    feedback_id: Option<u16>,
    model: Option<String>,
    vendor: Option<String>,
    mit_kp: Option<f32>,
    mit_kd: Option<f32>,
    vel_kp: Option<f32>,
    vel_ki: Option<f32>,
    pos_kp: Option<f32>,
    pos_ki: Option<f32>,
    vlim: Option<f32>,
}

impl JointBuilder {
    fn is_empty(&self) -> bool {
        self.name.is_none() && self.motor_id.is_none() && self.feedback_id.is_none()
    }

    fn build(self) -> Result<JointSpec, Box<dyn Error>> {
        Ok(JointSpec {
            name: Box::leak(self.name.ok_or("joint missing name")?.into_boxed_str()),
            motor_id: self.motor_id.ok_or("joint missing motor_id")?,
            feedback_id: self.feedback_id.ok_or("joint missing feedback_id")?,
            model: Box::leak(self.model.unwrap_or_else(|| "4340P".to_string()).into_boxed_str()),
            vendor: Box::leak(
                self.vendor
                    .unwrap_or_else(|| "damiao".to_string())
                    .to_ascii_lowercase()
                    .into_boxed_str(),
            ),
            mit_kp: self.mit_kp.unwrap_or(0.0),
            mit_kd: self.mit_kd.unwrap_or(0.0),
            vel_kp: self.vel_kp.unwrap_or(0.0),
            vel_ki: self.vel_ki.unwrap_or(0.0),
            pos_kp: self.pos_kp.unwrap_or(0.0),
            pos_ki: self.pos_ki.unwrap_or(0.0),
            vlim: self.vlim.unwrap_or(2.0),
        })
    }
}

pub fn arg_value(args: &[String], name: &str) -> Option<String> {
    let prefix = format!("{name}=");
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        if arg == name {
            return iter.next().cloned();
        }
        if let Some(value) = arg.strip_prefix(&prefix) {
            return Some(value.to_string());
        }
    }
    None
}

pub fn default_config_path() -> PathBuf {
    repo_root().join(DEFAULT_CONFIG_REL)
}

pub fn parse_config_path(args: &[String]) -> Option<PathBuf> {
    arg_value(args, "--config")
        .or_else(|| arg_value(args, "-c"))
        .map(PathBuf::from)
}

fn resolve_repo_path(path: impl AsRef<Path>, base: Option<&Path>) -> PathBuf {
    let path = path.as_ref();
    if path.is_absolute() {
        return path.to_path_buf();
    }
    let mut candidates = Vec::new();
    if let Some(base) = base {
        candidates.push(base.join(path));
    }
    candidates.push(repo_root().join(path));
    candidates.push(env::current_dir().unwrap_or_else(|_| PathBuf::from(".")).join(path));
    candidates.push(path.to_path_buf());
    candidates
        .into_iter()
        .find(|candidate| candidate.exists())
        .unwrap_or_else(|| path.to_path_buf())
}

fn yaml_scalar(value: &str) -> String {
    let mut value = value.split('#').next().unwrap_or("").trim().to_string();
    if value.len() >= 2 {
        let bytes = value.as_bytes();
        if (bytes[0] == b'"' && bytes[value.len() - 1] == b'"')
            || (bytes[0] == b'\'' && bytes[value.len() - 1] == b'\'')
        {
            value = value[1..value.len() - 1].to_string();
        }
    }
    value
}

fn yaml_value(line: &str) -> Option<(&str, String)> {
    let stripped = line.trim();
    let (key, value) = stripped.split_once(':')?;
    Some((key.trim(), yaml_scalar(value)))
}

fn parse_yaml_id(value: &str) -> Result<u16, Box<dyn Error>> {
    let value = value.trim();
    if let Some(hex) = value.strip_prefix("0x").or_else(|| value.strip_prefix("0X")) {
        Ok(u16::from_str_radix(hex, 16)?)
    } else {
        Ok(value.parse()?)
    }
}

fn finalize_joint(builder: &mut JointBuilder, joints: &mut Vec<JointSpec>) -> Result<(), Box<dyn Error>> {
    if builder.is_empty() {
        return Ok(());
    }
    let current = std::mem::take(builder);
    joints.push(current.build()?);
    Ok(())
}

pub fn load_arm_config(args: &[String]) -> Result<ArmConfig, Box<dyn Error>> {
    let Some(path) = parse_config_path(args) else {
        return Ok(ArmConfig {
            channel: DEFAULT_PORT.to_string(),
            urdf_path: Some(default_urdf_path()),
            joints: B601_JOINTS.to_vec(),
        });
    };
    let path = resolve_repo_path(path, None);
    let base = path.parent();
    let text = fs::read_to_string(&path)?;
    let mut channel = DEFAULT_PORT.to_string();
    let mut urdf_path: Option<PathBuf> = None;
    let mut joints = Vec::new();
    let mut current = JointBuilder::default();
    let mut section = "";

    for line in text.lines() {
        let raw = line.split('#').next().unwrap_or("");
        if raw.trim().is_empty() {
            continue;
        }
        let indent = raw.len() - raw.trim_start().len();
        let stripped = raw.trim();

        if indent == 0 {
            if let Some((key, value)) = yaml_value(stripped) {
                match key {
                    "channel" => channel = value,
                    "urdf_path" => urdf_path = Some(resolve_repo_path(value, base)),
                    _ => {}
                }
            }
            continue;
        }

        if stripped.starts_with("- name:") {
            finalize_joint(&mut current, &mut joints)?;
            current.name = Some(yaml_scalar(stripped.trim_start_matches("- name:")));
            section = "";
            continue;
        }

        if stripped == "MIT:" {
            section = "MIT";
            continue;
        }
        if stripped == "POS_VEL:" {
            section = "POS_VEL";
            continue;
        }

        let Some((key, value)) = yaml_value(stripped) else {
            continue;
        };
        match (section, key) {
            ("MIT", "kp") => current.mit_kp = value.parse().ok(),
            ("MIT", "kd") => current.mit_kd = value.parse().ok(),
            ("POS_VEL", "vel_kp") => current.vel_kp = value.parse().ok(),
            ("POS_VEL", "vel_ki") => current.vel_ki = value.parse().ok(),
            ("POS_VEL", "pos_kp") => current.pos_kp = value.parse().ok(),
            ("POS_VEL", "pos_ki") => current.pos_ki = value.parse().ok(),
            ("POS_VEL", "vlim") => current.vlim = value.parse().ok(),
            (_, "motor_id") => current.motor_id = Some(parse_yaml_id(&value)?),
            (_, "feedback_id") => current.feedback_id = Some(parse_yaml_id(&value)?),
            (_, "model") => current.model = Some(value),
            (_, "vendor") => current.vendor = Some(value),
            _ => {}
        }
    }
    finalize_joint(&mut current, &mut joints)?;
    if joints.is_empty() {
        return Err(format!("{} does not contain joints", path.display()).into());
    }
    Ok(ArmConfig {
        channel,
        urdf_path,
        joints,
    })
}

pub fn arm_joints(args: &[String]) -> Result<Vec<JointSpec>, Box<dyn Error>> {
    Ok(load_arm_config(args)?.joints)
}

pub fn arg_values(args: &[String], name: &str) -> Vec<String> {
    let mut out = Vec::new();
    let prefix = format!("{name}=");
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        if arg == name {
            if let Some(value) = iter.next() {
                out.push(value.clone());
            }
        } else if let Some(value) = arg.strip_prefix(&prefix) {
            out.push(value.to_string());
        }
    }
    out
}

pub fn has_flag(args: &[String], name: &str) -> bool {
    args.iter().any(|arg| arg == name)
}

pub fn parse_port(args: &[String]) -> String {
    arg_value(args, "--port")
        .or_else(|| arg_value(args, "-p"))
        .or_else(|| load_arm_config(args).ok().map(|cfg| cfg.channel))
        .unwrap_or_else(|| DEFAULT_PORT.to_string())
}

pub fn parse_rate(args: &[String], default_hz: f64) -> f64 {
    arg_value(args, "--rate")
        .and_then(|value| value.parse::<f64>().ok())
        .filter(|value| *value > 0.0)
        .unwrap_or(default_hz)
}

pub fn parse_float_arg(args: &[String], name: &str, default: f64) -> f64 {
    arg_value(args, name)
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(default)
}

pub fn parse_bool_arg(args: &[String], name: &str, default: bool) -> bool {
    arg_value(args, name)
        .map(|value| matches!(value.as_str(), "1" | "true" | "True" | "yes" | "on"))
        .unwrap_or(default)
}

pub fn parse_joint(value: &str) -> Result<usize, Box<dyn Error>> {
    if let Ok(index) = value.parse::<usize>() {
        if index < ALL_DOF {
            return Ok(index);
        }
        if (1..=ALL_DOF).contains(&index) {
            return Ok(index - 1);
        }
    }
    if let Some(rest) = value.strip_prefix("joint") {
        let one_based = rest.parse::<usize>()?;
        if (1..=ALL_DOF).contains(&one_based) {
            return Ok(one_based - 1);
        }
    }
    current_joints_or_default()
        .iter()
        .position(|joint| joint.name == value)
        .ok_or_else(|| format!("unknown joint: {value}").into())
}

fn current_joints_or_default() -> Vec<JointSpec> {
    let args: Vec<String> = env::args().skip(1).collect();
    arm_joints(&args).unwrap_or_else(|_| B601_JOINTS.to_vec())
}

pub fn parse_floats(line: &str) -> Result<Vec<f64>, Box<dyn Error>> {
    line.split_whitespace()
        .map(|part| Ok(part.parse::<f64>()?))
        .collect()
}

pub fn prompt(text: &str) -> io::Result<Option<String>> {
    print!("{text}");
    io::stdout().flush()?;
    let mut line = String::new();
    let n = io::stdin().read_line(&mut line)?;
    if n == 0 {
        return Ok(None);
    }
    Ok(Some(line.trim().to_string()))
}

pub fn deg_to_rad_f32(value: f64) -> f32 {
    value.to_radians() as f32
}

pub fn rad_to_deg_f32(value: f32) -> f32 {
    value.to_degrees()
}

pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."))
}

pub fn default_urdf_path() -> PathBuf {
    repo_root().join(DEFAULT_URDF_REL)
}

pub fn parse_urdf_path(args: &[String]) -> PathBuf {
    arg_value(args, "--urdf")
        .map(PathBuf::from)
        .or_else(|| load_arm_config(args).ok().and_then(|cfg| cfg.urdf_path))
        .unwrap_or_else(default_urdf_path)
}

pub struct TemporaryUrdf {
    path: PathBuf,
}

impl TemporaryUrdf {
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TemporaryUrdf {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}

pub fn gravity_urdf_for_gripper(
    args: &[String],
    use_gripper: bool,
) -> Result<(PathBuf, Option<TemporaryUrdf>, f64), Box<dyn Error>> {
    let explicit_urdf = arg_value(args, "--urdf");
    let config_urdf = load_arm_config(args).ok().and_then(|cfg| cfg.urdf_path);
    let base_urdf = explicit_urdf
        .as_ref()
        .map(PathBuf::from)
        .or(config_urdf)
        .unwrap_or_else(default_urdf_path);
    let has_explicit_or_config_urdf = explicit_urdf.is_some() || parse_config_path(args).is_some();
    let scale = if let Some(value) = arg_value(args, "--end-link-load-scale") {
        value.parse::<f64>()?
    } else if use_gripper {
        if has_explicit_or_config_urdf {
            1.0
        } else {
            END_LINK_LOAD_SCALE_WITH_GRIPPER
        }
    } else {
        0.0
    };
    if (scale - 1.0).abs() <= f64::EPSILON || !has_end_link_inertial(&base_urdf)? {
        return Ok((base_urdf, None, scale));
    }
    let temp = end_link_load_urdf(&base_urdf, scale)?;
    Ok((temp.path().to_path_buf(), Some(temp), scale))
}

fn has_end_link_inertial(urdf_path: &Path) -> Result<bool, Box<dyn Error>> {
    let xml = fs::read_to_string(urdf_path)?;
    let Some(end_link_pos) = xml
        .find("name=\"end_link\"")
        .or_else(|| xml.find("name='end_link'"))
    else {
        return Ok(false);
    };
    Ok(xml[end_link_pos..].find("<inertial").is_some())
}

fn end_link_load_urdf(urdf_path: &Path, scale: f64) -> Result<TemporaryUrdf, Box<dyn Error>> {
    if scale < 0.0 {
        return Err("end_link load scale must be >= 0".into());
    }
    let xml = fs::read_to_string(urdf_path)?;
    let modified = scale_end_link_inertial(&xml, scale)?;
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let path = env::temp_dir().join(format!(
        "rebotarm_control_rt_end_link_{}_{}.urdf",
        process::id(),
        now_ns
    ));
    fs::write(&path, modified)?;
    Ok(TemporaryUrdf { path })
}

fn scale_end_link_inertial(xml: &str, scale: f64) -> Result<String, Box<dyn Error>> {
    let end_link_pos = xml
        .find("name=\"end_link\"")
        .or_else(|| xml.find("name='end_link'"))
        .ok_or("URDF does not contain link name=\"end_link\"")?;
    let inertial_start = end_link_pos
        + xml[end_link_pos..]
            .find("<inertial")
            .ok_or("URDF end_link does not contain an inertial block")?;
    let inertial_open_end = inertial_start
        + xml[inertial_start..]
            .find('>')
            .ok_or("URDF end_link inertial block is malformed")?
        + 1;
    let inertial_end = inertial_open_end
        + xml[inertial_open_end..]
            .find("</inertial>")
            .ok_or("URDF end_link inertial block is not closed")?
        + "</inertial>".len();

    let mut out = String::with_capacity(xml.len());
    out.push_str(&xml[..inertial_start]);
    if scale > 0.0 {
        let block = &xml[inertial_start..inertial_end];
        out.push_str(&scale_inertial_block(block, scale)?);
    }
    out.push_str(&xml[inertial_end..]);
    Ok(out)
}

fn scale_inertial_block(block: &str, scale: f64) -> Result<String, Box<dyn Error>> {
    let mut out = scale_attr_once(block, "mass", "value", scale)?;
    for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"] {
        out = scale_attr_once(&out, "inertia", attr, scale)?;
    }
    Ok(out)
}

fn scale_attr_once(
    source: &str,
    element: &str,
    attr: &str,
    scale: f64,
) -> Result<String, Box<dyn Error>> {
    let elem_start = find_xml_element(source, element)
        .ok_or_else(|| format!("URDF inertial block is missing <{element}>"))?;
    let elem_end = elem_start
        + source[elem_start..]
            .find('>')
            .ok_or_else(|| format!("URDF <{element}> tag is malformed"))?;
    let tag = &source[elem_start..elem_end];
    let (attr_offset, quote, pattern_len) = if let Some(offset) = tag.find(&format!("{attr}=\"")) {
        (offset, '"', attr.len() + 2)
    } else if let Some(offset) = tag.find(&format!("{attr}='")) {
        (offset, '\'', attr.len() + 2)
    } else {
        return Err(format!("URDF <{element}> tag is missing {attr}").into());
    };
    let value_start = elem_start + attr_offset + pattern_len;
    let value_end = value_start
        + source[value_start..]
            .find(quote)
            .ok_or_else(|| format!("URDF <{element}> {attr} quote is not closed"))?;
    let value = source[value_start..value_end].parse::<f64>()?;
    let replacement = format_float(value * scale);

    let mut out = String::with_capacity(source.len() + replacement.len());
    out.push_str(&source[..value_start]);
    out.push_str(&replacement);
    out.push_str(&source[value_end..]);
    Ok(out)
}

fn find_xml_element(source: &str, element: &str) -> Option<usize> {
    let needle = format!("<{element}");
    let mut offset = 0;
    while let Some(rel) = source[offset..].find(&needle) {
        let pos = offset + rel;
        let next = source[pos + needle.len()..].chars().next();
        if matches!(next, Some(' ' | '\t' | '\r' | '\n' | '/' | '>')) {
            return Some(pos);
        }
        offset = pos + needle.len();
    }
    None
}

fn format_float(value: f64) -> String {
    let mut text = format!("{value:.10}");
    while text.contains('.') && text.ends_with('0') {
        text.pop();
    }
    if text.ends_with('.') {
        text.push('0');
    }
    text
}

enum ExampleController {
    Damiao(DamiaoController),
    Robstride(RobstrideController),
}

impl ExampleController {
    fn open(port: &str, vendor: &str) -> Result<Self, Box<dyn Error>> {
        let lower = port.to_ascii_lowercase();
        match vendor {
            "damiao" => {
                if port.starts_with("/dev/") || lower.starts_with("com") {
                    Ok(Self::Damiao(DamiaoController::new_dm_serial(port, 921_600)?))
                } else {
                    Ok(Self::Damiao(DamiaoController::new_socketcan(port)?))
                }
            }
            "robstride" => {
                if port.starts_with("/dev/") || lower.starts_with("com") {
                    Err(format!("RobStride requires a CAN channel, got {port}").into())
                } else {
                    Ok(Self::Robstride(RobstrideController::new_socketcan(port)?))
                }
            }
            other => Err(format!("unsupported example vendor: {other}").into()),
        }
    }

    fn add_motor(&self, joint: &JointSpec) -> Result<ExampleMotor, Box<dyn Error>> {
        match self {
            ExampleController::Damiao(controller) => Ok(ExampleMotor::Damiao(controller.add_motor(
                joint.motor_id,
                joint.feedback_id,
                joint.model,
            )?)),
            ExampleController::Robstride(controller) => Ok(ExampleMotor::Robstride(
                controller.add_motor(joint.motor_id, joint.feedback_id, joint.model)?,
            )),
        }
    }

    fn enable_all(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleController::Damiao(controller) => Ok(controller.enable_all()?),
            ExampleController::Robstride(controller) => Ok(controller.enable_all()?),
        }
    }

    fn disable_all(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleController::Damiao(controller) => Ok(controller.disable_all()?),
            ExampleController::Robstride(controller) => Ok(controller.disable_all()?),
        }
    }

    fn poll_feedback_once(&self) {
        match self {
            ExampleController::Damiao(controller) => {
                let _ = controller.poll_feedback_once();
            }
            ExampleController::Robstride(controller) => {
                let _ = controller.poll_feedback_once();
            }
        }
    }

    fn shutdown(&self) {
        match self {
            ExampleController::Damiao(controller) => {
                let _ = controller.shutdown();
            }
            ExampleController::Robstride(controller) => {
                let _ = controller.shutdown();
            }
        }
    }

    fn close_bus(&self) {
        match self {
            ExampleController::Damiao(controller) => {
                let _ = controller.close_bus();
            }
            ExampleController::Robstride(controller) => {
                let _ = controller.close_bus();
            }
        }
    }
}

#[derive(Clone)]
pub enum ExampleMotor {
    Damiao(Arc<DamiaoMotor>),
    Robstride(Arc<RobstrideMotor>),
}

impl ExampleMotor {
    pub fn vendor(&self) -> &'static str {
        match self {
            ExampleMotor::Damiao(_) => "damiao",
            ExampleMotor::Robstride(_) => "robstride",
        }
    }

    fn clear_error(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.clear_error()?),
            ExampleMotor::Robstride(motor) => Ok(motor.clear_error()?),
        }
    }

    pub fn disable(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.disable()?),
            ExampleMotor::Robstride(motor) => Ok(motor.disable()?),
        }
    }

    pub fn set_zero_position(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.set_zero_position()?),
            ExampleMotor::Robstride(motor) => Ok(motor.set_zero_position()?),
        }
    }

    pub fn ensure_control_mode(
        &self,
        mode: ControlMode,
        timeout: Duration,
    ) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => {
                let mode = match mode {
                    ControlMode::Mit => DamiaoMode::Mit,
                    ControlMode::PosVel => DamiaoMode::PosVel,
                    ControlMode::Vel => DamiaoMode::Vel,
                    ControlMode::ForcePos => DamiaoMode::ForcePos,
                };
                Ok(motor.ensure_control_mode(mode, timeout)?)
            }
            ExampleMotor::Robstride(motor) => {
                let mode = match mode {
                    ControlMode::Mit => RobstrideMode::Mit,
                    ControlMode::PosVel => RobstrideMode::Position,
                    ControlMode::Vel => RobstrideMode::Velocity,
                    ControlMode::ForcePos => {
                        return Err("RobStride does not support FORCE_POS mode".into())
                    }
                };
                Ok(motor.ensure_control_mode(mode, timeout)?)
            }
        }
    }

    pub fn request_motor_feedback(&self) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.request_motor_feedback()?),
            ExampleMotor::Robstride(_) => Ok(()),
        }
    }

    pub fn latest_state(&self) -> Option<MotorFeedbackState> {
        match self {
            ExampleMotor::Damiao(motor) => motor.latest_state().map(|s| MotorFeedbackState {
                status_code: s.status_code,
                pos: s.pos,
                vel: s.vel,
                torq: s.torq,
            }),
            ExampleMotor::Robstride(motor) => motor.latest_state().map(|s| MotorFeedbackState {
                status_code: u8::from(s.undervoltage)
                    | (u8::from(s.overcurrent) << 1)
                    | (u8::from(s.overtemperature) << 2)
                    | (u8::from(s.magnetic_encoder_fault) << 3)
                    | (u8::from(s.stall) << 4)
                    | (u8::from(s.uncalibrated) << 5),
                pos: s.position,
                vel: s.velocity,
                torq: s.torque,
            }),
        }
    }

    pub fn send_cmd_mit(
        &self,
        pos: f32,
        vel: f32,
        kp: f32,
        kd: f32,
        tau: f32,
    ) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.send_cmd_mit(pos, vel, kp, kd, tau)?),
            ExampleMotor::Robstride(motor) => Ok(motor.send_cmd_mit(pos, vel, kp, kd, tau)?),
        }
    }

    pub fn send_cmd_pos_vel(&self, pos: f32, vlim: f32) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.send_cmd_pos_vel(pos, vlim)?),
            ExampleMotor::Robstride(motor) => {
                motor.set_mode(RobstrideMode::Position)?;
                let v = vlim.abs();
                if v.is_finite() && v > 0.0 {
                    motor.write_parameter(0x7017, RobstrideParameterValue::F32(v))?;
                }
                motor.write_parameter(0x7016, RobstrideParameterValue::F32(pos))?;
                Ok(())
            }
        }
    }

    pub fn send_cmd_vel(&self, vel: f32) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.send_cmd_vel(vel)?),
            ExampleMotor::Robstride(motor) => {
                motor.set_mode(RobstrideMode::Velocity)?;
                motor.set_velocity_target(vel)?;
                Ok(())
            }
        }
    }

    pub fn send_cmd_force_pos(
        &self,
        pos: f32,
        vlim: f32,
        ratio: f32,
    ) -> Result<(), Box<dyn Error>> {
        match self {
            ExampleMotor::Damiao(motor) => Ok(motor.send_cmd_force_pos(pos, vlim, ratio)?),
            ExampleMotor::Robstride(_) => Err("RobStride does not support FORCE_POS commands".into()),
        }
    }

    fn set_active_report(&self, enabled: bool) {
        if let ExampleMotor::Robstride(motor) = self {
            let _ = motor.set_active_report(enabled);
        }
    }
}

pub fn open_controller(port: &str) -> Result<DamiaoController, Box<dyn Error>> {
    let lower = port.to_ascii_lowercase();
    if port.starts_with("/dev/") || lower.starts_with("com") {
        Ok(DamiaoController::new_dm_serial(port, 921_600)?)
    } else {
        Ok(DamiaoController::new_socketcan(port)?)
    }
}

pub struct B601Arm {
    pub port: String,
    controller: ExampleController,
    pub motors: Vec<ExampleMotor>,
    pub joints: Vec<JointSpec>,
}

impl B601Arm {
    pub fn open(port: &str) -> Result<Self, Box<dyn Error>> {
        let args: Vec<String> = env::args().skip(1).collect();
        Self::open_with_args(port, &args)
    }

    pub fn open_with_args(port: &str, args: &[String]) -> Result<Self, Box<dyn Error>> {
        let joints = arm_joints(args)?;
        let vendor = joints
            .first()
            .map(|joint| joint.vendor)
            .ok_or("empty joint config")?;
        if joints.iter().any(|joint| joint.vendor != vendor) {
            return Err("Rust examples currently expect a single vendor per arm config".into());
        }
        let controller = ExampleController::open(port, vendor)?;
        let mut motors = Vec::with_capacity(joints.len());
        for joint in &joints {
            motors.push(controller.add_motor(joint)?);
        }
        for motor in &motors {
            motor.set_active_report(true);
        }
        Ok(Self {
            port: port.to_string(),
            controller,
            motors,
            joints,
        })
    }

    pub fn enable(&self) -> Result<(), Box<dyn Error>> {
        for motor in &self.motors {
            let _ = motor.clear_error();
        }
        self.controller.enable_all()?;
        Ok(())
    }

    pub fn disable(&self) -> Result<(), Box<dyn Error>> {
        self.controller.disable_all()?;
        Ok(())
    }

    pub fn close(&self) {
        let _ = self.controller.disable_all();
        thread::sleep(Duration::from_millis(20));
        self.controller.shutdown();
        self.controller.close_bus();
    }

    pub fn ensure_all_mode(&self, mode: ControlMode) {
        let timeout = Duration::from_millis(300);
        for (joint, motor) in self.joints.iter().zip(&self.motors) {
            if let Err(err) = motor.ensure_control_mode(mode, timeout) {
                eprintln!("warning: {} mode switch failed: {err}", joint.name);
            }
        }
    }

    pub fn ensure_arm_mode(&self, mode: ControlMode) {
        let timeout = Duration::from_millis(300);
        for (joint, motor) in self.joints.iter().zip(&self.motors).take(ARM_DOF) {
            if let Err(err) = motor.ensure_control_mode(mode, timeout) {
                eprintln!("warning: {} mode switch failed: {err}", joint.name);
            }
        }
    }

    pub fn request_feedback(&self) {
        for motor in &self.motors {
            let _ = motor.request_motor_feedback();
        }
        self.controller.poll_feedback_once();
        thread::sleep(Duration::from_millis(20));
        self.controller.poll_feedback_once();
    }

    pub fn states(&self) -> Vec<Option<MotorFeedbackState>> {
        self.request_feedback();
        self.motors
            .iter()
            .map(|motor| motor.latest_state())
            .collect()
    }

    pub fn positions_or_zero(&self) -> Vec<f32> {
        let mut out: Vec<f32> = self
            .states()
            .into_iter()
            .map(|state| state.map(|s| s.pos).unwrap_or(0.0))
            .collect();
        while out.len() < ALL_DOF {
            out.push(0.0);
        }
        out
    }

    pub fn arm_positions_or_zero(&self) -> [f64; ARM_DOF] {
        let mut q = [0.0_f64; ARM_DOF];
        for (idx, value) in self
            .positions_or_zero()
            .into_iter()
            .take(ARM_DOF)
            .enumerate()
        {
            q[idx] = value as f64;
        }
        q
    }

    pub fn print_state(&self) {
        for (joint, state) in self.joints.iter().zip(self.states()) {
            match state {
                Some(s) => println!(
                    "{:<14} pos={:>8.2} deg vel={:>8.2} deg/s torque={:>8.3} status={}",
                    joint.name,
                    rad_to_deg_f32(s.pos),
                    rad_to_deg_f32(s.vel),
                    s.torq,
                    s.status_code
                ),
                None => println!("{:<14} no feedback", joint.name),
            }
        }
    }

    pub fn send_mit_all(
        &self,
        pos: &[f32],
        vel: &[f32],
        kp: &[f32],
        kd: &[f32],
        tau: &[f32],
    ) -> Result<(), Box<dyn Error>> {
        for idx in 0..self.motors.len() {
            self.motors[idx].send_cmd_mit(
                pos.get(idx).copied().unwrap_or(0.0),
                vel.get(idx).copied().unwrap_or(0.0),
                kp.get(idx).copied().unwrap_or(self.joints[idx].mit_kp),
                kd.get(idx).copied().unwrap_or(self.joints[idx].mit_kd),
                tau.get(idx).copied().unwrap_or(0.0),
            )?;
        }
        Ok(())
    }

    pub fn send_pos_vel_all(&self, pos: &[f32], vlim: &[f32]) -> Result<(), Box<dyn Error>> {
        for idx in 0..self.motors.len() {
            self.motors[idx].send_cmd_pos_vel(
                pos.get(idx).copied().unwrap_or(0.0),
                vlim.get(idx).copied().unwrap_or(self.joints[idx].vlim),
            )?;
        }
        Ok(())
    }
}

pub fn sleep_to_rate(start: Instant, rate_hz: f64) {
    let period = Duration::from_secs_f64(1.0 / rate_hz.max(1.0));
    if let Some(remaining) = period.checked_sub(start.elapsed()) {
        thread::sleep(remaining);
    }
}

pub fn default_kp() -> Vec<f32> {
    current_joints_or_default()
        .iter()
        .map(|joint| joint.mit_kp)
        .collect()
}

pub fn default_kd() -> Vec<f32> {
    current_joints_or_default()
        .iter()
        .map(|joint| joint.mit_kd)
        .collect()
}

pub fn default_vlim() -> Vec<f32> {
    current_joints_or_default()
        .iter()
        .map(|joint| joint.vlim)
        .collect()
}

pub fn move_pos_vel_path(
    arm: &B601Arm,
    start: &[f32],
    end: &[f32],
    duration_s: f64,
    rate_hz: f64,
) -> Result<(), Box<dyn Error>> {
    let steps = (duration_s.max(0.02) * rate_hz.max(1.0)).ceil() as usize;
    let vlim = default_vlim();
    for step in 1..=steps {
        let tick = Instant::now();
        let alpha = step as f32 / steps as f32;
        let mut target = vec![0.0_f32; ALL_DOF];
        for idx in 0..ALL_DOF {
            let s = start.get(idx).copied().unwrap_or(0.0);
            let e = end.get(idx).copied().unwrap_or(s);
            target[idx] = s + (e - s) * alpha;
        }
        arm.send_pos_vel_all(&target, &vlim)?;
        sleep_to_rate(tick, rate_hz);
    }
    Ok(())
}

pub fn run_single_motor_console() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!("Usage: cargo run --bin 0x01damiao_test -- --port /dev/ttyACM0 --joint 0");
        return Ok(());
    }

    let port = parse_port(&args);
    let joint_arg = arg_value(&args, "--joint")
        .or_else(|| arg_value(&args, "-j"))
        .unwrap_or_else(|| "0".to_string());
    let joint_idx = parse_joint(&joint_arg)?;
    let arm = B601Arm::open(&port)?;
    let joint = arm
        .joints
        .get(joint_idx)
        .ok_or_else(|| format!("joint index {joint_idx} out of range"))?;

    println!("connected: B601 on {}", arm.port);
    println!("joint: {} ({})", joint_idx, joint.name);
    println!(
        "commands: enable / disable / set_zero / mode / mit / posvel / vel / forcepos / state / q"
    );
    println!("examples: mit 10 0 20 2 0 | posvel 10 1.0 | vel 0.2 | forcepos -120 3.0 0.05");

    let mut target = arm.positions_or_zero();
    arm.enable()?;
    arm.ensure_all_mode(ControlMode::Mit);

    loop {
        let Some(line) = prompt("> ")? else {
            break;
        };
        if line.is_empty() {
            continue;
        }
        let lower = line.to_ascii_lowercase();
        if matches!(lower.as_str(), "q" | "quit" | "exit") {
            break;
        }
        if lower == "enable" {
            arm.enable()?;
            println!("enabled");
            continue;
        }
        if lower == "disable" {
            arm.disable()?;
            println!("disabled");
            continue;
        }
        if lower == "state" {
            arm.print_state();
            continue;
        }
        if lower == "set_zero" {
            println!("set zero requires disabled motor. Type YES to continue.");
            if prompt("confirm> ")?.as_deref() == Some("YES") {
                arm.motors[joint_idx].disable()?;
                arm.motors[joint_idx].set_zero_position()?;
                println!("zero set for {}", joint.name);
            }
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.first().copied() {
            Some("mode") if parts.len() >= 2 => {
                match parts[1] {
                    "mit" => arm.motors[joint_idx]
                        .ensure_control_mode(ControlMode::Mit, Duration::from_millis(300))?,
                    "posvel" | "pos_vel" => arm.motors[joint_idx]
                        .ensure_control_mode(ControlMode::PosVel, Duration::from_millis(300))?,
                    "vel" => arm.motors[joint_idx]
                        .ensure_control_mode(ControlMode::Vel, Duration::from_millis(300))?,
                    "forcepos" | "force_pos" => arm.motors[joint_idx]
                        .ensure_control_mode(ControlMode::ForcePos, Duration::from_millis(300))?,
                    other => {
                        println!("unknown mode: {other}");
                        continue;
                    }
                }
                println!("mode set for {}", joint.name);
            }
            Some("mit") if parts.len() >= 2 => {
                let pos = deg_to_rad_f32(parts[1].parse()?);
                let vel = parts.get(2).and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let kp = parts
                    .get(3)
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(joint.mit_kp);
                let kd = parts
                    .get(4)
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(joint.mit_kd);
                let tau = parts.get(5).and_then(|v| v.parse().ok()).unwrap_or(0.0);
                target[joint_idx] = pos;
                arm.motors[joint_idx].send_cmd_mit(pos, vel, kp, kd, tau)?;
                println!("sent MIT target {:.2} deg", parts[1].parse::<f64>()?);
            }
            Some("posvel") | Some("pos_vel") if parts.len() >= 2 => {
                let pos = deg_to_rad_f32(parts[1].parse()?);
                let vlim = parts
                    .get(2)
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(joint.vlim);
                target[joint_idx] = pos;
                arm.motors[joint_idx].send_cmd_pos_vel(pos, vlim)?;
                println!(
                    "sent POS_VEL target {:.2} deg, vlim={vlim:.3}",
                    parts[1].parse::<f64>()?
                );
            }
            Some("vel") if parts.len() >= 2 => {
                let vel = parts[1].parse::<f32>()?;
                arm.motors[joint_idx].send_cmd_vel(vel)?;
                println!("sent velocity {vel:.3} rad/s");
            }
            Some("forcepos") | Some("force_pos") if parts.len() >= 2 => {
                let pos = deg_to_rad_f32(parts[1].parse()?);
                let vlim = parts
                    .get(2)
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(joint.vlim);
                let ratio = parts.get(3).and_then(|v| v.parse().ok()).unwrap_or(0.05);
                target[joint_idx] = pos;
                arm.motors[joint_idx].send_cmd_force_pos(pos, vlim, ratio)?;
                println!(
                    "sent FORCE_POS target {:.2} deg, vlim={vlim:.3}, ratio={ratio:.3}",
                    parts[1].parse::<f64>()?
                );
            }
            _ => println!("unknown command"),
        }
    }

    arm.close();
    Ok(())
}

pub type Mat4 = [[f64; 4]; 4];

fn eye() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn mat_mul(a: Mat4, b: Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn translation(x: f64, y: f64, z: f64) -> Mat4 {
    let mut t = eye();
    t[0][3] = x;
    t[1][3] = y;
    t[2][3] = z;
    t
}

fn rot_x(a: f64) -> Mat4 {
    let (s, c) = a.sin_cos();
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn rot_y(a: f64) -> Mat4 {
    let (s, c) = a.sin_cos();
    [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn rot_z(a: f64) -> Mat4 {
    let (s, c) = a.sin_cos();
    [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn rpy(roll: f64, pitch: f64, yaw: f64) -> Mat4 {
    mat_mul(mat_mul(rot_z(yaw), rot_y(pitch)), rot_x(roll))
}

fn axis_angle(axis: [f64; 3], angle: f64) -> Mat4 {
    let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if norm <= f64::EPSILON {
        return eye();
    }
    let x = axis[0] / norm;
    let y = axis[1] / norm;
    let z = axis[2] / norm;
    let (s, c) = angle.sin_cos();
    let v = 1.0 - c;
    [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0.0],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s, 0.0],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

const JOINT_ORIGINS: [([f64; 3], [f64; 3], [f64; 3]); ARM_DOF] = [
    (
        [-0.000_084_16, 0.0, 0.084_65],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ),
    (
        [0.020_084, 0.031_625, 0.055_55],
        [-1.5708, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ),
    ([-0.264, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    (
        [0.2426, -0.054, -0.001_625],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ),
    (
        [0.078_308, -0.0375, -0.03],
        [-1.5708, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ),
    ([0.028_008, 0.0, 0.04], [0.0, 1.5708, 0.0], [0.0, 0.0, 1.0]),
];

pub fn fk(q: &[f64; ARM_DOF]) -> Mat4 {
    let mut t = eye();
    for idx in 0..ARM_DOF {
        let (xyz, rpy0, axis) = JOINT_ORIGINS[idx];
        t = mat_mul(t, translation(xyz[0], xyz[1], xyz[2]));
        t = mat_mul(t, rpy(rpy0[0], rpy0[1], rpy0[2]));
        t = mat_mul(t, axis_angle(axis, q[idx]));
    }
    t = mat_mul(t, translation(0.0, 0.0, 0.155_39));
    t = mat_mul(t, rpy(0.0, -1.5708, PI));
    t
}

pub fn pose_xyz(t: &Mat4) -> [f64; 3] {
    [t[0][3], t[1][3], t[2][3]]
}

pub fn pose_rpy(t: &Mat4) -> [f64; 3] {
    let sy = (-t[2][0]).clamp(-1.0, 1.0);
    let pitch = sy.asin();
    let cp = pitch.cos();
    if cp.abs() > 1e-8 {
        [t[2][1].atan2(t[2][2]), pitch, t[1][0].atan2(t[0][0])]
    } else {
        [0.0, pitch, (-t[0][1]).atan2(t[1][1])]
    }
}

pub struct IkResult {
    pub q: [f64; ARM_DOF],
    pub error: f64,
    pub iterations: usize,
    pub converged: bool,
}

pub fn solve_ik_position(target_xyz: [f64; 3], seed: [f64; ARM_DOF], max_iter: usize) -> IkResult {
    let mut q = seed;
    clamp_q(&mut q);
    let eps = 1e-4;
    let mut err_norm = f64::INFINITY;
    let mut iter_done = 0;

    for iter in 0..max_iter {
        iter_done = iter + 1;
        let current = pose_xyz(&fk(&q));
        let err = [
            target_xyz[0] - current[0],
            target_xyz[1] - current[1],
            target_xyz[2] - current[2],
        ];
        err_norm = (err[0] * err[0] + err[1] * err[1] + err[2] * err[2]).sqrt();
        if err_norm < 1e-4 {
            return IkResult {
                q,
                error: err_norm,
                iterations: iter_done,
                converged: true,
            };
        }

        let mut grad = [0.0_f64; ARM_DOF];
        for j in 0..ARM_DOF {
            let mut q2 = q;
            q2[j] += eps;
            let p2 = pose_xyz(&fk(&q2));
            let jac = [
                (p2[0] - current[0]) / eps,
                (p2[1] - current[1]) / eps,
                (p2[2] - current[2]) / eps,
            ];
            grad[j] = jac[0] * err[0] + jac[1] * err[1] + jac[2] * err[2];
        }

        for j in 0..ARM_DOF {
            let step = (0.8 * grad[j]).clamp(-0.08, 0.08);
            q[j] += step;
        }
        clamp_q(&mut q);
    }

    IkResult {
        q,
        error: err_norm,
        iterations: iter_done,
        converged: false,
    }
}

pub fn clamp_q(q: &mut [f64; ARM_DOF]) {
    for idx in 0..ARM_DOF {
        q[idx] = q[idx].clamp(ARM_LIMITS_RAD[idx].0, ARM_LIMITS_RAD[idx].1);
    }
}

pub fn q_deg(q: &[f64; ARM_DOF]) -> [f64; ARM_DOF] {
    let mut out = [0.0; ARM_DOF];
    for idx in 0..ARM_DOF {
        out[idx] = q[idx].to_degrees();
    }
    out
}

pub fn q_rad_from_deg(values: &[f64]) -> [f64; ARM_DOF] {
    let mut out = [0.0; ARM_DOF];
    for idx in 0..ARM_DOF.min(values.len()) {
        out[idx] = values[idx].to_radians();
    }
    out
}

pub fn print_pose(q: &[f64; ARM_DOF]) {
    let t = fk(q);
    let xyz = pose_xyz(&t);
    let rpy0 = pose_rpy(&t);
    println!(
        "  ee position: [{:+.4}, {:+.4}, {:+.4}] m",
        xyz[0], xyz[1], xyz[2]
    );
    println!(
        "  ee rpy:      [{:+.2}, {:+.2}, {:+.2}] deg",
        rpy0[0].to_degrees(),
        rpy0[1].to_degrees(),
        rpy0[2].to_degrees()
    );
}

pub fn approx_gravity_torque(q: &[f32], scale: f32) -> Vec<f32> {
    let mut tau = vec![0.0_f32; ALL_DOF];
    if q.len() >= 4 {
        tau[1] = -2.5 * scale * q[1].sin();
        tau[2] = -1.6 * scale * (q[1] + q[2]).sin();
        tau[3] = -0.4 * scale * (q[1] + q[2] + q[3]).sin();
    }
    tau
}

type ModelNewFn = unsafe extern "C" fn(*const c_char) -> *mut c_void;
type ModelFreeFn = unsafe extern "C" fn(*mut c_void);
type ModelNqFn = unsafe extern "C" fn(*const c_void) -> c_int;
type EndFrameIdFn = unsafe extern "C" fn(*const c_void) -> c_int;
type NeutralFn = unsafe extern "C" fn(*const c_void, *mut c_double, c_int) -> c_int;
type FkFn = unsafe extern "C" fn(
    *const c_void,
    *const c_double,
    c_int,
    *const c_char,
    *mut c_double,
    *mut c_double,
    *mut c_double,
) -> c_int;
type IkFn = unsafe extern "C" fn(
    *const c_void,
    *const c_double,
    *const c_double,
    c_int,
    c_int,
    c_int,
    c_double,
    c_double,
    c_double,
    *mut c_double,
    *mut CIkResult,
) -> c_int;
type GravityFn =
    unsafe extern "C" fn(*const c_void, *const c_double, c_int, *mut c_double, c_int) -> c_int;
type InverseDynamicsFn = unsafe extern "C" fn(
    *const c_void,
    *const c_double,
    c_int,
    *const c_double,
    c_int,
    *const c_double,
    c_int,
    *mut c_double,
    c_int,
) -> c_int;
type NumDynamicParamsFn = unsafe extern "C" fn(*const c_void) -> c_int;
type NumTotalParamsFn = unsafe extern "C" fn(*const c_void, c_int) -> c_int;
type ModelDynamicParamsFn = unsafe extern "C" fn(*const c_void, *mut c_double, c_int) -> c_int;
type BuildRegressionFn = unsafe extern "C" fn(
    *const c_void,
    *const c_double,
    *const c_double,
    *const c_double,
    c_int,
    c_int,
    c_int,
    c_double,
    *mut c_double,
    c_int,
    c_int,
) -> c_int;
type StackTauFn =
    unsafe extern "C" fn(*const c_double, c_int, c_int, *mut c_double, c_int) -> c_int;

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct CLsInfo {
    pub rank: c_int,
    pub condition: c_double,
    pub residual_norm: c_double,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct CMetrics {
    pub rmse: c_double,
    pub mae: c_double,
    pub max_abs: c_double,
    pub r2: c_double,
}

type FitLeastSquaresFn = unsafe extern "C" fn(
    *const c_double,
    c_int,
    c_int,
    *const c_double,
    c_int,
    c_double,
    *mut c_double,
    c_int,
    *mut c_double,
    c_int,
    *mut CLsInfo,
) -> c_int;
type FitBaseQrFn = unsafe extern "C" fn(
    *const c_double,
    c_int,
    c_int,
    *const c_double,
    c_int,
    c_double,
    *mut c_double,
    c_int,
    *mut c_int,
    c_int,
    *mut c_double,
    c_int,
    *mut CLsInfo,
) -> c_int;
type RegressionMetricsFn = unsafe extern "C" fn(
    *const c_double,
    *const c_double,
    c_int,
    c_int,
    *mut CMetrics,
    *mut c_double,
    *mut c_double,
    c_int,
) -> c_int;
type LastErrorFn = unsafe extern "C" fn() -> *const c_char;

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct CIkResult {
    success: c_int,
    error: c_double,
    iterations: c_int,
}

struct MathApi {
    _lib: Library,
    model_new: ModelNewFn,
    model_free: ModelFreeFn,
    model_nq: ModelNqFn,
    end_frame_id: EndFrameIdFn,
    neutral: NeutralFn,
    fk: FkFn,
    ik: IkFn,
    gravity: GravityFn,
    inverse_dynamics: InverseDynamicsFn,
    num_dynamic_params: NumDynamicParamsFn,
    num_total_params: NumTotalParamsFn,
    model_dynamic_params: ModelDynamicParamsFn,
    build_regression: BuildRegressionFn,
    stack_tau: StackTauFn,
    fit_ls: FitLeastSquaresFn,
    fit_base_qr: FitBaseQrFn,
    regression_metrics: RegressionMetricsFn,
    last_error: LastErrorFn,
}

impl MathApi {
    fn load() -> Result<Arc<Self>, Box<dyn Error>> {
        let lib_path = math_lib_path();
        let lib = unsafe { Library::new(&lib_path)? };
        unsafe {
            let model_new = *lib.get::<ModelNewFn>(b"rebotarm_math_model_new\0")?;
            let model_free = *lib.get::<ModelFreeFn>(b"rebotarm_math_model_free\0")?;
            let model_nq = *lib.get::<ModelNqFn>(b"rebotarm_math_model_nq\0")?;
            let end_frame_id = *lib.get::<EndFrameIdFn>(b"rebotarm_math_end_frame_id\0")?;
            let neutral = *lib.get::<NeutralFn>(b"rebotarm_math_neutral\0")?;
            let fk = *lib.get::<FkFn>(b"rebotarm_math_fk\0")?;
            let ik = *lib.get::<IkFn>(b"rebotarm_math_ik\0")?;
            let gravity = *lib.get::<GravityFn>(b"rebotarm_math_generalized_gravity\0")?;
            let inverse_dynamics =
                *lib.get::<InverseDynamicsFn>(b"rebotarm_math_inverse_dynamics\0")?;
            let num_dynamic_params =
                *lib.get::<NumDynamicParamsFn>(b"rebotarm_math_num_dynamic_parameters\0")?;
            let num_total_params =
                *lib.get::<NumTotalParamsFn>(b"rebotarm_math_num_total_parameters\0")?;
            let model_dynamic_params =
                *lib.get::<ModelDynamicParamsFn>(b"rebotarm_math_model_dynamic_parameters\0")?;
            let build_regression =
                *lib.get::<BuildRegressionFn>(b"rebotarm_math_build_regression_matrix\0")?;
            let stack_tau = *lib.get::<StackTauFn>(b"rebotarm_math_stack_tau_samples\0")?;
            let fit_ls = *lib.get::<FitLeastSquaresFn>(b"rebotarm_math_fit_least_squares\0")?;
            let fit_base_qr = *lib.get::<FitBaseQrFn>(b"rebotarm_math_fit_base_parameters_qr\0")?;
            let regression_metrics =
                *lib.get::<RegressionMetricsFn>(b"rebotarm_math_regression_metrics\0")?;
            let last_error = *lib.get::<LastErrorFn>(b"rebotarm_math_last_error\0")?;
            Ok(Arc::new(Self {
                _lib: lib,
                model_new,
                model_free,
                model_nq,
                end_frame_id,
                neutral,
                fk,
                ik,
                gravity,
                inverse_dynamics,
                num_dynamic_params,
                num_total_params,
                model_dynamic_params,
                build_regression,
                stack_tau,
                fit_ls,
                fit_base_qr,
                regression_metrics,
                last_error,
            }))
        }
    }

    fn last_error(&self) -> String {
        unsafe {
            let ptr = (self.last_error)();
            if ptr.is_null() {
                return "unknown C++ math error".to_string();
            }
            CStr::from_ptr(ptr).to_string_lossy().into_owned()
        }
    }
}

pub struct MathModel {
    api: Arc<MathApi>,
    handle: *mut c_void,
    pub nq: usize,
    pub end_frame_id: i32,
}

impl MathModel {
    pub fn load(urdf_path: &Path) -> Result<Self, Box<dyn Error>> {
        let api = MathApi::load()?;
        let urdf = CString::new(urdf_path.to_string_lossy().as_bytes())?;
        let handle = unsafe { (api.model_new)(urdf.as_ptr()) };
        if handle.is_null() {
            return Err(api.last_error().into());
        }
        let nq = unsafe { (api.model_nq)(handle) };
        if nq <= 0 {
            return Err("invalid nq returned by C++ math model".into());
        }
        let end_frame_id = unsafe { (api.end_frame_id)(handle) };
        if end_frame_id < 0 {
            return Err(api.last_error().into());
        }
        Ok(Self {
            api,
            handle,
            nq: nq as usize,
            end_frame_id,
        })
    }

    pub fn default() -> Result<Self, Box<dyn Error>> {
        Self::load(&default_urdf_path())
    }

    pub fn neutral(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let mut q = vec![0.0_f64; self.nq];
        let rc = unsafe { (self.api.neutral)(self.handle, q.as_mut_ptr(), q.len() as c_int) };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(q)
    }

    pub fn fk_cpp(&self, q: &[f64]) -> Result<Pose, Box<dyn Error>> {
        let frame = CString::new("")?;
        let mut xyz = [0.0_f64; 3];
        let mut rpy = [0.0_f64; 3];
        let mut raw_t = [0.0_f64; 16];
        let rc = unsafe {
            (self.api.fk)(
                self.handle,
                q.as_ptr(),
                q.len() as c_int,
                frame.as_ptr(),
                xyz.as_mut_ptr(),
                rpy.as_mut_ptr(),
                raw_t.as_mut_ptr(),
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(Pose {
            xyz,
            rpy,
            matrix: row_major_to_mat4(raw_t),
        })
    }

    pub fn ik_position_cpp(
        &self,
        target_xyz: [f64; 3],
        seed: &[f64],
        max_iter: usize,
    ) -> Result<IkResult, Box<dyn Error>> {
        let mut target = self.fk_cpp(seed)?.matrix;
        target[0][3] = target_xyz[0];
        target[1][3] = target_xyz[1];
        target[2][3] = target_xyz[2];
        self.ik_matrix_cpp(target, seed, max_iter)
    }

    pub fn ik_matrix_cpp(
        &self,
        target: Mat4,
        seed: &[f64],
        max_iter: usize,
    ) -> Result<IkResult, Box<dyn Error>> {
        let mut q = vec![0.0_f64; self.nq];
        let mut result = CIkResult::default();
        let raw_target = mat4_to_row_major(target);
        let rc = unsafe {
            (self.api.ik)(
                self.handle,
                raw_target.as_ptr(),
                seed.as_ptr(),
                seed.len() as c_int,
                self.end_frame_id as c_int,
                max_iter as c_int,
                1e-4,
                0.5,
                1e-6,
                q.as_mut_ptr(),
                &mut result,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(IkResult {
            q: vec_to_q6(&q),
            error: result.error,
            iterations: result.iterations.max(0) as usize,
            converged: result.success != 0,
        })
    }

    pub fn generalized_gravity_cpp(&self, q: &[f64]) -> Result<Vec<f64>, Box<dyn Error>> {
        let mut tau = vec![0.0_f64; self.nq];
        let rc = unsafe {
            (self.api.gravity)(
                self.handle,
                q.as_ptr(),
                q.len() as c_int,
                tau.as_mut_ptr(),
                tau.len() as c_int,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(tau)
    }

    pub fn inverse_dynamics_cpp(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
    ) -> Result<Vec<f64>, Box<dyn Error>> {
        let mut tau = vec![0.0_f64; self.nq];
        let rc = unsafe {
            (self.api.inverse_dynamics)(
                self.handle,
                q.as_ptr(),
                q.len() as c_int,
                dq.as_ptr(),
                dq.len() as c_int,
                ddq.as_ptr(),
                ddq.len() as c_int,
                tau.as_mut_ptr(),
                tau.len() as c_int,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(tau)
    }

    pub fn num_dynamic_parameters(&self) -> Result<usize, Box<dyn Error>> {
        let n = unsafe { (self.api.num_dynamic_params)(self.handle) };
        if n <= 0 {
            return Err(self.api.last_error().into());
        }
        Ok(n as usize)
    }

    pub fn num_total_parameters(&self, include_friction: bool) -> Result<usize, Box<dyn Error>> {
        let n = unsafe { (self.api.num_total_params)(self.handle, include_friction as c_int) };
        if n <= 0 {
            return Err(self.api.last_error().into());
        }
        Ok(n as usize)
    }

    pub fn model_dynamic_parameters(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let len = self.num_dynamic_parameters()?;
        let mut params = vec![0.0_f64; len];
        let rc = unsafe {
            (self.api.model_dynamic_params)(self.handle, params.as_mut_ptr(), params.len() as c_int)
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(params)
    }

    pub fn build_regression_matrix(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        samples: usize,
        include_friction: bool,
        coulomb_eps: f64,
    ) -> Result<Vec<f64>, Box<dyn Error>> {
        let cols = self.num_total_parameters(include_friction)?;
        let rows = samples * self.nq;
        let mut y = vec![0.0_f64; rows * cols];
        let rc = unsafe {
            (self.api.build_regression)(
                self.handle,
                q.as_ptr(),
                dq.as_ptr(),
                ddq.as_ptr(),
                samples as c_int,
                self.nq as c_int,
                include_friction as c_int,
                coulomb_eps,
                y.as_mut_ptr(),
                rows as c_int,
                cols as c_int,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(y)
    }

    pub fn stack_tau_samples(
        &self,
        tau_samples: &[f64],
        samples: usize,
    ) -> Result<Vec<f64>, Box<dyn Error>> {
        let mut tau = vec![0.0_f64; samples * self.nq];
        let rc = unsafe {
            (self.api.stack_tau)(
                tau_samples.as_ptr(),
                samples as c_int,
                self.nq as c_int,
                tau.as_mut_ptr(),
                tau.len() as c_int,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok(tau)
    }

    pub fn fit_least_squares(
        &self,
        y: &[f64],
        rows: usize,
        cols: usize,
        tau: &[f64],
        rcond: f64,
    ) -> Result<(Vec<f64>, Vec<f64>, CLsInfo), Box<dyn Error>> {
        let mut beta = vec![0.0_f64; cols];
        let mut tau_pred = vec![0.0_f64; rows];
        let mut info = CLsInfo::default();
        let rc = unsafe {
            (self.api.fit_ls)(
                y.as_ptr(),
                rows as c_int,
                cols as c_int,
                tau.as_ptr(),
                tau.len() as c_int,
                rcond,
                beta.as_mut_ptr(),
                beta.len() as c_int,
                tau_pred.as_mut_ptr(),
                tau_pred.len() as c_int,
                &mut info,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok((beta, tau_pred, info))
    }

    pub fn fit_base_qr(
        &self,
        y: &[f64],
        rows: usize,
        cols: usize,
        tau: &[f64],
        rcond: f64,
    ) -> Result<(Vec<f64>, Vec<i32>, Vec<f64>, CLsInfo), Box<dyn Error>> {
        let mut beta = vec![0.0_f64; cols];
        let mut selected = vec![0_i32; cols];
        let mut tau_pred = vec![0.0_f64; rows];
        let mut info = CLsInfo::default();
        let rank = unsafe {
            (self.api.fit_base_qr)(
                y.as_ptr(),
                rows as c_int,
                cols as c_int,
                tau.as_ptr(),
                tau.len() as c_int,
                rcond,
                beta.as_mut_ptr(),
                beta.len() as c_int,
                selected.as_mut_ptr(),
                selected.len() as c_int,
                tau_pred.as_mut_ptr(),
                tau_pred.len() as c_int,
                &mut info,
            )
        };
        if rank < 0 {
            return Err(self.api.last_error().into());
        }
        beta.truncate(rank as usize);
        selected.truncate(rank as usize);
        Ok((beta, selected, tau_pred, info))
    }

    pub fn regression_metrics(
        &self,
        tau: &[f64],
        tau_pred: &[f64],
    ) -> Result<(CMetrics, Vec<f64>, Vec<f64>), Box<dyn Error>> {
        let mut metrics = CMetrics::default();
        let mut rmse = vec![0.0_f64; self.nq];
        let mut mae = vec![0.0_f64; self.nq];
        let rc = unsafe {
            (self.api.regression_metrics)(
                tau.as_ptr(),
                tau_pred.as_ptr(),
                tau.len() as c_int,
                self.nq as c_int,
                &mut metrics,
                rmse.as_mut_ptr(),
                mae.as_mut_ptr(),
                self.nq as c_int,
            )
        };
        if rc != 0 {
            return Err(self.api.last_error().into());
        }
        Ok((metrics, rmse, mae))
    }
}

impl Drop for MathModel {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { (self.api.model_free)(self.handle) };
            self.handle = std::ptr::null_mut();
        }
    }
}

pub struct Pose {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3],
    pub matrix: Mat4,
}

fn math_lib_path() -> PathBuf {
    if let Ok(path) = env::var("REBOTARM_MATH_LIB") {
        return PathBuf::from(path);
    }
    let source_tree = repo_root().join("python/rebotarm_control_rt/librebotarm_math.so");
    if source_tree.exists() {
        return source_tree;
    }
    PathBuf::from("librebotarm_math.so")
}

fn row_major_to_mat4(raw: [f64; 16]) -> Mat4 {
    let mut out = [[0.0_f64; 4]; 4];
    for r in 0..4 {
        for c in 0..4 {
            out[r][c] = raw[r * 4 + c];
        }
    }
    out
}

fn mat4_to_row_major(matrix: Mat4) -> [f64; 16] {
    let mut out = [0.0_f64; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = matrix[r][c];
        }
    }
    out
}

fn vec_to_q6(values: &[f64]) -> [f64; ARM_DOF] {
    let mut out = [0.0_f64; ARM_DOF];
    for idx in 0..ARM_DOF.min(values.len()) {
        out[idx] = values[idx];
    }
    out
}

pub fn print_pose_with_model(model: &MathModel, q: &[f64]) -> Result<(), Box<dyn Error>> {
    let pose = model.fk_cpp(q)?;
    println!(
        "  ee position: [{:+.4}, {:+.4}, {:+.4}] m",
        pose.xyz[0], pose.xyz[1], pose.xyz[2]
    );
    println!(
        "  ee rpy:      [{:+.2}, {:+.2}, {:+.2}] deg",
        pose.rpy[0].to_degrees(),
        pose.rpy[1].to_degrees(),
        pose.rpy[2].to_degrees()
    );
    Ok(())
}
