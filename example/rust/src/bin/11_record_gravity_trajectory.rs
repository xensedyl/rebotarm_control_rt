use motor_vendor_damiao::ControlMode;
use rebotarm_control_rt_rust_examples::common::{
    arg_value, gravity_urdf_for_gripper, has_flag, install_signal_handler, parse_bool_arg,
    parse_float_arg, parse_port, parse_rate, sleep_to_rate, stop_requested, B601Arm, MathModel,
    ALL_DOF, ARM_DOF,
};
use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Instant;

struct Row {
    t: f64,
    q: Vec<f64>,
    dq: Vec<f64>,
    tau: Vec<f64>,
}

fn ensure_parent_dir(path: &str) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = Path::new(path).parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    Ok(())
}

fn save_trajectory(path: &str, rows: &[Row], dof: usize) -> Result<(), Box<dyn Error>> {
    ensure_parent_dir(path)?;
    let mut text = String::new();
    text.push_str("time");
    for prefix in ["q", "dq"] {
        for i in 1..=dof {
            text.push_str(&format!(",{prefix}{i}"));
        }
    }
    for i in 1..=dof {
        text.push_str(&format!(",tau{i}"));
    }
    text.push('\n');
    for row in rows {
        text.push_str(&format!("{:.12}", row.t));
        for values in [&row.q, &row.dq, &row.tau] {
            for value in values {
                text.push_str(&format!(",{value:.12}"));
            }
        }
        text.push('\n');
    }
    fs::write(path, text)?;
    Ok(())
}

fn print_summary(rows: &[Row], dof: usize) {
    if rows.is_empty() {
        return;
    }
    let mut q_min = vec![f64::INFINITY; dof];
    let mut q_max = vec![f64::NEG_INFINITY; dof];
    let mut dq_abs = vec![0.0_f64; dof];
    for row in rows {
        for j in 0..dof {
            q_min[j] = q_min[j].min(row.q[j]);
            q_max[j] = q_max[j].max(row.q[j]);
            dq_abs[j] = dq_abs[j].max(row.dq[j].abs());
        }
    }
    let dt = if rows.len() >= 2 {
        (rows.last().unwrap().t - rows.first().unwrap().t) / (rows.len() - 1) as f64
    } else {
        0.0
    };
    println!(
        "samples={} dof={} duration={:.3}s dt~={:.4}s",
        rows.len(),
        dof,
        rows.last().unwrap().t,
        dt
    );
    println!("q min/max [deg]:");
    for j in 0..dof {
        println!(
            "  joint{}: {:+.2} .. {:+.2}",
            j + 1,
            q_min[j].to_degrees(),
            q_max[j].to_degrees()
        );
    }
    let max_dq: Vec<f64> = dq_abs.iter().map(|v| v.to_degrees()).collect();
    println!("max |dq| [deg/s]: {:?}", max_dq);
}

fn release_mit_torque_hold(arm: &B601Arm) {
    let q = arm.positions_or_zero();
    let zeros = vec![0.0_f32; ALL_DOF];
    for _ in 0..10 {
        let _ = arm.send_mit_all(&q, &zeros, &zeros, &zeros, &zeros);
        thread::sleep(std::time::Duration::from_millis(20));
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!(
            "Usage: cargo run --bin 11_record_gravity_trajectory -- --output calibration/recorded_trajectory.csv --port /dev/ttyACM0 --rate 200 --sample-rate 100 --kd 1.0 --gravity-scale 1.0 [--urdf robot.urdf] [--use_gripper=true]"
        );
        return Ok(());
    }

    let output = arg_value(&args, "--output")
        .unwrap_or_else(|| "calibration/recorded_trajectory.csv".to_string());
    let rate = parse_rate(&args, 200.0);
    let sample_rate = parse_float_arg(&args, "--sample-rate", 100.0);
    let max_duration = parse_float_arg(&args, "--max-duration-s", 180.0);
    let kd = parse_float_arg(&args, "--kd", 1.0) as f32;
    let gravity_scale = parse_float_arg(&args, "--gravity-scale", 1.0);
    let use_gripper = parse_bool_arg(&args, "--use_gripper", true);
    if sample_rate <= 0.0 {
        return Err("--sample-rate must be positive".into());
    }
    if max_duration <= 0.0 {
        return Err("--max-duration-s must be positive".into());
    }

    let (urdf_path, _temp_urdf, end_link_scale) = gravity_urdf_for_gripper(&args, use_gripper)?;
    let model = MathModel::load(&urdf_path)?;
    let dof = model.nq;
    let command_count = if use_gripper { ALL_DOF } else { ARM_DOF };

    println!("========================================================================");
    println!("  reBotArm hand-guided trajectory recording (Rust)");
    println!("========================================================================");
    println!("[urdf] {}", urdf_path.display());
    println!("[model] nq={}", model.nq);
    println!("[gripper/end_link load] scale={end_link_scale:.3}");
    println!("[free-drive] rate={rate:.1} Hz, kd={kd:.3}, gravity_scale={gravity_scale:.3}");
    println!("[record] sample_rate={sample_rate:.1} Hz, max_duration={max_duration:.1}s");
    println!(
        "After saving, gravity compensation keeps running. Drag the arm back to zero, then Ctrl+C."
    );
    println!("------------------------------------------------------------------------");

    install_signal_handler();
    let arm = B601Arm::open(&parse_port(&args))?;
    arm.enable()?;
    if use_gripper {
        arm.ensure_all_mode(ControlMode::Mit);
    } else {
        arm.ensure_arm_mode(ControlMode::Mit);
    }
    println!("[connect] OK\n[enable] OK\n[free-drive] started. Drag the arm by hand.");

    let start_record = Arc::new(AtomicBool::new(false));
    let stop_record = Arc::new(AtomicBool::new(false));
    {
        let start_record = Arc::clone(&start_record);
        let stop_record = Arc::clone(&stop_record);
        thread::spawn(move || {
            let mut line = String::new();
            println!("Press Enter to START recording after gravity compensation feels stable...");
            let _ = io::stdin().read_line(&mut line);
            start_record.store(true, Ordering::SeqCst);
            line.clear();
            println!("Recording. Press Enter to STOP recording and save. Gravity compensation will keep running...");
            let _ = io::stdin().read_line(&mut line);
            stop_record.store(true, Ordering::SeqCst);
        });
    }

    let mut rows = Vec::new();
    let mut recording = false;
    let mut record_start = Instant::now();
    let mut next_sample = record_start;
    let sample_period = std::time::Duration::from_secs_f64(1.0 / sample_rate);

    while !stop_requested() {
        let tick = Instant::now();
        let states = arm.states();
        let mut q_all = vec![0.0_f32; ALL_DOF];
        let mut dq_all = vec![0.0_f32; ALL_DOF];
        let mut tau_all = vec![0.0_f32; ALL_DOF];
        for (idx, state) in states.into_iter().enumerate().take(ALL_DOF) {
            if let Some(state) = state {
                q_all[idx] = state.pos;
                dq_all[idx] = state.vel;
                tau_all[idx] = state.torq;
            }
        }
        let q_model: Vec<f64> = q_all.iter().take(dof).map(|v| f64::from(*v)).collect();
        let tau_model = model.generalized_gravity_cpp(&q_model)?;
        for idx in 0..command_count {
            let tau = tau_model.get(idx).copied().unwrap_or(0.0) * gravity_scale;
            arm.motors[idx].send_cmd_mit(q_all[idx], 0.0, 0.0, kd, tau as f32)?;
        }

        if !recording && start_record.load(Ordering::SeqCst) {
            recording = true;
            record_start = tick;
            next_sample = tick;
            println!("[record] started.");
        }
        if recording && tick >= next_sample {
            let t = tick.duration_since(record_start).as_secs_f64();
            rows.push(Row {
                t,
                q: q_all.iter().take(dof).map(|v| f64::from(*v)).collect(),
                dq: dq_all.iter().take(dof).map(|v| f64::from(*v)).collect(),
                tau: tau_all.iter().take(dof).map(|v| f64::from(*v)).collect(),
            });
            next_sample += sample_period;
            if t >= max_duration {
                println!("[record] max duration reached.");
                stop_record.store(true, Ordering::SeqCst);
            }
        }
        if recording && stop_record.load(Ordering::SeqCst) {
            if rows.is_empty() {
                println!("[record] no samples captured.");
            } else {
                save_trajectory(&output, &rows, dof)?;
                println!("[saved] {output}");
                print_summary(&rows, dof);
            }
            println!(
                "[free-drive] still running. Drag the arm back to zero, then press Ctrl+C to exit."
            );
            recording = false;
            stop_record.store(false, Ordering::SeqCst);
        }
        sleep_to_rate(tick, rate);
    }

    println!("\n[stop] disconnecting...");
    release_mit_torque_hold(&arm);
    arm.close();
    println!("[done] disconnected");
    Ok(())
}
