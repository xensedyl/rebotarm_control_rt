use rebotarm_control_rt_rust_examples::common::{
    arg_value, default_vlim, has_flag, install_signal_handler, parse_float_arg, parse_port,
    parse_rate, sleep_to_rate, stop_requested, B601Arm, ControlMode,
};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;
use std::time::Instant;

struct Trajectory {
    time: Vec<f64>,
    q: Vec<Vec<f64>>,
    dq: Vec<Vec<f64>>,
}

impl Trajectory {
    fn samples(&self) -> usize {
        self.q.len()
    }
    fn dof(&self) -> usize {
        self.q.first().map(|q| q.len()).unwrap_or(0)
    }
    fn duration(&self) -> f64 {
        *self.time.last().unwrap_or(&0.0)
    }
    fn dt(&self) -> f64 {
        if self.time.len() < 2 {
            0.0
        } else {
            (self.time.last().unwrap() - self.time.first().unwrap()) / (self.time.len() - 1) as f64
        }
    }
}

#[derive(Clone)]
struct SampleRow {
    t: f64,
    q: Vec<f64>,
    dq: Vec<f64>,
    tau: Vec<f64>,
}

fn split_csv(line: &str) -> Vec<&str> {
    line.split(',').map(str::trim).collect()
}

fn find_col(header: &[&str], name: &str) -> Option<usize> {
    header.iter().position(|item| *item == name)
}

fn load_trajectory(path: &str) -> Result<Trajectory, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    let header_line = lines.next().ok_or("empty trajectory CSV")?;
    let header = split_csv(header_line);
    let mut dof = 0;
    while find_col(&header, &format!("q{}", dof + 1)).is_some() {
        dof += 1;
    }
    if dof == 0 {
        return Err("could not infer trajectory dof from q columns".into());
    }
    let mut q_cols = Vec::new();
    let mut dq_cols = Vec::new();
    for i in 1..=dof {
        q_cols.push(find_col(&header, &format!("q{i}")).ok_or("missing q column")?);
        dq_cols.push(find_col(&header, &format!("dq{i}")).ok_or("missing dq column")?);
    }
    let time_col = find_col(&header, "time");
    let mut traj = Trajectory {
        time: Vec::new(),
        q: Vec::new(),
        dq: Vec::new(),
    };
    for line in lines {
        if line.trim().is_empty() {
            continue;
        }
        let values: Vec<f64> = line
            .split(',')
            .map(|cell| cell.trim().parse::<f64>())
            .collect::<Result<_, _>>()?;
        traj.time.push(
            time_col
                .map(|idx| values[idx])
                .unwrap_or(traj.time.len() as f64),
        );
        traj.q.push(q_cols.iter().map(|idx| values[*idx]).collect());
        traj.dq
            .push(dq_cols.iter().map(|idx| values[*idx]).collect());
    }
    if traj.q.is_empty() {
        return Err("trajectory contains no samples".into());
    }
    Ok(traj)
}

fn print_summary(traj: &Trajectory) {
    let dof = traj.dof();
    let mut q_min = vec![f64::INFINITY; dof];
    let mut q_max = vec![f64::NEG_INFINITY; dof];
    let mut dq_abs = vec![0.0_f64; dof];
    for i in 0..traj.samples() {
        for j in 0..dof {
            q_min[j] = q_min[j].min(traj.q[i][j]);
            q_max[j] = q_max[j].max(traj.q[i][j]);
            dq_abs[j] = dq_abs[j].max(traj.dq[i][j].abs());
        }
    }
    println!(
        "samples={} dof={} duration={:.3}s dt={:.4}s",
        traj.samples(),
        dof,
        traj.duration(),
        traj.dt()
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

fn ensure_parent_dir(path: &str) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = Path::new(path).parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    Ok(())
}

fn resample_rows(rows: &[SampleRow], fps: f64) -> Vec<SampleRow> {
    if fps <= 0.0 || rows.len() < 2 {
        return rows.to_vec();
    }
    let t0 = rows.first().unwrap().t;
    let t_end = rows.last().unwrap().t;
    let mut out = Vec::new();
    let mut idx = 0;
    let mut t = t0;
    while t <= t_end + 1e-12 {
        while idx + 1 < rows.len() && rows[idx + 1].t <= t {
            idx += 1;
        }
        out.push(rows[idx].clone());
        t += 1.0 / fps;
    }
    out
}

fn save_identification_csv(
    path: &str,
    raw_rows: &[SampleRow],
    dof: usize,
    output_fps: f64,
) -> Result<(), Box<dyn Error>> {
    let rows = resample_rows(raw_rows, output_fps);
    if rows.is_empty() {
        return Err("no samples collected".into());
    }
    let mut ddq = vec![vec![0.0; dof]; rows.len()];
    if rows.len() >= 2 {
        for i in 0..rows.len() {
            let prev = if i == 0 { 0 } else { i - 1 };
            let next = if i + 1 >= rows.len() {
                rows.len() - 1
            } else {
                i + 1
            };
            let dt = (rows[next].t - rows[prev].t).max(1e-9);
            for j in 0..dof {
                ddq[i][j] = (rows[next].dq[j] - rows[prev].dq[j]) / dt;
            }
        }
    }
    ensure_parent_dir(path)?;
    let mut text = String::new();
    text.push_str("time");
    for prefix in ["q", "dq", "ddq", "tau"] {
        for i in 1..=dof {
            text.push_str(&format!(",{prefix}{i}"));
        }
    }
    text.push('\n');
    let t0 = rows.first().unwrap().t;
    for (i, row) in rows.iter().enumerate() {
        text.push_str(&format!("{:.12}", row.t - t0));
        for values in [&row.q, &row.dq, &ddq[i], &row.tau] {
            for value in values {
                text.push_str(&format!(",{value:.12}"));
            }
        }
        text.push('\n');
    }
    fs::write(path, text)?;
    Ok(())
}

fn move_to_start(
    arm: &B601Arm,
    q_start: &[f64],
    vlim: &[f32],
    rate: f64,
    threshold_rad: f64,
) -> Result<(), Box<dyn Error>> {
    let q_cur = arm.positions_or_zero();
    let mut max_delta = 0.0_f64;
    for idx in 0..q_start.len() {
        max_delta = max_delta.max((q_start[idx] - f64::from(q_cur[idx])).abs());
    }
    let mut target = q_cur.clone();
    if max_delta <= threshold_rad {
        for idx in 0..q_start.len() {
            target[idx] = q_start[idx] as f32;
        }
        arm.send_pos_vel_all(&target, vlim)?;
        return Ok(());
    }
    let min_vlim = vlim
        .iter()
        .take(q_start.len())
        .fold(f32::INFINITY, |acc, value| acc.min(*value));
    let duration = (max_delta / f64::from(min_vlim).max(1e-6)).max(1.0);
    let steps = (duration * rate).ceil().max(2.0) as usize;
    println!("[pre] moving to trajectory start in {duration:.2}s ({steps} steps)");
    for step in 0..=steps {
        if stop_requested() {
            break;
        }
        let tick = Instant::now();
        let mut alpha = step as f64 / steps as f64;
        alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        target = q_cur.clone();
        for idx in 0..q_start.len() {
            target[idx] = ((1.0 - alpha) * f64::from(q_cur[idx]) + alpha * q_start[idx]) as f32;
        }
        arm.send_pos_vel_all(&target, vlim)?;
        sleep_to_rate(tick, rate);
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!(
            "Usage: cargo run --bin 12_collect_identification_data -- --trajectory calibration/recorded.csv --output calibration/id_data.csv --port /dev/ttyACM0 [--execute]"
        );
        return Ok(());
    }
    let trajectory_path = arg_value(&args, "--trajectory").ok_or("--trajectory is required")?;
    let output =
        arg_value(&args, "--output").unwrap_or_else(|| "calibration/id_data_train.csv".to_string());
    let rate = parse_rate(&args, 150.0);
    let feedback_rate = parse_float_arg(&args, "--feedback-rate", 300.0);
    let output_fps = parse_float_arg(&args, "--output-fps", 0.0);
    let vlim_value = parse_float_arg(&args, "--vlim", 0.8) as f32;
    let start_threshold = parse_float_arg(&args, "--start-threshold-deg", 2.0).to_radians();
    let settle_s = parse_float_arg(&args, "--settle-s", 1.0);
    let execute = has_flag(&args, "--execute");

    let traj = load_trajectory(&trajectory_path)?;
    println!("========================================================================");
    println!("  reBotArm recorded trajectory replay and identification data collection (Rust)");
    println!("========================================================================");
    print_summary(&traj);
    println!("output: {output}");
    if !execute {
        println!("\n[dry-run] Add --execute to move the real arm and collect data.");
        return Ok(());
    }

    install_signal_handler();
    let arm = B601Arm::open(&parse_port(&args))?;
    arm.enable()?;
    arm.ensure_all_mode(ControlMode::PosVel);
    println!("[connect] OK\n[enable] OK\n[mode] POS_VEL");

    let mut vlim = default_vlim();
    vlim.fill(vlim_value);
    move_to_start(&arm, &traj.q[0], &vlim, rate, start_threshold)?;
    if settle_s > 0.0 {
        println!("[pre] settling {settle_s:.2}s");
        let end = Instant::now() + std::time::Duration::from_secs_f64(settle_s);
        let mut target = arm.positions_or_zero();
        for (idx, value) in traj.q[0].iter().enumerate() {
            target[idx] = *value as f32;
        }
        while Instant::now() < end && !stop_requested() {
            let tick = Instant::now();
            arm.send_pos_vel_all(&target, &vlim)?;
            sleep_to_rate(tick, rate);
        }
    }

    let mut rows = Vec::new();
    println!("[record] executing trajectory...");
    let start = Instant::now();
    let mut next_sample = start;
    let sample_period = std::time::Duration::from_secs_f64(1.0 / feedback_rate);
    let mut target_index = 0;
    while !stop_requested() && target_index < traj.samples() {
        let tick = Instant::now();
        let elapsed = tick.duration_since(start).as_secs_f64();
        while target_index + 1 < traj.samples() && traj.time[target_index + 1] <= elapsed {
            target_index += 1;
        }
        let mut target = arm.positions_or_zero();
        for idx in 0..traj.dof() {
            target[idx] = traj.q[target_index][idx] as f32;
        }
        arm.send_pos_vel_all(&target, &vlim)?;

        if tick >= next_sample {
            let states = arm.states();
            let mut row = SampleRow {
                t: elapsed,
                q: vec![0.0; traj.dof()],
                dq: vec![0.0; traj.dof()],
                tau: vec![0.0; traj.dof()],
            };
            for idx in 0..traj.dof() {
                if let Some(Some(state)) = states.get(idx) {
                    row.q[idx] = f64::from(state.pos);
                    row.dq[idx] = f64::from(state.vel);
                    row.tau[idx] = f64::from(state.torq);
                }
            }
            rows.push(row);
            next_sample += sample_period;
        }
        if elapsed >= *traj.time.last().unwrap() {
            break;
        }
        sleep_to_rate(tick, rate);
    }

    if !rows.is_empty() {
        save_identification_csv(&output, &rows, traj.dof(), output_fps)?;
        println!("[saved] {output}");
        println!("[record] collected {} raw samples", rows.len());
    }
    arm.close();
    Ok(())
}
