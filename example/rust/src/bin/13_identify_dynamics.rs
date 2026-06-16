use rebotarm_control_rt_rust_examples::common::{
    arg_value, has_flag, parse_bool_arg, parse_float_arg, parse_urdf_path, MathModel,
};
use std::env;
use std::error::Error;
use std::fs;

struct Dataset {
    q: Vec<f64>,
    dq: Vec<f64>,
    ddq: Vec<f64>,
    tau: Vec<f64>,
    samples: usize,
}

fn find_col(header: &[&str], name: &str) -> Result<usize, Box<dyn Error>> {
    header
        .iter()
        .position(|item| *item == name)
        .ok_or_else(|| format!("missing CSV column: {name}").into())
}

fn load_csv(path: &str, dof: usize) -> Result<Dataset, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    let header_line = lines.next().ok_or("empty CSV")?;
    let header: Vec<&str> = header_line.split(',').collect();
    let q_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, &format!("q{i}")))
        .collect::<Result<_, _>>()?;
    let dq_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, &format!("dq{i}")))
        .collect::<Result<_, _>>()?;
    let ddq_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, &format!("ddq{i}")))
        .collect::<Result<_, _>>()?;
    let tau_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, &format!("tau{i}")))
        .collect::<Result<_, _>>()?;

    let mut q = Vec::new();
    let mut dq = Vec::new();
    let mut ddq = Vec::new();
    let mut tau = Vec::new();
    let mut samples = 0;
    for line in lines {
        if line.trim().is_empty() {
            continue;
        }
        let values: Vec<f64> = line
            .split(',')
            .map(|cell| cell.trim().parse::<f64>())
            .collect::<Result<_, _>>()?;
        for idx in &q_cols {
            q.push(values[*idx]);
        }
        for idx in &dq_cols {
            dq.push(values[*idx]);
        }
        for idx in &ddq_cols {
            ddq.push(values[*idx]);
        }
        for idx in &tau_cols {
            tau.push(values[*idx]);
        }
        samples += 1;
    }
    Ok(Dataset {
        q,
        dq,
        ddq,
        tau,
        samples,
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!(
            "Usage: cargo run --bin 13_identify_dynamics -- --data calibration/id_data_train.csv [--mode full|base] [--urdf robot.urdf]"
        );
        return Ok(());
    }
    let data_path = arg_value(&args, "--data").ok_or("--data is required")?;
    let mode = arg_value(&args, "--mode").unwrap_or_else(|| "full".to_string());
    let include_friction = !parse_bool_arg(&args, "--no-friction", false);
    let coulomb_eps = parse_float_arg(&args, "--coulomb-eps", 1e-3);
    let rcond = parse_float_arg(&args, "--rcond", 1e-12);

    let model = MathModel::load(&parse_urdf_path(&args))?;
    let dataset = load_csv(&data_path, model.nq)?;
    let y = model.build_regression_matrix(
        &dataset.q,
        &dataset.dq,
        &dataset.ddq,
        dataset.samples,
        include_friction,
        coulomb_eps,
    )?;
    let tau = model.stack_tau_samples(&dataset.tau, dataset.samples)?;
    let rows = dataset.samples * model.nq;
    let cols = model.num_total_parameters(include_friction)?;

    let (tau_pred, rank, condition, residual) = if mode == "full" {
        let (beta, tau_pred, info) = model.fit_least_squares(&y, rows, cols, &tau, rcond)?;
        println!("beta length: {}", beta.len());
        (tau_pred, info.rank, info.condition, info.residual_norm)
    } else if mode == "base" {
        let (beta, selected, tau_pred, info) = model.fit_base_qr(&y, rows, cols, &tau, rcond)?;
        println!("base beta length: {}", beta.len());
        println!("selected columns: {:?}", selected);
        (tau_pred, info.rank, info.condition, info.residual_norm)
    } else {
        return Err("--mode must be full or base".into());
    };

    let (metrics, per_joint_rmse, _per_joint_mae) = model.regression_metrics(&tau, &tau_pred)?;
    println!(
        "fit mode={} samples={} rank={} cond={:.6e} residual={:.6e}",
        mode, dataset.samples, rank, condition, residual
    );
    println!(
        "rmse={:.6e} mae={:.6e} max_abs={:.6e} r2={:.6e}",
        metrics.rmse, metrics.mae, metrics.max_abs, metrics.r2
    );
    println!("per-joint rmse: {:?}", per_joint_rmse);
    Ok(())
}
