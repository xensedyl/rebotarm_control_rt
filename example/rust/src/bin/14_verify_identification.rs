use rebotarm_control_rt_rust_examples::common::{
    arg_value, has_flag, parse_float_arg, parse_urdf_path, MathModel,
};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

struct Dataset {
    q: Vec<f64>,
    dq: Vec<f64>,
    ddq: Vec<f64>,
    tau: Vec<f64>,
    samples: usize,
}

fn find_col(header: &[&str], prefix: &str, joint: usize) -> Result<usize, Box<dyn Error>> {
    let names = [
        format!("{prefix}{joint}"),
        format!("{prefix}_{joint}"),
        format!("{prefix}.joint_{joint}"),
    ];
    header
        .iter()
        .position(|item| names.iter().any(|name| name == item))
        .ok_or_else(|| format!("missing CSV column for {prefix}{joint}").into())
}

fn load_csv(path: &str, dof: usize) -> Result<Dataset, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    let header_line = lines.next().ok_or("empty CSV")?;
    let header: Vec<&str> = header_line.split(',').map(str::trim).collect();
    let q_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, "q", i))
        .collect::<Result<_, _>>()?;
    let dq_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, "dq", i))
        .collect::<Result<_, _>>()?;
    let ddq_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, "ddq", i))
        .collect::<Result<_, _>>()?;
    let tau_cols: Vec<_> = (1..=dof)
        .map(|i| find_col(&header, "tau", i))
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

fn trim(text: &str) -> &str {
    text.trim_matches(|ch: char| ch.is_whitespace())
}

fn unquote(text: &str) -> String {
    let text = trim(text);
    if text.len() >= 2
        && ((text.starts_with('"') && text.ends_with('"'))
            || (text.starts_with('\'') && text.ends_with('\'')))
    {
        text[1..text.len() - 1].to_string()
    } else {
        text.to_string()
    }
}

fn yaml_value<'a>(yaml: &'a str, key: &str) -> Option<&'a str> {
    let prefix = format!("{key}:");
    yaml.lines().find_map(|line| {
        let stripped = trim(line);
        stripped.strip_prefix(&prefix).map(|value| trim(value))
    })
}

fn yaml_string(yaml: &str, key: &str, default: &str) -> String {
    yaml_value(yaml, key)
        .map(unquote)
        .unwrap_or_else(|| default.to_string())
}

fn yaml_bool(yaml: &str, key: &str, default: bool) -> bool {
    yaml_value(yaml, key)
        .map(|value| {
            matches!(
                trim(value).to_ascii_lowercase().as_str(),
                "true" | "1" | "yes" | "on"
            )
        })
        .unwrap_or(default)
}

fn parse_number_list_body(body: &str) -> Vec<f64> {
    body.replace([',', '[', ']'], " ")
        .split_whitespace()
        .filter_map(|value| value.parse::<f64>().ok())
        .collect()
}

fn yaml_float_vector(yaml: &str, key: &str) -> Vec<f64> {
    let prefix = format!("{key}:");
    let mut in_block = false;
    let mut body = String::new();
    for line in yaml.lines() {
        let stripped = trim(line);
        if !in_block {
            if let Some(tail) = stripped.strip_prefix(&prefix) {
                let tail = trim(tail);
                if tail.contains('[') {
                    body.push_str(tail);
                    if tail.contains(']') {
                        break;
                    }
                    in_block = true;
                } else {
                    in_block = true;
                }
            }
            continue;
        }
        if body.contains(']') {
            break;
        }
        if let Some(item) = stripped.strip_prefix("- ") {
            body.push(' ');
            body.push_str(item);
        } else if stripped.contains(']') || stripped.contains('[') || stripped.contains(',') {
            body.push(' ');
            body.push_str(stripped);
        } else if !body.is_empty() {
            break;
        }
    }
    parse_number_list_body(&body)
}

fn yaml_int_vector(yaml: &str, key: &str) -> Vec<usize> {
    yaml_float_vector(yaml, key)
        .into_iter()
        .map(|value| value as usize)
        .collect()
}

fn mat_vec_mul(y: &[f64], rows: usize, cols: usize, x: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; rows];
    for r in 0..rows {
        let mut sum = 0.0;
        for c in 0..cols {
            sum += y[r * cols + c] * x[c];
        }
        out[r] = sum;
    }
    out
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!(
            "Usage: cargo run --bin 14_verify_identification -- --data id_verify.csv --params identified.yaml [--urdf robot.urdf]\n\
             or: cargo run --bin 14_verify_identification -- --data id_verify.csv --beta beta.txt [--urdf robot.urdf] [--no-friction]"
        );
        return Ok(());
    }

    let data_path = arg_value(&args, "--data").ok_or("--data is required")?;
    let params_path = arg_value(&args, "--params");
    let beta_path = arg_value(&args, "--beta");
    if params_path.is_none() && beta_path.is_none() {
        return Err("either --params or --beta is required".into());
    }

    let mut mode = "full".to_string();
    let mut include_friction = !has_flag(&args, "--no-friction");
    let mut coulomb_eps = parse_float_arg(&args, "--coulomb-eps", 1e-3);
    let mut selected_columns: Vec<usize> = Vec::new();
    let mut beta: Vec<f64>;
    let urdf_path: PathBuf;

    if let Some(params_path) = params_path {
        let yaml = fs::read_to_string(params_path)?;
        mode = yaml_string(&yaml, "mode", "full");
        include_friction = yaml_bool(&yaml, "include_friction", true);
        if has_flag(&args, "--no-friction") {
            include_friction = false;
        }
        if let Some(value) = yaml_value(&yaml, "coulomb_eps") {
            coulomb_eps = value.parse::<f64>().unwrap_or(coulomb_eps);
        }
        urdf_path = arg_value(&args, "--urdf")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(yaml_string(&yaml, "input_urdf", "")));
        if mode == "base" {
            beta = yaml_float_vector(&yaml, "beta_base");
            if beta.is_empty() {
                beta = yaml_float_vector(&yaml, "beta");
            }
            selected_columns = yaml_int_vector(&yaml, "selected_columns");
        } else if mode == "full" {
            beta = yaml_float_vector(&yaml, "beta");
        } else {
            return Err("--params must contain mode full or base".into());
        }
    } else {
        let text = fs::read_to_string(beta_path.unwrap())?;
        beta = text
            .split_whitespace()
            .map(|value| value.parse::<f64>())
            .collect::<Result<_, _>>()?;
        urdf_path = parse_urdf_path(&args);
    }
    if beta.is_empty() {
        return Err("no beta values found".into());
    }

    let model = MathModel::load(&urdf_path)?;
    let dataset = load_csv(&data_path, model.nq)?;
    let rows = dataset.samples * model.nq;
    let cols = model.num_total_parameters(include_friction)?;
    let y = model.build_regression_matrix(
        &dataset.q,
        &dataset.dq,
        &dataset.ddq,
        dataset.samples,
        include_friction,
        coulomb_eps,
    )?;
    let tau = model.stack_tau_samples(&dataset.tau, dataset.samples)?;
    let tau_pred = if mode == "base" {
        if selected_columns.len() != beta.len() {
            return Err("base beta length does not match selected_columns".into());
        }
        let mut out = vec![0.0; rows];
        for r in 0..rows {
            let mut sum = 0.0;
            for (i, col) in selected_columns.iter().copied().enumerate() {
                if col >= cols {
                    return Err("selected column out of range".into());
                }
                sum += y[r * cols + col] * beta[i];
            }
            out[r] = sum;
        }
        out
    } else {
        if beta.len() != cols {
            return Err("beta length does not match regressor columns".into());
        }
        mat_vec_mul(&y, rows, cols, &beta)
    };
    let (metrics, per_joint_rmse, per_joint_mae) = model.regression_metrics(&tau, &tau_pred)?;
    println!(
        "verify samples={} rmse={:.6} mae={:.6} max_abs={:.6} r2={:.6}",
        dataset.samples, metrics.rmse, metrics.mae, metrics.max_abs, metrics.r2
    );
    println!("per-joint rmse: {:?}", per_joint_rmse);
    println!("per-joint mae:  {:?}", per_joint_mae);
    Ok(())
}
