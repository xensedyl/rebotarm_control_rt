use rebotarm_control_rt_rust_examples::common::{
    arg_value, has_flag, parse_float_arg, parse_urdf_path, MathModel,
};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;
use std::process;
use std::time::{SystemTime, UNIX_EPOCH};

struct Dataset {
    q: Vec<f64>,
    dq: Vec<f64>,
    ddq: Vec<f64>,
    tau: Vec<f64>,
    samples: usize,
}

struct PayloadFit {
    beta: Vec<f64>,
    payload_params: Vec<f64>,
    nominal_params: Vec<f64>,
    tau_pred: Vec<f64>,
    rank: i32,
    condition: f64,
    residual_norm: f64,
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

fn ensure_parent_dir(path: &str) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = Path::new(path).parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    Ok(())
}

fn yaml_float(value: f64) -> String {
    format!("{value:.12}")
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

fn format_float(value: f64) -> String {
    let text = format!("{value:.10}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text.is_empty() {
        "0".to_string()
    } else {
        text.to_string()
    }
}

fn vector_yaml(name: &str, values: &[f64]) -> String {
    let body = values
        .iter()
        .map(|value| yaml_float(*value))
        .collect::<Vec<_>>()
        .join(", ");
    format!("{name}: [{body}]\n")
}

fn tag_text<'a>(source: &'a str, tag: &str) -> Option<&'a str> {
    let start = source.find(&format!("<{tag}"))?;
    let end = source[start..].find('>')? + start + 1;
    Some(&source[start..end])
}

fn attr_value(tag: &str, attr: &str) -> Option<String> {
    let needle = format!("{attr}=\"");
    let (offset, quote) = if let Some(pos) = tag.find(&needle) {
        (pos + needle.len(), '"')
    } else {
        let needle = format!("{attr}='");
        (tag.find(&needle)? + needle.len(), '\'')
    };
    let end = tag[offset..].find(quote)? + offset;
    Some(tag[offset..end].to_string())
}

fn link_range(xml: &str, link_name: &str) -> Result<(usize, usize), Box<dyn Error>> {
    let name_pos = xml
        .find(&format!("name=\"{link_name}\""))
        .or_else(|| xml.find(&format!("name='{link_name}'")))
        .ok_or_else(|| format!("URDF link not found: {link_name}"))?;
    let start = xml[..name_pos]
        .rfind("<link")
        .ok_or_else(|| format!("URDF link start not found: {link_name}"))?;
    let end = xml[start..]
        .find("</link>")
        .ok_or_else(|| format!("URDF link is not closed: {link_name}"))?
        + start
        + "</link>".len();
    Ok((start, end))
}

fn inertial_range(link_text: &str) -> Result<Option<(usize, usize)>, Box<dyn Error>> {
    let Some(start) = link_text.find("<inertial") else {
        return Ok(None);
    };
    let end = link_text[start..]
        .find("</inertial>")
        .ok_or("URDF inertial block is not closed")?
        + start
        + "</inertial>".len();
    Ok(Some((start, end)))
}

fn parse_xyz(text: &str) -> Result<[f64; 3], Box<dyn Error>> {
    let values: Vec<f64> = text
        .split_whitespace()
        .map(|value| value.parse::<f64>())
        .collect::<Result<_, _>>()?;
    if values.len() != 3 {
        return Err("expected 3 xyz values".into());
    }
    Ok([values[0], values[1], values[2]])
}

fn symmetric_from_params(values: &[f64]) -> [[f64; 3]; 3] {
    [
        [values[0], values[1], values[3]],
        [values[1], values[2], values[4]],
        [values[3], values[4], values[5]],
    ]
}

fn symmetric_to_params(matrix: [[f64; 3]; 3]) -> [f64; 6] {
    [
        matrix[0][0],
        matrix[0][1],
        matrix[1][1],
        matrix[0][2],
        matrix[1][2],
        matrix[2][2],
    ]
}

fn parallel_axis(mass: f64, com: [f64; 3]) -> [[f64; 3]; 3] {
    let norm2 = com[0] * com[0] + com[1] * com[1] + com[2] * com[2];
    let mut out = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = mass * if r == c { norm2 } else { 0.0 } - mass * com[r] * com[c];
        }
    }
    out
}

fn mat_add(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = a[r][c] + b[r][c];
        }
    }
    out
}

fn mat_sub(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = a[r][c] - b[r][c];
        }
    }
    out
}

fn mat_scale(a: [[f64; 3]; 3], scale: f64) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = a[r][c] * scale;
        }
    }
    out
}

fn dynamic_params_from_inertia(
    mass: f64,
    com: [f64; 3],
    inertia_at_com: [[f64; 3]; 3],
) -> Vec<f64> {
    let mut params = vec![0.0; 10];
    params[0] = mass;
    params[1] = mass * com[0];
    params[2] = mass * com[1];
    params[3] = mass * com[2];
    let i_origin = mat_add(inertia_at_com, parallel_axis(mass, com));
    params[4..10].copy_from_slice(&symmetric_to_params(i_origin));
    params
}

fn inertia_from_dynamic_params(
    params: &[f64],
) -> Result<(f64, [f64; 3], [[f64; 3]; 3]), Box<dyn Error>> {
    if params.len() != 10 {
        return Err("one inertial block must contain 10 parameters".into());
    }
    let mass = params[0];
    if !mass.is_finite() || mass <= 0.0 {
        return Err(format!("identified mass must be positive: {mass}").into());
    }
    let com = [params[1] / mass, params[2] / mass, params[3] / mass];
    let i_origin = symmetric_from_params(&params[4..10]);
    let ic = mat_sub(i_origin, parallel_axis(mass, com));
    Ok((mass, com, ic))
}

fn default_payload_params(mass: f64) -> Result<Vec<f64>, Box<dyn Error>> {
    if !mass.is_finite() || mass <= 0.0 {
        return Err("default payload mass must be positive".into());
    }
    Ok(vec![mass, 0.0, 0.0, 0.0, 1e-5, 0.0, 1e-5, 0.0, 0.0, 1e-5])
}

fn dynamic_params_from_link(
    xml: &str,
    link_name: &str,
    default_mass: f64,
) -> Result<Vec<f64>, Box<dyn Error>> {
    let (start, end) = link_range(xml, link_name)?;
    let link_text = &xml[start..end];
    let Some((i_start, i_end)) = inertial_range(link_text)? else {
        return default_payload_params(default_mass);
    };
    let block = &link_text[i_start..i_end];
    let Some(mass_tag) = tag_text(block, "mass") else {
        return default_payload_params(default_mass);
    };
    let Some(inertia_tag) = tag_text(block, "inertia") else {
        return default_payload_params(default_mass);
    };
    let mass = attr_value(mass_tag, "value")
        .unwrap_or_else(|| "0".to_string())
        .parse::<f64>()?;
    let mut com = [0.0, 0.0, 0.0];
    if let Some(origin_tag) = tag_text(block, "origin") {
        if let Some(xyz) = attr_value(origin_tag, "xyz") {
            com = parse_xyz(&xyz)?;
        }
    }
    let mut ic = [[0.0; 3]; 3];
    ic[0][0] = attr_value(inertia_tag, "ixx")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    ic[0][1] = attr_value(inertia_tag, "ixy")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    ic[1][0] = ic[0][1];
    ic[0][2] = attr_value(inertia_tag, "ixz")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    ic[2][0] = ic[0][2];
    ic[1][1] = attr_value(inertia_tag, "iyy")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    ic[1][2] = attr_value(inertia_tag, "iyz")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    ic[2][1] = ic[1][2];
    ic[2][2] = attr_value(inertia_tag, "izz")
        .unwrap_or_else(|| "0".to_string())
        .parse()?;
    Ok(dynamic_params_from_inertia(mass, com, ic))
}

fn leading_indent(text: &str, pos: usize) -> String {
    let line_start = text[..pos].rfind('\n').map(|idx| idx + 1).unwrap_or(0);
    text[line_start..pos].to_string()
}

fn format_inertial_block(
    params: &[f64],
    indent: &str,
    rpy: &str,
) -> Result<String, Box<dyn Error>> {
    let (mass, com, ic) = inertia_from_dynamic_params(params)?;
    let child = format!("{indent}  ");
    let attr = format!("{indent}    ");
    Ok(format!(
        "{indent}<inertial>\n\
{child}<origin\n\
{attr}xyz=\"{} {} {}\"\n\
{attr}rpy=\"{rpy}\" />\n\
{child}<mass\n\
{attr}value=\"{}\" />\n\
{child}<inertia\n\
{attr}ixx=\"{}\"\n\
{attr}ixy=\"{}\"\n\
{attr}ixz=\"{}\"\n\
{attr}iyy=\"{}\"\n\
{attr}iyz=\"{}\"\n\
{attr}izz=\"{}\" />\n\
{indent}</inertial>",
        format_float(com[0]),
        format_float(com[1]),
        format_float(com[2]),
        format_float(mass),
        format_float(ic[0][0]),
        format_float(ic[0][1]),
        format_float(ic[0][2]),
        format_float(ic[1][1]),
        format_float(ic[1][2]),
        format_float(ic[2][2]),
    ))
}

fn replace_link_inertial(
    xml: &str,
    link_name: &str,
    params: &[f64],
) -> Result<String, Box<dyn Error>> {
    let (link_start, link_end) = link_range(xml, link_name)?;
    let link_text = &xml[link_start..link_end];
    let (i_start, i_end) = inertial_range(link_text)?
        .ok_or_else(|| format!("URDF link has no inertial: {link_name}"))?;
    let block = &link_text[i_start..i_end];
    let rpy = tag_text(block, "origin")
        .and_then(|tag| attr_value(tag, "rpy"))
        .unwrap_or_else(|| "0 0 0".to_string());
    let indent = leading_indent(link_text, i_start);
    let new_inertial = format_inertial_block(params, &indent, &rpy)?;
    let new_link = format!(
        "{}{}{}",
        &link_text[..i_start],
        new_inertial,
        &link_text[i_end..]
    );
    Ok(format!(
        "{}{}{}",
        &xml[..link_start],
        new_link,
        &xml[link_end..]
    ))
}

fn remove_link_inertial(xml: &str, link_name: &str) -> Result<String, Box<dyn Error>> {
    let (link_start, link_end) = link_range(xml, link_name)?;
    let link_text = &xml[link_start..link_end];
    let Some((i_start, i_end)) = inertial_range(link_text)? else {
        return Ok(xml.to_string());
    };
    let new_link = format!("{}{}", &link_text[..i_start], &link_text[i_end..]);
    Ok(format!(
        "{}{}{}",
        &xml[..link_start],
        new_link,
        &xml[link_end..]
    ))
}

fn child_link_from_joint_text(joint_text: &str) -> Option<String> {
    tag_text(joint_text, "child").and_then(|tag| attr_value(tag, "link"))
}

fn parent_link_from_joint_text(joint_text: &str) -> Option<String> {
    tag_text(joint_text, "parent").and_then(|tag| attr_value(tag, "link"))
}

fn joint_blocks(xml: &str) -> Result<Vec<&str>, Box<dyn Error>> {
    let mut blocks = Vec::new();
    let mut offset = 0;
    while let Some(rel_start) = xml[offset..].find("<joint") {
        let start = offset + rel_start;
        let end = start
            + xml[start..]
                .find("</joint>")
                .ok_or("URDF joint block is not closed")?
            + "</joint>".len();
        blocks.push(&xml[start..end]);
        offset = end;
    }
    Ok(blocks)
}

fn movable_joint_child_links(xml: &str) -> Result<Vec<String>, Box<dyn Error>> {
    let mut links = Vec::new();
    for joint_text in joint_blocks(xml)? {
        let is_fixed = tag_text(joint_text, "joint")
            .and_then(|tag| attr_value(tag, "type"))
            .is_some_and(|value| value == "fixed");
        if !is_fixed {
            if let Some(child) = child_link_from_joint_text(joint_text) {
                links.push(child);
            }
        }
    }
    Ok(links)
}

fn fixed_descendant_links(xml: &str, parents: &[String]) -> Result<Vec<String>, Box<dyn Error>> {
    let mut edges: Vec<(String, String)> = Vec::new();
    for joint_text in joint_blocks(xml)? {
        let is_fixed = tag_text(joint_text, "joint")
            .and_then(|tag| attr_value(tag, "type"))
            .is_some_and(|value| value == "fixed");
        if is_fixed {
            if let (Some(parent), Some(child)) = (
                parent_link_from_joint_text(joint_text),
                child_link_from_joint_text(joint_text),
            ) {
                edges.push((parent, child));
            }
        }
    }

    let mut stack = parents.to_vec();
    let mut seen = parents.to_vec();
    let mut out = Vec::new();
    while let Some(parent) = stack.pop() {
        for (edge_parent, edge_child) in &edges {
            if edge_parent != &parent || seen.iter().any(|value| value == edge_child) {
                continue;
            }
            seen.push(edge_child.clone());
            out.push(edge_child.clone());
            stack.push(edge_child.clone());
        }
    }
    Ok(out)
}

fn remove_link_inertials(xml: &str, link_names: &[String]) -> Result<String, Box<dyn Error>> {
    let mut out = xml.to_string();
    for link_name in link_names {
        out = remove_link_inertial(&out, link_name)?;
    }
    Ok(out)
}

fn apply_full_dynamic_parameters_to_urdf(
    xml: &str,
    dynamic_parameters: &[f64],
) -> Result<String, Box<dyn Error>> {
    if dynamic_parameters.len() % 10 != 0 {
        return Err("dynamic parameter vector length must be a multiple of 10".into());
    }
    let links = movable_joint_child_links(xml)?;
    let blocks = dynamic_parameters.len() / 10;
    if links.len() != blocks {
        return Err("URDF movable joint link count does not match dynamic parameter blocks".into());
    }
    let mut out = xml.to_string();
    for (i, link_name) in links.iter().enumerate() {
        out = replace_link_inertial(&out, link_name, &dynamic_parameters[i * 10..i * 10 + 10])?;
    }
    let fixed_links = fixed_descendant_links(&out, &links)?;
    remove_link_inertials(&out, &fixed_links)
}

fn payload_indices(count: usize) -> Result<Vec<usize>, Box<dyn Error>> {
    match count {
        4 => Ok(vec![0, 1, 2, 3]),
        10 => Ok((0..10).collect()),
        _ => Err("--payload-parameters must be 4 or 10".into()),
    }
}

fn payload_params_with_preserved_com_inertia(
    base_params: &[f64],
    payload_params: &[f64],
) -> Result<Vec<f64>, Box<dyn Error>> {
    let (base_mass, _base_com, base_ic) = inertia_from_dynamic_params(base_params)?;
    let mass = payload_params[0];
    if !mass.is_finite() || mass <= 0.0 {
        return Err(format!("identified payload mass must be positive: {mass}").into());
    }
    let com = [
        payload_params[1] / mass,
        payload_params[2] / mass,
        payload_params[3] / mass,
    ];
    Ok(dynamic_params_from_inertia(
        mass,
        com,
        mat_scale(base_ic, mass / base_mass),
    ))
}

fn temp_urdf(text: &str, label: &str) -> Result<std::path::PathBuf, Box<dyn Error>> {
    let ns = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let path = env::temp_dir().join(format!(
        "rebotarm_control_rt_{label}_{}_{}.urdf",
        process::id(),
        ns
    ));
    fs::write(&path, text)?;
    Ok(path)
}

fn inverse_dynamics_samples(
    model: &MathModel,
    dataset: &Dataset,
) -> Result<Vec<f64>, Box<dyn Error>> {
    let dof = model.nq;
    let mut out = Vec::with_capacity(dataset.samples * dof);
    for i in 0..dataset.samples {
        let start = i * dof;
        let tau = model.inverse_dynamics_cpp(
            &dataset.q[start..start + dof],
            &dataset.dq[start..start + dof],
            &dataset.ddq[start..start + dof],
        )?;
        out.extend(tau);
    }
    Ok(out)
}

fn vec_sub(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x - y).collect()
}

fn vec_add(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
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

fn norm(values: &[f64]) -> f64 {
    values.iter().map(|v| v * v).sum::<f64>().sqrt()
}

fn fit_payload(
    urdf_path: &Path,
    dataset: &Dataset,
    link_name: &str,
    parameter_count: usize,
    default_mass: f64,
    fd_eps: f64,
    rcond: f64,
) -> Result<PayloadFit, Box<dyn Error>> {
    if fd_eps <= 0.0 {
        return Err("--payload-fd-eps must be positive".into());
    }
    let xml = fs::read_to_string(urdf_path)?;
    let nominal_params = dynamic_params_from_link(&xml, link_name, default_mass)?;
    let indices = payload_indices(parameter_count)?;
    let arm_only_path = temp_urdf(&remove_link_inertial(&xml, link_name)?, "payload_arm_only")?;
    let nominal_path = temp_urdf(
        &replace_link_inertial(&xml, link_name, &nominal_params)?,
        "payload_nominal",
    )?;
    let _arm_model = MathModel::load(&arm_only_path)?;
    let nominal_model = MathModel::load(&nominal_path)?;
    let tau_nominal = inverse_dynamics_samples(&nominal_model, dataset)?;
    let rows = dataset.samples * nominal_model.nq;
    let cols = indices.len();
    let mut y = vec![0.0; rows * cols];

    for (col, param_index) in indices.iter().copied().enumerate() {
        let mut perturbed = nominal_params.clone();
        let scale = nominal_params[param_index].abs().max(1.0);
        let mut step = fd_eps * scale;
        if param_index == 0 {
            step = step.max(fd_eps);
        }
        perturbed[param_index] += step;
        if perturbed[0] <= 0.0 {
            return Err("payload mass perturbation became non-positive".into());
        }
        let perturbed_path = temp_urdf(
            &replace_link_inertial(&xml, link_name, &perturbed)?,
            "payload_perturbed",
        )?;
        let perturbed_model = MathModel::load(&perturbed_path)?;
        let tau_perturbed = inverse_dynamics_samples(&perturbed_model, dataset)?;
        for row in 0..rows {
            y[row * cols + col] = (tau_perturbed[row] - tau_nominal[row]) / step;
        }
    }

    let tau = nominal_model.stack_tau_samples(&dataset.tau, dataset.samples)?;
    let nominal_selected: Vec<f64> = indices.iter().map(|idx| nominal_params[*idx]).collect();
    let y_nominal = mat_vec_mul(&y, rows, cols, &nominal_selected);
    let tau_fixed = vec_sub(&tau_nominal, &y_nominal);
    let residual_tau = vec_sub(&tau, &tau_fixed);
    let (beta, tau_payload_pred, info) =
        nominal_model.fit_least_squares(&y, rows, cols, &residual_tau, rcond)?;
    let mut payload_params = nominal_params.clone();
    for (i, idx) in indices.iter().copied().enumerate() {
        payload_params[idx] = beta[i];
    }
    let tau_pred = vec_add(&tau_fixed, &tau_payload_pred);
    let residual_norm = norm(&vec_sub(&tau, &tau_pred));
    Ok(PayloadFit {
        beta,
        payload_params,
        nominal_params,
        tau_pred,
        rank: info.rank,
        condition: info.condition,
        residual_norm,
    })
}

fn write_result_yaml(
    path: &str,
    mode: &str,
    data_path: &str,
    input_urdf: &Path,
    identification_urdf: &Path,
    samples: usize,
    dof: usize,
    rank: i32,
    condition: f64,
    residual_norm: f64,
    metrics: rebotarm_control_rt_rust_examples::common::CMetrics,
    per_joint_rmse: &[f64],
    per_joint_mae: &[f64],
    beta: Option<&[f64]>,
    dynamic_parameters: Option<&[f64]>,
    selected_columns: Option<&[i32]>,
    payload: Option<&PayloadFit>,
    payload_link: &str,
    payload_parameter_count: usize,
    payload_default_mass: f64,
    payload_fd_eps: f64,
    rcond: f64,
    include_friction: bool,
    use_model_prior: bool,
) -> Result<(), Box<dyn Error>> {
    ensure_parent_dir(path)?;
    let mut text = String::new();
    text.push_str(&format!("mode: {mode}\n"));
    text.push_str(&format!("samples: {samples}\n"));
    text.push_str(&format!("dof: {dof}\n"));
    text.push_str(&format!("input_data: {data_path}\n"));
    text.push_str(&format!("input_urdf: {}\n", input_urdf.display()));
    text.push_str(&format!(
        "identification_urdf: {}\n",
        identification_urdf.display()
    ));
    text.push_str(&format!("include_friction: {}\n", include_friction));
    text.push_str(&format!("use_model_prior: {}\n", use_model_prior));
    text.push_str(&format!("rcond: {}\n", yaml_float(rcond)));
    text.push_str(&format!("rank: {rank}\n"));
    text.push_str(&format!("condition: {}\n", yaml_float(condition)));
    text.push_str(&format!("residual_norm: {}\n", yaml_float(residual_norm)));
    text.push_str("metrics:\n");
    text.push_str(&format!("  rmse: {}\n", yaml_float(metrics.rmse)));
    text.push_str(&format!("  mae: {}\n", yaml_float(metrics.mae)));
    text.push_str(&format!("  max_abs: {}\n", yaml_float(metrics.max_abs)));
    text.push_str(&format!("  r2: {}\n", yaml_float(metrics.r2)));
    text.push_str(&format!(
        "  per_joint_rmse: [{}]\n",
        per_joint_rmse
            .iter()
            .map(|v| yaml_float(*v))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    text.push_str(&format!(
        "  per_joint_mae: [{}]\n",
        per_joint_mae
            .iter()
            .map(|v| yaml_float(*v))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    if let Some(beta) = beta {
        text.push_str(&vector_yaml("beta", beta));
    }
    if let Some(params) = dynamic_parameters {
        text.push_str(&vector_yaml("dynamic_parameters", params));
    }
    if let Some(cols) = selected_columns {
        text.push_str(&format!(
            "selected_columns: [{}]\n",
            cols.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    if let Some(payload) = payload {
        text.push_str(&format!("payload_link: {payload_link}\n"));
        text.push_str(&format!(
            "payload_parameter_count: {payload_parameter_count}\n"
        ));
        text.push_str(&vector_yaml("payload_beta", &payload.beta));
        text.push_str(&vector_yaml(
            "payload_dynamic_parameters",
            &payload.payload_params,
        ));
        text.push_str(&vector_yaml(
            "nominal_payload_dynamic_parameters",
            &payload.nominal_params,
        ));
        text.push_str(&format!(
            "default_mass: {}\n",
            yaml_float(payload_default_mass)
        ));
        text.push_str(&format!(
            "finite_difference_eps: {}\n",
            yaml_float(payload_fd_eps)
        ));
    }
    fs::write(path, text)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if has_flag(&args, "--help") || has_flag(&args, "-h") {
        println!(
            "Usage: cargo run --bin 13_identify_dynamics -- --data calibration/id_data_train.csv [--mode full|base|payload] [--urdf robot.urdf] [--output out.yaml] [--urdf-output out.urdf]"
        );
        return Ok(());
    }
    let data_path = arg_value(&args, "--data").ok_or("--data is required")?;
    let mode = arg_value(&args, "--mode").unwrap_or_else(|| "full".to_string());
    let output = arg_value(&args, "--output")
        .unwrap_or_else(|| "calibration/identified_dynamics_rust.yaml".to_string());
    let urdf_output = arg_value(&args, "--urdf-output");
    let include_friction = !has_flag(&args, "--no-friction");
    let use_model_prior = !has_flag(&args, "--no-model-prior");
    let coulomb_eps = parse_float_arg(&args, "--coulomb-eps", 1e-3);
    let rcond = parse_float_arg(&args, "--rcond", 1e-12);
    let payload_link = arg_value(&args, "--payload-link").unwrap_or_else(|| "end_link".to_string());
    let payload_parameter_count = arg_value(&args, "--payload-parameters")
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(4);
    let payload_default_mass = parse_float_arg(&args, "--payload-default-mass", 0.5);
    let payload_fd_eps = parse_float_arg(&args, "--payload-fd-eps", 1e-5);
    let ignore_payload_link = arg_value(&args, "--ignore-payload-link");

    let urdf_path = parse_urdf_path(&args);
    let identification_urdf_path =
        if (mode == "full" || mode == "base") && ignore_payload_link.is_some() {
            let link_name = ignore_payload_link.as_ref().unwrap();
            let xml = fs::read_to_string(&urdf_path)?;
            let path = temp_urdf(
                &remove_link_inertial(&xml, link_name)?,
                &format!("ignore_payload_{link_name}"),
            )?;
            println!("[info] ignoring inertial of payload link for identification: {link_name}");
            path
        } else {
            urdf_path.clone()
        };
    let model = MathModel::load(&identification_urdf_path)?;
    let dataset = load_csv(&data_path, model.nq)?;
    let rows = dataset.samples * model.nq;

    let tau_pred: Vec<f64>;
    let mut beta: Option<Vec<f64>> = None;
    let mut dynamic_parameters: Option<Vec<f64>> = None;
    let mut selected_columns: Option<Vec<i32>> = None;
    let rank: i32;
    let condition: f64;
    let residual_norm: f64;
    let mut payload_fit: Option<PayloadFit> = None;

    if mode == "payload" {
        if include_friction {
            println!("[warn] --mode payload keeps arm/friction fixed; --no-friction is implied.");
        }
        let fit = fit_payload(
            &urdf_path,
            &dataset,
            &payload_link,
            payload_parameter_count,
            payload_default_mass,
            payload_fd_eps,
            rcond,
        )?;
        tau_pred = fit.tau_pred.clone();
        rank = fit.rank;
        condition = fit.condition;
        residual_norm = fit.residual_norm;
        payload_fit = Some(fit);
    } else if mode == "full" || mode == "base" {
        let y = model.build_regression_matrix(
            &dataset.q,
            &dataset.dq,
            &dataset.ddq,
            dataset.samples,
            include_friction,
            coulomb_eps,
        )?;
        let tau = model.stack_tau_samples(&dataset.tau, dataset.samples)?;
        let cols = model.num_total_parameters(include_friction)?;
        if mode == "full" {
            let (fit_beta, fit_tau_pred, info) =
                model.fit_least_squares(&y, rows, cols, &tau, rcond)?;
            let final_beta = if use_model_prior {
                let dyn_count = model.num_dynamic_parameters()?;
                let mut prior = vec![0.0; cols];
                let model_dynamic = model.model_dynamic_parameters()?;
                prior[..dyn_count].copy_from_slice(&model_dynamic[..dyn_count]);
                let prior_tau = mat_vec_mul(&y, rows, cols, &prior);
                let residual_tau = vec_sub(&tau, &prior_tau);
                let (delta, _delta_tau_pred, _delta_info) =
                    model.fit_least_squares(&y, rows, cols, &residual_tau, rcond)?;
                prior.iter().zip(delta.iter()).map(|(a, b)| a + b).collect()
            } else {
                fit_beta
            };
            tau_pred = if use_model_prior {
                mat_vec_mul(&y, rows, cols, &final_beta)
            } else {
                fit_tau_pred
            };
            let dyn_count = model.num_dynamic_parameters()?;
            dynamic_parameters = Some(final_beta[..dyn_count].to_vec());
            beta = Some(final_beta);
            rank = info.rank;
            condition = info.condition;
            residual_norm = norm(&vec_sub(&tau, &tau_pred));
            println!("beta length: {}", beta.as_ref().unwrap().len());
        } else {
            let (fit_beta, selected, fit_tau_pred, info) =
                model.fit_base_qr(&y, rows, cols, &tau, rcond)?;
            tau_pred = fit_tau_pred;
            beta = Some(fit_beta);
            selected_columns = Some(selected);
            rank = info.rank;
            condition = info.condition;
            residual_norm = info.residual_norm;
            println!("base beta length: {}", beta.as_ref().unwrap().len());
            println!("selected columns: {:?}", selected_columns.as_ref().unwrap());
        }
    } else {
        return Err("--mode must be full, base, or payload".into());
    }

    let tau = model.stack_tau_samples(&dataset.tau, dataset.samples)?;
    let (metrics, per_joint_rmse, per_joint_mae) = model.regression_metrics(&tau, &tau_pred)?;
    write_result_yaml(
        &output,
        &mode,
        &data_path,
        &urdf_path,
        &identification_urdf_path,
        dataset.samples,
        model.nq,
        rank,
        condition,
        residual_norm,
        metrics,
        &per_joint_rmse,
        &per_joint_mae,
        beta.as_deref(),
        dynamic_parameters.as_deref(),
        selected_columns.as_deref(),
        payload_fit.as_ref(),
        &payload_link,
        payload_parameter_count,
        payload_default_mass,
        payload_fd_eps,
        rcond,
        if mode == "payload" {
            false
        } else {
            include_friction
        },
        if mode == "full" {
            use_model_prior
        } else {
            false
        },
    )?;
    println!("[saved] {output}");
    println!(
        "fit mode={} samples={} rank={} cond={:.3e} rmse={:.6} mae={:.6} r2={:.6}",
        mode, dataset.samples, rank, condition, metrics.rmse, metrics.mae, metrics.r2
    );
    println!("per-joint rmse: {:?}", per_joint_rmse);

    if let Some(urdf_output) = urdf_output {
        if mode == "base" {
            return Err("--urdf-output requires --mode full or --mode payload; base parameters cannot be uniquely written to URDF".into());
        }
        if mode == "payload" {
            let fit = payload_fit.as_ref().ok_or("missing payload fit")?;
            let xml = fs::read_to_string(&urdf_path)?;
            let params = if payload_parameter_count == 4 {
                payload_params_with_preserved_com_inertia(
                    &fit.nominal_params,
                    &fit.payload_params[..4],
                )?
            } else {
                fit.payload_params.clone()
            };
            ensure_parent_dir(&urdf_output)?;
            fs::write(
                &urdf_output,
                replace_link_inertial(&xml, &payload_link, &params)?,
            )?;
            println!("[saved] {urdf_output}");
        } else {
            let params = dynamic_parameters
                .as_deref()
                .ok_or("missing full dynamic parameters")?;
            let xml = fs::read_to_string(&identification_urdf_path)?;
            ensure_parent_dir(&urdf_output)?;
            fs::write(
                &urdf_output,
                apply_full_dynamic_parameters_to_urdf(&xml, params)?,
            )?;
            println!("[saved] {urdf_output}");
        }
    }

    Ok(())
}
