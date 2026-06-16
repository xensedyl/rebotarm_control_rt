#include "example_common.hpp"
#include "rebotarm/dynamics.hpp"
#include "rebotarm/identification.hpp"

#include <Eigen/SVD>

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Dataset {
  Eigen::MatrixXd q;
  Eigen::MatrixXd dq;
  Eigen::MatrixXd ddq;
  Eigen::MatrixXd tau;
};

struct PayloadFit {
  Eigen::VectorXd beta;
  Eigen::VectorXd payload_params;
  Eigen::VectorXd nominal_params;
  Eigen::VectorXd tau_pred;
  int rank = 0;
  double condition = 0.0;
  double residual_norm = 0.0;
};

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    const auto start = cell.find_first_not_of(" \t\r\n");
    const auto end = cell.find_last_not_of(" \t\r\n");
    out.push_back(start == std::string::npos ? "" : cell.substr(start, end - start + 1));
  }
  return out;
}

int find_col(const std::vector<std::string>& header, const std::string& name) {
  for (int i = 0; i < static_cast<int>(header.size()); ++i) {
    if (header[i] == name) return i;
  }
  return -1;
}

int find_joint_col(const std::vector<std::string>& header, const std::string& prefix, int joint) {
  const std::vector<std::string> candidates = {
      prefix + std::to_string(joint),
      prefix + "_" + std::to_string(joint),
      prefix + ".joint_" + std::to_string(joint),
  };
  for (const auto& name : candidates) {
    const int col = find_col(header, name);
    if (col >= 0) return col;
  }
  return -1;
}

Dataset load_csv(const std::string& path, int dof) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open " + path);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty CSV");
  const auto header = split_csv(line);
  std::vector<int> q_cols, dq_cols, ddq_cols, tau_cols;
  for (int i = 1; i <= dof; ++i) {
    q_cols.push_back(find_joint_col(header, "q", i));
    dq_cols.push_back(find_joint_col(header, "dq", i));
    ddq_cols.push_back(find_joint_col(header, "ddq", i));
    tau_cols.push_back(find_joint_col(header, "tau", i));
  }
  for (const auto& cols : {q_cols, dq_cols, ddq_cols, tau_cols}) {
    for (int col : cols) {
      if (col < 0) throw std::runtime_error("CSV missing required q/dq/ddq/tau columns");
    }
  }

  std::vector<std::vector<double>> rows;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto cells = split_csv(line);
    std::vector<double> row;
    row.reserve(cells.size());
    for (const auto& cell : cells) row.push_back(std::stod(cell));
    rows.push_back(row);
  }
  Dataset ds;
  ds.q.resize(rows.size(), dof);
  ds.dq.resize(rows.size(), dof);
  ds.ddq.resize(rows.size(), dof);
  ds.tau.resize(rows.size(), dof);
  for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
    for (int j = 0; j < dof; ++j) {
      ds.q(r, j) = rows[r][q_cols[j]];
      ds.dq(r, j) = rows[r][dq_cols[j]];
      ds.ddq(r, j) = rows[r][ddq_cols[j]];
      ds.tau(r, j) = rows[r][tau_cols[j]];
    }
  }
  return ds;
}

void ensure_parent_dir(const std::string& path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

std::string read_text(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open " + path);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

void write_text(const std::string& path, const std::string& text) {
  ensure_parent_dir(path);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("failed to write " + path);
  out << text;
}

std::string yaml_float(double value) {
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}

void write_yaml_vector(std::ostream& out, const std::string& name, const Eigen::VectorXd& values) {
  out << name << ": [";
  for (int i = 0; i < values.size(); ++i) {
    if (i) out << ", ";
    out << yaml_float(values[i]);
  }
  out << "]\n";
}

void write_yaml_metrics(std::ostream& out, const rebotarm::ident::RegressionMetrics& metrics) {
  out << "metrics:\n";
  out << "  rmse: " << yaml_float(metrics.rmse) << "\n";
  out << "  mae: " << yaml_float(metrics.mae) << "\n";
  out << "  max_abs: " << yaml_float(metrics.max_abs) << "\n";
  out << "  r2: " << yaml_float(metrics.r2) << "\n";
  out << "  per_joint_rmse: [";
  for (int i = 0; i < metrics.per_joint_rmse.size(); ++i) {
    if (i) out << ", ";
    out << yaml_float(metrics.per_joint_rmse[i]);
  }
  out << "]\n";
  out << "  per_joint_mae: [";
  for (int i = 0; i < metrics.per_joint_mae.size(); ++i) {
    if (i) out << ", ";
    out << yaml_float(metrics.per_joint_mae[i]);
  }
  out << "]\n";
}

std::optional<size_t> find_link_start(const std::string& xml, const std::string& link_name) {
  const std::string needle1 = "name=\"" + link_name + "\"";
  const std::string needle2 = "name='" + link_name + "'";
  size_t pos = xml.find(needle1);
  if (pos == std::string::npos) pos = xml.find(needle2);
  if (pos == std::string::npos) return std::nullopt;
  return xml.rfind("<link", pos);
}

std::pair<size_t, size_t> link_range(const std::string& xml, const std::string& link_name) {
  const auto start_opt = find_link_start(xml, link_name);
  if (!start_opt) throw std::runtime_error("URDF link not found: " + link_name);
  const size_t start = *start_opt;
  const size_t end = xml.find("</link>", start);
  if (end == std::string::npos) throw std::runtime_error("URDF link is not closed: " + link_name);
  return {start, end + std::string("</link>").size()};
}

std::optional<std::pair<size_t, size_t>> inertial_range_in_link(const std::string& link_text) {
  const size_t start = link_text.find("<inertial");
  if (start == std::string::npos) return std::nullopt;
  const size_t end = link_text.find("</inertial>", start);
  if (end == std::string::npos) throw std::runtime_error("URDF inertial block is not closed");
  return std::make_pair(start, end + std::string("</inertial>").size());
}

std::optional<std::string> tag_text(const std::string& source, const std::string& tag) {
  const size_t start = source.find("<" + tag);
  if (start == std::string::npos) return std::nullopt;
  const size_t end = source.find('>', start);
  if (end == std::string::npos) throw std::runtime_error("URDF <" + tag + "> tag is malformed");
  return source.substr(start, end - start + 1);
}

std::optional<std::string> attr_value(const std::string& tag, const std::string& attr) {
  size_t offset = tag.find(attr + "=\"");
  char quote = '"';
  if (offset == std::string::npos) {
    offset = tag.find(attr + "='");
    quote = '\'';
  }
  if (offset == std::string::npos) return std::nullopt;
  const size_t value_start = offset + attr.size() + 2;
  const size_t value_end = tag.find(quote, value_start);
  if (value_end == std::string::npos) throw std::runtime_error("URDF attribute quote is not closed: " + attr);
  return tag.substr(value_start, value_end - value_start);
}

std::array<double, 3> parse_xyz(const std::string& text) {
  std::istringstream in(text);
  std::array<double, 3> out{0.0, 0.0, 0.0};
  in >> out[0] >> out[1] >> out[2];
  return out;
}

Eigen::Matrix3d symmetric_from_params(const Eigen::VectorXd& values) {
  Eigen::Matrix3d out;
  out << values[0], values[1], values[3],
         values[1], values[2], values[4],
         values[3], values[4], values[5];
  return out;
}

Eigen::VectorXd symmetric_to_params(const Eigen::Matrix3d& matrix) {
  Eigen::VectorXd out(6);
  out << matrix(0, 0), matrix(0, 1), matrix(1, 1), matrix(0, 2), matrix(1, 2), matrix(2, 2);
  return out;
}

Eigen::Matrix3d parallel_axis(double mass, const Eigen::Vector3d& com) {
  return mass * (com.squaredNorm() * Eigen::Matrix3d::Identity() - com * com.transpose());
}

Eigen::VectorXd dynamic_params_from_inertia(double mass, const Eigen::Vector3d& com,
                                            const Eigen::Matrix3d& inertia_at_com) {
  Eigen::VectorXd params(10);
  params.setZero();
  params[0] = mass;
  params.segment<3>(1) = mass * com;
  params.segment<6>(4) = symmetric_to_params(inertia_at_com + parallel_axis(mass, com));
  return params;
}

void inertia_from_dynamic_params(const Eigen::VectorXd& params, double& mass,
                                 Eigen::Vector3d& com, Eigen::Matrix3d& inertia_at_com) {
  if (params.size() != 10) throw std::runtime_error("one inertial block must contain 10 parameters");
  mass = params[0];
  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::runtime_error("identified mass must be positive: " + std::to_string(mass));
  }
  com = params.segment<3>(1) / mass;
  inertia_at_com = symmetric_from_params(params.segment<6>(4)) - parallel_axis(mass, com);
}

Eigen::VectorXd default_payload_params(double mass) {
  if (!std::isfinite(mass) || mass <= 0.0) throw std::runtime_error("default payload mass must be positive");
  Eigen::VectorXd params(10);
  params << mass, 0.0, 0.0, 0.0, 1e-5, 0.0, 1e-5, 0.0, 0.0, 1e-5;
  return params;
}

Eigen::VectorXd dynamic_params_from_link_xml(const std::string& xml, const std::string& link_name,
                                             double default_mass) {
  const auto [link_start, link_end] = link_range(xml, link_name);
  const std::string link_text = xml.substr(link_start, link_end - link_start);
  const auto inertial = inertial_range_in_link(link_text);
  if (!inertial) return default_payload_params(default_mass);
  const std::string block = link_text.substr(inertial->first, inertial->second - inertial->first);
  const auto mass_tag = tag_text(block, "mass");
  const auto inertia_tag = tag_text(block, "inertia");
  if (!mass_tag || !inertia_tag) return default_payload_params(default_mass);
  const double mass = std::stod(attr_value(*mass_tag, "value").value_or("0"));
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  if (const auto origin_tag = tag_text(block, "origin")) {
    if (const auto xyz = attr_value(*origin_tag, "xyz")) {
      const auto parsed = parse_xyz(*xyz);
      com << parsed[0], parsed[1], parsed[2];
    }
  }
  Eigen::Matrix3d inertia_at_com = Eigen::Matrix3d::Zero();
  inertia_at_com(0, 0) = std::stod(attr_value(*inertia_tag, "ixx").value_or("0"));
  inertia_at_com(0, 1) = std::stod(attr_value(*inertia_tag, "ixy").value_or("0"));
  inertia_at_com(1, 0) = inertia_at_com(0, 1);
  inertia_at_com(0, 2) = std::stod(attr_value(*inertia_tag, "ixz").value_or("0"));
  inertia_at_com(2, 0) = inertia_at_com(0, 2);
  inertia_at_com(1, 1) = std::stod(attr_value(*inertia_tag, "iyy").value_or("0"));
  inertia_at_com(1, 2) = std::stod(attr_value(*inertia_tag, "iyz").value_or("0"));
  inertia_at_com(2, 1) = inertia_at_com(1, 2);
  inertia_at_com(2, 2) = std::stod(attr_value(*inertia_tag, "izz").value_or("0"));
  return dynamic_params_from_inertia(mass, com, inertia_at_com);
}

Eigen::VectorXd payload_params_with_preserved_com_inertia(const Eigen::VectorXd& base_params,
                                                          const Eigen::VectorXd& payload_params) {
  double base_mass = 0.0;
  Eigen::Vector3d base_com;
  Eigen::Matrix3d base_inertia_at_com;
  inertia_from_dynamic_params(base_params, base_mass, base_com, base_inertia_at_com);
  const double mass = payload_params[0];
  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::runtime_error("identified payload mass must be positive: " + std::to_string(mass));
  }
  const Eigen::Vector3d com = payload_params.segment<3>(1) / mass;
  return dynamic_params_from_inertia(mass, com, base_inertia_at_com * (mass / base_mass));
}

std::string format_inertial_block(const Eigen::VectorXd& params, const std::string& indent,
                                  const std::string& rpy) {
  double mass = 0.0;
  Eigen::Vector3d com;
  Eigen::Matrix3d ic;
  inertia_from_dynamic_params(params, mass, com, ic);
  const std::string child = indent + "  ";
  const std::string attr = indent + "    ";
  std::ostringstream out;
  out << indent << "<inertial>\n";
  out << child << "<origin\n";
  out << attr << "xyz=\"" << example::format_float(com[0]) << " " << example::format_float(com[1]) << " "
      << example::format_float(com[2]) << "\"\n";
  out << attr << "rpy=\"" << rpy << "\" />\n";
  out << child << "<mass\n";
  out << attr << "value=\"" << example::format_float(mass) << "\" />\n";
  out << child << "<inertia\n";
  out << attr << "ixx=\"" << example::format_float(ic(0, 0)) << "\"\n";
  out << attr << "ixy=\"" << example::format_float(ic(0, 1)) << "\"\n";
  out << attr << "ixz=\"" << example::format_float(ic(0, 2)) << "\"\n";
  out << attr << "iyy=\"" << example::format_float(ic(1, 1)) << "\"\n";
  out << attr << "iyz=\"" << example::format_float(ic(1, 2)) << "\"\n";
  out << attr << "izz=\"" << example::format_float(ic(2, 2)) << "\" />\n";
  out << indent << "</inertial>";
  return out.str();
}

std::string leading_indent(const std::string& text, size_t pos) {
  size_t line_start = text.rfind('\n', pos);
  line_start = line_start == std::string::npos ? 0 : line_start + 1;
  return text.substr(line_start, pos - line_start);
}

std::string replace_link_inertial(const std::string& xml, const std::string& link_name,
                                  const Eigen::VectorXd& params) {
  const auto [link_start, link_end] = link_range(xml, link_name);
  const std::string link_text = xml.substr(link_start, link_end - link_start);
  const auto inertial = inertial_range_in_link(link_text);
  if (!inertial) throw std::runtime_error("URDF link has no inertial: " + link_name);
  const std::string inertial_text = link_text.substr(inertial->first, inertial->second - inertial->first);
  std::string rpy = "0 0 0";
  if (const auto origin = tag_text(inertial_text, "origin")) {
    rpy = attr_value(*origin, "rpy").value_or("0 0 0");
  }
  const std::string indent = leading_indent(link_text, inertial->first);
  const std::string new_inertial = format_inertial_block(params, indent, rpy);
  std::string new_link = link_text.substr(0, inertial->first) + new_inertial +
                         link_text.substr(inertial->second);
  return xml.substr(0, link_start) + new_link + xml.substr(link_end);
}

std::string remove_link_inertial(const std::string& xml, const std::string& link_name) {
  const auto [link_start, link_end] = link_range(xml, link_name);
  const std::string link_text = xml.substr(link_start, link_end - link_start);
  const auto inertial = inertial_range_in_link(link_text);
  if (!inertial) return xml;
  std::string new_link = link_text.substr(0, inertial->first) + link_text.substr(inertial->second);
  return xml.substr(0, link_start) + new_link + xml.substr(link_end);
}

std::optional<std::string> child_link_from_joint_text(const std::string& joint_text) {
  const auto child = tag_text(joint_text, "child");
  if (!child) return std::nullopt;
  return attr_value(*child, "link");
}

std::optional<std::string> parent_link_from_joint_text(const std::string& joint_text) {
  const auto parent = tag_text(joint_text, "parent");
  if (!parent) return std::nullopt;
  return attr_value(*parent, "link");
}

std::vector<std::string> movable_joint_child_links(const std::string& xml) {
  std::vector<std::string> links;
  size_t pos = 0;
  while (true) {
    const size_t start = xml.find("<joint", pos);
    if (start == std::string::npos) break;
    const size_t end = xml.find("</joint>", start);
    if (end == std::string::npos) throw std::runtime_error("URDF joint block is not closed");
    const std::string joint_text = xml.substr(start, end + std::string("</joint>").size() - start);
    const auto joint_open = tag_text(joint_text, "joint");
    const bool is_fixed = joint_open && attr_value(*joint_open, "type").value_or("") == "fixed";
    if (!is_fixed) {
      if (const auto child = child_link_from_joint_text(joint_text)) links.push_back(*child);
    }
    pos = end + std::string("</joint>").size();
  }
  return links;
}

std::vector<std::string> fixed_descendant_links(const std::string& xml,
                                                const std::vector<std::string>& parents) {
  std::vector<std::pair<std::string, std::string>> edges;
  size_t pos = 0;
  while (true) {
    const size_t start = xml.find("<joint", pos);
    if (start == std::string::npos) break;
    const size_t end = xml.find("</joint>", start);
    if (end == std::string::npos) throw std::runtime_error("URDF joint block is not closed");
    const std::string joint_text = xml.substr(start, end + std::string("</joint>").size() - start);
    const auto joint_open = tag_text(joint_text, "joint");
    const bool is_fixed = joint_open && attr_value(*joint_open, "type").value_or("") == "fixed";
    if (is_fixed) {
      const auto parent = parent_link_from_joint_text(joint_text);
      const auto child = child_link_from_joint_text(joint_text);
      if (parent && child) edges.emplace_back(*parent, *child);
    }
    pos = end + std::string("</joint>").size();
  }

  std::vector<std::string> stack = parents;
  std::vector<std::string> out;
  std::vector<std::string> seen = parents;
  while (!stack.empty()) {
    const std::string parent = stack.back();
    stack.pop_back();
    for (const auto& [edge_parent, edge_child] : edges) {
      if (edge_parent != parent) continue;
      if (std::find(seen.begin(), seen.end(), edge_child) != seen.end()) continue;
      seen.push_back(edge_child);
      out.push_back(edge_child);
      stack.push_back(edge_child);
    }
  }
  return out;
}

std::string remove_link_inertials(const std::string& xml, const std::vector<std::string>& link_names) {
  std::string out = xml;
  for (const auto& link_name : link_names) {
    out = remove_link_inertial(out, link_name);
  }
  return out;
}

std::string apply_full_dynamic_parameters_to_urdf(const std::string& xml,
                                                  const Eigen::VectorXd& dynamic_parameters) {
  if (dynamic_parameters.size() % 10 != 0) {
    throw std::runtime_error("dynamic parameter vector length must be a multiple of 10");
  }
  const auto links = movable_joint_child_links(xml);
  const int blocks = dynamic_parameters.size() / 10;
  if (static_cast<int>(links.size()) != blocks) {
    throw std::runtime_error("URDF movable joint link count does not match dynamic parameter blocks");
  }
  std::string out = xml;
  for (int i = 0; i < blocks; ++i) {
    out = replace_link_inertial(out, links[i], dynamic_parameters.segment<10>(i * 10));
  }
  out = remove_link_inertials(out, fixed_descendant_links(out, links));
  return out;
}

std::vector<int> payload_indices(int parameter_count) {
  if (parameter_count == 4) return {0, 1, 2, 3};
  if (parameter_count == 10) return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  throw std::runtime_error("--payload-parameters must be 4 or 10");
}

std::vector<std::string> payload_parameter_names(const std::string& link_name, int parameter_count) {
  const std::vector<std::string> fields = {"m", "mcx", "mcy", "mcz", "ixx", "ixy", "iyy", "ixz", "iyz", "izz"};
  std::vector<std::string> out;
  for (int idx : payload_indices(parameter_count)) out.push_back(link_name + "." + fields[idx]);
  return out;
}

Eigen::MatrixXd inverse_dynamics_samples(rebotarm::RobotModel& model, const Dataset& ds) {
  Eigen::MatrixXd tau(ds.q.rows(), ds.q.cols());
  for (int i = 0; i < ds.q.rows(); ++i) {
    tau.row(i) = rebotarm::dyn::inverse_dynamics(
                     model,
                     ds.q.row(i).transpose(),
                     ds.dq.row(i).transpose(),
                     ds.ddq.row(i).transpose())
                     .transpose();
  }
  return tau;
}

example::TemporaryUrdf make_temp_urdf(const std::string& text, const std::string& label) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = (std::filesystem::temp_directory_path() /
                     ("rebotarm_control_rt_" + label + "_" + std::to_string(now) + ".urdf"))
                        .string();
  write_text(path, text);
  return example::TemporaryUrdf(path);
}

PayloadFit fit_payload(const std::string& urdf_path, const Dataset& ds, const std::string& link_name,
                       int parameter_count, double default_mass, double fd_eps, double rcond) {
  if (fd_eps <= 0.0) throw std::runtime_error("--payload-fd-eps must be positive");
  const std::string xml = read_text(urdf_path);
  const Eigen::VectorXd nominal_params = dynamic_params_from_link_xml(xml, link_name, default_mass);
  const auto indices = payload_indices(parameter_count);

  auto arm_only_tmp = make_temp_urdf(remove_link_inertial(xml, link_name), "payload_arm_only");
  auto nominal_tmp = make_temp_urdf(replace_link_inertial(xml, link_name, nominal_params), "payload_nominal");
  rebotarm::RobotModel arm_model(arm_only_tmp.path());
  rebotarm::RobotModel nominal_model(nominal_tmp.path());
  (void)inverse_dynamics_samples(arm_model, ds);
  const Eigen::MatrixXd tau_nominal = inverse_dynamics_samples(nominal_model, ds);

  Eigen::MatrixXd Y(ds.q.rows() * ds.q.cols(), static_cast<int>(indices.size()));
  for (int col = 0; col < static_cast<int>(indices.size()); ++col) {
    const int param_index = indices[col];
    Eigen::VectorXd perturbed = nominal_params;
    const double scale = std::max(std::abs(nominal_params[param_index]), 1.0);
    double step = fd_eps * scale;
    if (param_index == 0) step = std::max(step, fd_eps);
    perturbed[param_index] += step;
    if (perturbed[0] <= 0.0) throw std::runtime_error("payload mass perturbation became non-positive");
    auto perturbed_tmp = make_temp_urdf(replace_link_inertial(xml, link_name, perturbed),
                                        "payload_perturbed_" + std::to_string(col));
    rebotarm::RobotModel perturbed_model(perturbed_tmp.path());
    const Eigen::MatrixXd tau_perturbed = inverse_dynamics_samples(perturbed_model, ds);
    Eigen::VectorXd column = ((tau_perturbed - tau_nominal) / step).reshaped<Eigen::RowMajor>();
    Y.col(col) = column;
  }

  const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(ds.tau);
  const Eigen::VectorXd tau_nominal_flat = tau_nominal.reshaped<Eigen::RowMajor>();
  Eigen::VectorXd nominal_selected(indices.size());
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) nominal_selected[i] = nominal_params[indices[i]];
  const Eigen::VectorXd tau_fixed = tau_nominal_flat - Y * nominal_selected;
  const Eigen::VectorXd residual_tau = tau - tau_fixed;
  const auto fit = rebotarm::ident::fit_least_squares(Y, residual_tau, rcond);

  Eigen::VectorXd payload_params = nominal_params;
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) payload_params[indices[i]] = fit.beta[i];

  PayloadFit out;
  out.beta = fit.beta;
  out.payload_params = payload_params;
  out.nominal_params = nominal_params;
  out.tau_pred = tau_fixed + fit.tau_pred;
  out.rank = fit.rank;
  out.condition = fit.condition;
  out.residual_norm = (tau - out.tau_pred).norm();
  return out;
}

Eigen::VectorXd full_fit_with_prior(const Eigen::MatrixXd& Y, const Eigen::VectorXd& tau,
                                    const Eigen::VectorXd& prior, double rcond) {
  Eigen::BDCSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd s = svd.singularValues();
  const double max_s = s.size() > 0 ? s[0] : 0.0;
  const double tol = std::max(Y.rows(), Y.cols()) * rcond * max_s;
  Eigen::VectorXd inv_s = Eigen::VectorXd::Zero(s.size());
  for (int i = 0; i < s.size(); ++i) {
    if (s[i] > tol) inv_s[i] = 1.0 / s[i];
  }
  return prior + svd.matrixV() * inv_s.asDiagonal() * svd.matrixU().transpose() * (tau - Y * prior);
}

void write_result_yaml(const std::string& path, const std::string& mode, const std::string& data_path,
                       const std::string& input_urdf, const std::string& identification_urdf,
                       int samples, int dof, int rank, double condition, double residual_norm,
                       const rebotarm::ident::RegressionMetrics& metrics,
                       const Eigen::VectorXd* beta, const Eigen::VectorXd* dynamic_params,
                       const std::vector<int>* selected_columns,
                       const PayloadFit* payload, const std::string& payload_link,
                       int payload_parameter_count, double default_mass, double fd_eps,
                       double rcond, bool include_friction, bool use_model_prior) {
  ensure_parent_dir(path);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("failed to write " + path);
  out << "mode: " << mode << "\n";
  out << "samples: " << samples << "\n";
  out << "dof: " << dof << "\n";
  out << "input_data: " << data_path << "\n";
  out << "input_urdf: " << input_urdf << "\n";
  out << "identification_urdf: " << identification_urdf << "\n";
  out << "include_friction: " << (include_friction ? "true" : "false") << "\n";
  out << "use_model_prior: " << (use_model_prior ? "true" : "false") << "\n";
  out << "rcond: " << yaml_float(rcond) << "\n";
  out << "rank: " << rank << "\n";
  out << "condition: " << yaml_float(condition) << "\n";
  out << "residual_norm: " << yaml_float(residual_norm) << "\n";
  write_yaml_metrics(out, metrics);
  if (beta) write_yaml_vector(out, "beta", *beta);
  if (dynamic_params) write_yaml_vector(out, "dynamic_parameters", *dynamic_params);
  if (selected_columns) {
    out << "selected_columns: [";
    for (std::size_t i = 0; i < selected_columns->size(); ++i) {
      if (i) out << ", ";
      out << (*selected_columns)[i];
    }
    out << "]\n";
  }
  if (payload) {
    out << "payload_link: " << payload_link << "\n";
    out << "payload_parameter_count: " << payload_parameter_count << "\n";
    out << "payload_parameter_names: [";
    const auto names = payload_parameter_names(payload_link, payload_parameter_count);
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (i) out << ", ";
      out << names[i];
    }
    out << "]\n";
    write_yaml_vector(out, "payload_beta", payload->beta);
    write_yaml_vector(out, "payload_dynamic_parameters", payload->payload_params);
    write_yaml_vector(out, "nominal_payload_dynamic_parameters", payload->nominal_params);
    out << "default_mass: " << yaml_float(default_mass) << "\n";
    out << "finite_difference_eps: " << yaml_float(fd_eps) << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (example::has_flag(argc, argv, "--help") || example::has_flag(argc, argv, "-h")) {
      std::cout << "Usage: ./13_identify_dynamics --data calibration/id_data_train.csv "
                   "[--mode full|base|payload] [--urdf robot.urdf] "
                   "[--output calibration/identified.yaml] [--urdf-output out.urdf]\n";
      return 0;
    }
    const std::string data_path = example::arg_value(argc, argv, "--data");
    if (data_path.empty()) throw std::runtime_error("--data is required");
    const std::string mode = example::arg_value(argc, argv, "--mode", "full");
    const std::string output_path = example::arg_value(argc, argv, "--output", "calibration/identified_dynamics_cpp.yaml");
    const std::string urdf_output = example::arg_value(argc, argv, "--urdf-output");
    const std::string urdf_path = example::urdf_arg(argc, argv);
    const bool include_friction = !example::has_flag(argc, argv, "--no-friction");
    const bool use_model_prior = !example::has_flag(argc, argv, "--no-model-prior");
    const double coulomb_eps = example::arg_double(argc, argv, "--coulomb-eps", 1e-3);
    const double rcond = example::arg_double(argc, argv, "--rcond", 1e-12);
    const std::string payload_link = example::arg_value(argc, argv, "--payload-link", "end_link");
    const int payload_parameters = example::arg_int(argc, argv, "--payload-parameters", 4);
    const double payload_default_mass = example::arg_double(argc, argv, "--payload-default-mass", 0.5);
    const double payload_fd_eps = example::arg_double(argc, argv, "--payload-fd-eps", 1e-5);
    const std::string ignore_payload_link = example::arg_value(argc, argv, "--ignore-payload-link");

    std::string identification_urdf_path = urdf_path;
    std::unique_ptr<example::TemporaryUrdf> ignore_payload_tmp;
    if ((mode == "full" || mode == "base") && !ignore_payload_link.empty()) {
      const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
      identification_urdf_path =
          (std::filesystem::temp_directory_path() /
           ("rebotarm_control_rt_ignore_payload_" + ignore_payload_link + "_" +
            std::to_string(now) + ".urdf"))
              .string();
      write_text(identification_urdf_path, remove_link_inertial(read_text(urdf_path), ignore_payload_link));
      ignore_payload_tmp = std::make_unique<example::TemporaryUrdf>(identification_urdf_path);
      std::cout << "[info] ignoring inertial of payload link for identification: "
                << ignore_payload_link << "\n";
    }

    rebotarm::RobotModel base_model(identification_urdf_path);
    Dataset ds = load_csv(data_path, base_model.nv());

    Eigen::VectorXd tau_pred;
    Eigen::VectorXd beta;
    Eigen::VectorXd dynamic_parameters;
    std::vector<int> selected_columns;
    int rank = 0;
    double condition = 0.0;
    double residual = 0.0;
    std::unique_ptr<PayloadFit> payload_fit;

    if (mode == "payload") {
      if (include_friction) {
        std::cout << "[warn] --mode payload keeps arm/friction fixed; --no-friction is implied.\n";
      }
      payload_fit = std::make_unique<PayloadFit>(
          fit_payload(urdf_path, ds, payload_link, payload_parameters, payload_default_mass,
                      payload_fd_eps, rcond));
      tau_pred = payload_fit->tau_pred;
      rank = payload_fit->rank;
      condition = payload_fit->condition;
      residual = payload_fit->residual_norm;
    } else if (mode == "full" || mode == "base") {
      const Eigen::MatrixXd Y = rebotarm::ident::build_regression_matrix(
          base_model, ds.q, ds.dq, ds.ddq, include_friction, coulomb_eps);
      const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(ds.tau);
      if (mode == "full") {
        const auto fit = rebotarm::ident::fit_least_squares(Y, tau, rcond);
        beta = fit.beta;
        if (use_model_prior) {
          const int dyn_count = rebotarm::ident::num_dynamic_parameters(base_model);
          Eigen::VectorXd prior = Eigen::VectorXd::Zero(Y.cols());
          prior.head(dyn_count) = rebotarm::ident::model_dynamic_parameters(base_model);
          beta = full_fit_with_prior(Y, tau, prior, rcond);
          tau_pred = Y * beta;
        } else {
          tau_pred = fit.tau_pred;
        }
        rank = fit.rank;
        condition = fit.condition;
        residual = (tau - tau_pred).norm();
        const int dyn_count = rebotarm::ident::num_dynamic_parameters(base_model);
        dynamic_parameters = beta.head(dyn_count);
        std::cout << "beta length: " << beta.size() << "\n";
      } else {
        const auto fit = rebotarm::ident::fit_base_parameters_qr(Y, tau, rcond);
        beta = fit.beta_base;
        tau_pred = fit.tau_pred;
        rank = fit.rank;
        condition = fit.condition;
        residual = fit.residual_norm;
        selected_columns.reserve(fit.selected_columns.size());
        for (int i = 0; i < fit.selected_columns.size(); ++i) selected_columns.push_back(fit.selected_columns[i]);
        std::cout << "base beta length: " << beta.size() << "\n";
        std::cout << "selected columns:";
        for (int col : selected_columns) std::cout << " " << col;
        std::cout << "\n";
      }
    } else {
      throw std::runtime_error("--mode must be full, base, or payload");
    }

    const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(ds.tau);
    const auto metrics = rebotarm::ident::regression_metrics(tau, tau_pred, base_model.nv());
    write_result_yaml(output_path, mode, data_path, urdf_path, identification_urdf_path,
                      ds.q.rows(), ds.q.cols(),
                      rank, condition, residual, metrics,
                      beta.size() ? &beta : nullptr,
                      dynamic_parameters.size() ? &dynamic_parameters : nullptr,
                      selected_columns.empty() ? nullptr : &selected_columns,
                      payload_fit.get(), payload_link, payload_parameters,
                      payload_default_mass, payload_fd_eps, rcond,
                      mode == "payload" ? false : include_friction,
                      mode == "full" ? use_model_prior : false);
    std::cout << "[saved] " << output_path << "\n";
    std::cout << "fit mode=" << mode << " samples=" << ds.q.rows() << " rank=" << rank
              << " cond=" << condition << " rmse=" << metrics.rmse << " mae=" << metrics.mae
              << " r2=" << metrics.r2 << "\n";
    std::cout << "per-joint rmse: " << metrics.per_joint_rmse.transpose() << "\n";

    if (!urdf_output.empty()) {
      if (mode == "base") {
        throw std::runtime_error("--urdf-output requires --mode full or --mode payload; base parameters cannot be uniquely written to URDF");
      }
      if (mode == "payload") {
        Eigen::VectorXd params = payload_fit->payload_params;
        if (payload_parameters == 4) {
          params = payload_params_with_preserved_com_inertia(payload_fit->nominal_params, payload_fit->payload_params.head(4));
        }
        write_text(urdf_output, replace_link_inertial(read_text(urdf_path), payload_link, params));
      } else {
        write_text(urdf_output,
                   apply_full_dynamic_parameters_to_urdf(read_text(identification_urdf_path),
                                                         dynamic_parameters));
      }
      std::cout << "[saved] " << urdf_output << "\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
