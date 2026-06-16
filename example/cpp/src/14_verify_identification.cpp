#include "example_common.hpp"
#include "rebotarm/identification.hpp"

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

std::vector<double> load_beta_txt(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open " + path);
  std::vector<double> values;
  double value = 0.0;
  while (in >> value) values.push_back(value);
  return values;
}

std::string read_text_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open " + path);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

std::string trim(const std::string& text) {
  const auto start = text.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(start, end - start + 1);
}

std::string unquote(std::string text) {
  text = trim(text);
  if (text.size() >= 2 && ((text.front() == '"' && text.back() == '"') ||
                           (text.front() == '\'' && text.back() == '\''))) {
    return text.substr(1, text.size() - 2);
  }
  return text;
}

std::optional<std::string> yaml_value_tail(const std::string& yaml, const std::string& key) {
  std::istringstream in(yaml);
  std::string line;
  const std::string prefix = key + ":";
  while (std::getline(in, line)) {
    const std::string stripped = trim(line);
    if (stripped.rfind(prefix, 0) == 0) return trim(stripped.substr(prefix.size()));
  }
  return std::nullopt;
}

std::string yaml_string(const std::string& yaml, const std::string& key,
                        const std::string& default_value = "") {
  const auto value = yaml_value_tail(yaml, key);
  return value ? unquote(*value) : default_value;
}

bool yaml_bool(const std::string& yaml, const std::string& key, bool default_value) {
  const auto value = yaml_value_tail(yaml, key);
  if (!value) return default_value;
  const std::string lower = [&] {
    std::string out = trim(*value);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
  }();
  return lower == "true" || lower == "1" || lower == "yes" || lower == "on";
}

std::vector<double> parse_number_list_body(std::string body) {
  for (char& ch : body) {
    if (ch == ',' || ch == '[' || ch == ']') ch = ' ';
  }
  std::istringstream in(body);
  std::vector<double> values;
  double value = 0.0;
  while (in >> value) values.push_back(value);
  return values;
}

std::vector<double> yaml_float_vector(const std::string& yaml, const std::string& key) {
  std::istringstream in(yaml);
  std::string line;
  const std::string prefix = key + ":";
  bool in_block = false;
  std::string body;
  while (std::getline(in, line)) {
    const std::string stripped = trim(line);
    if (!in_block && stripped.rfind(prefix, 0) == 0) {
      std::string tail = trim(stripped.substr(prefix.size()));
      if (tail.find('[') != std::string::npos) {
        body += tail;
        while (body.find(']') == std::string::npos && std::getline(in, line)) body += " " + trim(line);
        return parse_number_list_body(body);
      }
      in_block = true;
      continue;
    }
    if (in_block) {
      if (stripped.rfind("- ", 0) != 0) break;
      body += " " + stripped.substr(2);
    }
  }
  return parse_number_list_body(body);
}

std::vector<int> yaml_int_vector(const std::string& yaml, const std::string& key) {
  const auto floats = yaml_float_vector(yaml, key);
  std::vector<int> values;
  values.reserve(floats.size());
  for (double value : floats) values.push_back(static_cast<int>(value));
  return values;
}

Eigen::VectorXd vector_to_eigen(const std::vector<double>& values) {
  Eigen::VectorXd out(values.size());
  for (int i = 0; i < out.size(); ++i) out[i] = values[i];
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (example::has_flag(argc, argv, "--help") || example::has_flag(argc, argv, "-h")) {
      std::cout << "Usage: ./14_verify_identification --data id_verify.csv "
                   "--params identified.yaml [--urdf robot.urdf]\n"
                   "   or: ./14_verify_identification --data id_verify.csv "
                   "--beta beta.txt [--urdf robot.urdf] [--no-friction]\n";
      return 0;
    }
    const std::string data_path = example::arg_value(argc, argv, "--data");
    const std::string params_path = example::arg_value(argc, argv, "--params");
    const std::string beta_path = example::arg_value(argc, argv, "--beta");
    if (data_path.empty() || (params_path.empty() && beta_path.empty())) {
      throw std::runtime_error("--data and either --params or --beta are required");
    }

    std::string yaml;
    std::string mode = "full";
    bool include_friction = !example::has_flag(argc, argv, "--no-friction");
    double coulomb_eps = example::arg_double(argc, argv, "--coulomb-eps", 1e-3);
    std::vector<double> beta_values;
    std::vector<int> selected_columns;
    std::string urdf_path = example::arg_value(argc, argv, "--urdf");

    if (!params_path.empty()) {
      yaml = read_text_file(params_path);
      mode = yaml_string(yaml, "mode", "full");
      include_friction = yaml_bool(yaml, "include_friction", true);
      if (example::has_flag(argc, argv, "--no-friction")) include_friction = false;
      const std::string coulomb = yaml_string(yaml, "coulomb_eps");
      if (!coulomb.empty()) coulomb_eps = std::stod(coulomb);
      if (urdf_path.empty()) urdf_path = yaml_string(yaml, "input_urdf", example::default_urdf_path());
      if (mode == "base") {
        beta_values = yaml_float_vector(yaml, "beta_base");
        if (beta_values.empty()) beta_values = yaml_float_vector(yaml, "beta");
        selected_columns = yaml_int_vector(yaml, "selected_columns");
      } else if (mode == "full") {
        beta_values = yaml_float_vector(yaml, "beta");
      } else {
        throw std::runtime_error("--params must contain mode full or base");
      }
    } else {
      beta_values = load_beta_txt(beta_path);
      if (urdf_path.empty()) urdf_path = example::urdf_arg(argc, argv);
    }
    if (beta_values.empty()) throw std::runtime_error("no beta values found");

    rebotarm::RobotModel model(urdf_path);
    Dataset ds = load_csv(data_path, model.nv());
    const Eigen::MatrixXd Y = rebotarm::ident::build_regression_matrix(
        model, ds.q, ds.dq, ds.ddq, include_friction, coulomb_eps);
    const Eigen::VectorXd beta = vector_to_eigen(beta_values);

    const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(ds.tau);
    Eigen::VectorXd tau_pred;
    if (mode == "base") {
      if (selected_columns.empty()) throw std::runtime_error("base params require selected_columns");
      if (beta.size() != static_cast<int>(selected_columns.size())) {
        throw std::runtime_error("base beta length does not match selected_columns");
      }
      Eigen::MatrixXd Y_selected(Y.rows(), selected_columns.size());
      for (int i = 0; i < static_cast<int>(selected_columns.size()); ++i) {
        if (selected_columns[i] < 0 || selected_columns[i] >= Y.cols()) {
          throw std::runtime_error("selected column out of range");
        }
        Y_selected.col(i) = Y.col(selected_columns[i]);
      }
      tau_pred = Y_selected * beta;
    } else {
      if (beta.size() != Y.cols()) throw std::runtime_error("beta length does not match regressor columns");
      tau_pred = Y * beta;
    }
    const auto metrics = rebotarm::ident::regression_metrics(tau, tau_pred, model.nv());
    std::cout << "verify samples=" << ds.q.rows() << " rmse=" << metrics.rmse
              << " mae=" << metrics.mae << " max_abs=" << metrics.max_abs
              << " r2=" << metrics.r2 << "\n";
    std::cout << "per-joint rmse: " << metrics.per_joint_rmse.transpose() << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
