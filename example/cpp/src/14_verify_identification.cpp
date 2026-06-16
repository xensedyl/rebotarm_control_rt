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

}  // namespace

int main(int argc, char** argv) {
  try {
    if (example::has_flag(argc, argv, "--help") || example::has_flag(argc, argv, "-h")) {
      std::cout << "Usage: ./14_verify_identification --data id_verify.csv --beta beta.txt "
                   "[--urdf robot.urdf] [--no-friction]\n";
      return 0;
    }
    const std::string data_path = example::arg_value(argc, argv, "--data");
    const std::string beta_path = example::arg_value(argc, argv, "--beta");
    if (data_path.empty() || beta_path.empty()) throw std::runtime_error("--data and --beta are required");
    const bool include_friction = !example::has_flag(argc, argv, "--no-friction");
    const double coulomb_eps = example::arg_double(argc, argv, "--coulomb-eps", 1e-3);

    rebotarm::RobotModel model(example::urdf_arg(argc, argv));
    Dataset ds = load_csv(data_path, model.nv());
    const Eigen::MatrixXd Y = rebotarm::ident::build_regression_matrix(
        model, ds.q, ds.dq, ds.ddq, include_friction, coulomb_eps);
    const auto beta_vec = load_beta_txt(beta_path);
    Eigen::VectorXd beta(beta_vec.size());
    for (int i = 0; i < beta.size(); ++i) beta[i] = beta_vec[i];
    if (beta.size() != Y.cols()) throw std::runtime_error("beta length does not match regressor columns");

    const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(ds.tau);
    const Eigen::VectorXd tau_pred = Y * beta;
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
