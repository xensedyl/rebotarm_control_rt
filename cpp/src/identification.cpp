#include "rebotarm/identification.hpp"

#include <pinocchio/algorithm/regressor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rebotarm {
namespace ident {

namespace pin = pinocchio;

namespace {

void check_sample_matrix(const char* name, const Eigen::MatrixXd& x, int cols) {
  if (x.cols() != cols) {
    throw std::invalid_argument(std::string(name) + " column count mismatch");
  }
}

void check_same_rows(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                     const Eigen::MatrixXd& c) {
  if (a.rows() != b.rows() || a.rows() != c.rows()) {
    throw std::invalid_argument("q/v/a sample count mismatch");
  }
}

Eigen::VectorXd singular_values(const Eigen::MatrixXd& Y) {
  Eigen::BDCSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.singularValues();
}

}  // namespace

Eigen::MatrixXd joint_torque_regressor(const RobotModel& rm, const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& v, const Eigen::VectorXd& a) {
  if (q.size() != rm.nq()) throw std::invalid_argument("q length mismatch");
  if (v.size() != rm.nv()) throw std::invalid_argument("v length mismatch");
  if (a.size() != rm.nv()) throw std::invalid_argument("a length mismatch");
  pin::computeJointTorqueRegressor(rm.model(), rm.data(), q, v, a);
  return rm.data().jointTorqueRegressor;
}

Eigen::MatrixXd friction_regressor(const Eigen::VectorXd& v, double coulomb_eps) {
  if (coulomb_eps <= 0.0) throw std::invalid_argument("coulomb_eps must be positive");
  const int dof = static_cast<int>(v.size());
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(dof, dof * 2);
  for (int i = 0; i < dof; ++i) {
    F(i, 2 * i) = v[i];
    F(i, 2 * i + 1) = std::tanh(v[i] / coulomb_eps);
  }
  return F;
}

Eigen::MatrixXd build_regression_matrix(const RobotModel& rm, const Eigen::MatrixXd& q_samples,
                                        const Eigen::MatrixXd& v_samples,
                                        const Eigen::MatrixXd& a_samples,
                                        bool include_friction,
                                        double coulomb_eps) {
  check_sample_matrix("q_samples", q_samples, rm.nq());
  check_sample_matrix("v_samples", v_samples, rm.nv());
  check_sample_matrix("a_samples", a_samples, rm.nv());
  check_same_rows(q_samples, v_samples, a_samples);

  const int samples = static_cast<int>(q_samples.rows());
  const int dof = rm.nv();
  const int dyn_cols = num_dynamic_parameters(rm);
  const int total_cols = dyn_cols + (include_friction ? 2 * dof : 0);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(samples * dof, total_cols);
  for (int i = 0; i < samples; ++i) {
    const Eigen::VectorXd q = q_samples.row(i).transpose();
    const Eigen::VectorXd v = v_samples.row(i).transpose();
    const Eigen::VectorXd a = a_samples.row(i).transpose();
    Y.block(i * dof, 0, dof, dyn_cols) = joint_torque_regressor(rm, q, v, a);
    if (include_friction) {
      Y.block(i * dof, dyn_cols, dof, 2 * dof) = friction_regressor(v, coulomb_eps);
    }
  }
  return Y;
}

Eigen::VectorXd stack_tau_samples(const Eigen::MatrixXd& tau_samples) {
  Eigen::VectorXd tau(tau_samples.rows() * tau_samples.cols());
  int idx = 0;
  for (int r = 0; r < tau_samples.rows(); ++r) {
    for (int c = 0; c < tau_samples.cols(); ++c) {
      tau[idx++] = tau_samples(r, c);
    }
  }
  return tau;
}

LeastSquaresResult fit_least_squares(const Eigen::MatrixXd& Y, const Eigen::VectorXd& tau,
                                     double rcond) {
  if (Y.rows() != tau.size()) throw std::invalid_argument("Y rows and tau length mismatch");
  if (rcond <= 0.0) throw std::invalid_argument("rcond must be positive");

  Eigen::BDCSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd s = svd.singularValues();
  const double max_s = s.size() > 0 ? s[0] : 0.0;
  const double tol = std::max(Y.rows(), Y.cols()) * rcond * max_s;
  int rank = 0;
  Eigen::VectorXd inv_s = Eigen::VectorXd::Zero(s.size());
  for (int i = 0; i < s.size(); ++i) {
    if (s[i] > tol) {
      inv_s[i] = 1.0 / s[i];
      ++rank;
    }
  }
  Eigen::VectorXd beta = svd.matrixV() * inv_s.asDiagonal() * svd.matrixU().transpose() * tau;
  Eigen::VectorXd tau_pred = Y * beta;
  LeastSquaresResult out;
  out.beta = beta;
  out.tau_pred = tau_pred;
  out.rank = rank;
  out.condition = condition_number(Y, rcond);
  out.residual_norm = (tau - tau_pred).norm();
  return out;
}

BaseParameterResult fit_base_parameters_qr(const Eigen::MatrixXd& Y, const Eigen::VectorXd& tau,
                                           double rcond) {
  if (Y.rows() != tau.size()) throw std::invalid_argument("Y rows and tau length mismatch");
  if (rcond <= 0.0) throw std::invalid_argument("rcond must be positive");

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Y);
  qr.setThreshold(rcond);
  const int rank = qr.rank();
  Eigen::VectorXi selected(rank);
  const auto indices = qr.colsPermutation().indices();
  for (int i = 0; i < rank; ++i) selected[i] = indices[i];

  Eigen::MatrixXd Y_base(Y.rows(), rank);
  for (int i = 0; i < rank; ++i) {
    Y_base.col(i) = Y.col(selected[i]);
  }
  LeastSquaresResult ls = fit_least_squares(Y_base, tau, rcond);

  BaseParameterResult out;
  out.beta_base = ls.beta;
  out.selected_columns = selected;
  out.tau_pred = ls.tau_pred;
  out.rank = rank;
  out.condition = ls.condition;
  out.residual_norm = ls.residual_norm;
  return out;
}

RegressionMetrics regression_metrics(const Eigen::VectorXd& tau,
                                     const Eigen::VectorXd& tau_pred, int dof) {
  if (tau.size() != tau_pred.size()) throw std::invalid_argument("tau length mismatch");
  if (dof <= 0 || tau.size() % dof != 0) throw std::invalid_argument("invalid dof");

  const Eigen::VectorXd err = tau_pred - tau;
  RegressionMetrics out;
  out.rmse = std::sqrt(err.array().square().mean());
  out.mae = err.array().abs().mean();
  out.max_abs = err.array().abs().maxCoeff();
  const double sse = err.squaredNorm();
  const double mean = tau.mean();
  const double sst = (tau.array() - mean).square().sum();
  out.r2 = sst > std::numeric_limits<double>::epsilon() ? 1.0 - sse / sst : 1.0;
  out.per_joint_rmse = Eigen::VectorXd::Zero(dof);
  out.per_joint_mae = Eigen::VectorXd::Zero(dof);
  const int samples = static_cast<int>(tau.size()) / dof;
  for (int j = 0; j < dof; ++j) {
    double sq = 0.0;
    double abs_sum = 0.0;
    for (int i = 0; i < samples; ++i) {
      const double e = err[i * dof + j];
      sq += e * e;
      abs_sum += std::abs(e);
    }
    out.per_joint_rmse[j] = std::sqrt(sq / samples);
    out.per_joint_mae[j] = abs_sum / samples;
  }
  return out;
}

double condition_number(const Eigen::MatrixXd& Y, double rcond) {
  if (Y.size() == 0) return 0.0;
  const Eigen::VectorXd s = singular_values(Y);
  if (s.size() == 0) return 0.0;
  const double max_s = s[0];
  const double tol = std::max(Y.rows(), Y.cols()) * rcond * max_s;
  double min_s = 0.0;
  for (int i = s.size() - 1; i >= 0; --i) {
    if (s[i] > tol) {
      min_s = s[i];
      break;
    }
  }
  if (min_s <= 0.0) return std::numeric_limits<double>::infinity();
  return max_s / min_s;
}

Eigen::VectorXd model_dynamic_parameters(const RobotModel& rm) {
  Eigen::VectorXd params(num_dynamic_parameters(rm));
  int offset = 0;
  for (std::size_t i = 1; i < rm.model().inertias.size(); ++i) {
    params.segment<10>(offset) = rm.model().inertias[i].toDynamicParameters();
    offset += 10;
  }
  return params;
}

int num_dynamic_parameters(const RobotModel& rm) {
  return 10 * (static_cast<int>(rm.model().njoints) - 1);
}

int num_total_parameters(const RobotModel& rm, bool include_friction) {
  return num_dynamic_parameters(rm) + (include_friction ? 2 * rm.nv() : 0);
}

std::vector<std::string> dynamic_parameter_block_names(const RobotModel& rm) {
  std::vector<std::string> names;
  names.reserve(rm.model().njoints > 0 ? rm.model().njoints - 1 : 0);
  for (std::size_t i = 1; i < rm.model().names.size(); ++i) {
    names.push_back(rm.model().names[i]);
  }
  return names;
}

std::vector<std::string> dynamic_parameter_names(const RobotModel& rm) {
  const std::vector<std::string> fields = {
      "m", "mcx", "mcy", "mcz", "ixx", "ixy", "iyy", "ixz", "iyz", "izz"};
  std::vector<std::string> names;
  for (const auto& block : dynamic_parameter_block_names(rm)) {
    for (const auto& field : fields) {
      names.push_back(block + "." + field);
    }
  }
  return names;
}

std::vector<std::string> friction_parameter_names(int dof) {
  std::vector<std::string> names;
  for (int i = 0; i < dof; ++i) {
    names.push_back("joint_" + std::to_string(i + 1) + ".viscous");
    names.push_back("joint_" + std::to_string(i + 1) + ".coulomb");
  }
  return names;
}

std::vector<std::string> total_parameter_names(const RobotModel& rm, bool include_friction) {
  std::vector<std::string> names = dynamic_parameter_names(rm);
  if (include_friction) {
    std::vector<std::string> friction = friction_parameter_names(rm.nv());
    names.insert(names.end(), friction.begin(), friction.end());
  }
  return names;
}

}  // namespace ident
}  // namespace rebotarm
