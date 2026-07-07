// Dynamics parameter identification helpers.
//
// The regressor follows Pinocchio's convention:
//   tau = Y(q, v, a) * pi
// where each 10-parameter block is
//   [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
// and I is the inertia at the center of mass, expressed in the body frame.
#pragma once

#include "rebotarm/robot_model.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace rebotarm {
namespace ident {

struct LeastSquaresResult {
  Eigen::VectorXd beta;
  Eigen::VectorXd tau_pred;
  int rank = 0;
  double condition = 0.0;
  double residual_norm = 0.0;
};

struct BaseParameterResult {
  Eigen::VectorXd beta_base;
  Eigen::VectorXi selected_columns;
  Eigen::VectorXd tau_pred;
  int rank = 0;
  double condition = 0.0;
  double residual_norm = 0.0;
};

struct RegressionMetrics {
  double rmse = 0.0;
  double mae = 0.0;
  double max_abs = 0.0;
  double r2 = 0.0;
  Eigen::VectorXd per_joint_rmse;
  Eigen::VectorXd per_joint_mae;
};

Eigen::MatrixXd joint_torque_regressor(const RobotModel& rm, const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& v, const Eigen::VectorXd& a);

Eigen::MatrixXd friction_regressor(const Eigen::VectorXd& v, double coulomb_eps = 1e-3);

Eigen::MatrixXd build_regression_matrix(const RobotModel& rm, const Eigen::MatrixXd& q_samples,
                                        const Eigen::MatrixXd& v_samples,
                                        const Eigen::MatrixXd& a_samples,
                                        bool include_friction = true,
                                        double coulomb_eps = 1e-3);

Eigen::VectorXd stack_tau_samples(const Eigen::MatrixXd& tau_samples);

LeastSquaresResult fit_least_squares(const Eigen::MatrixXd& Y, const Eigen::VectorXd& tau,
                                     double rcond = 1e-12);

BaseParameterResult fit_base_parameters_qr(const Eigen::MatrixXd& Y, const Eigen::VectorXd& tau,
                                           double rcond = 1e-12);

RegressionMetrics regression_metrics(const Eigen::VectorXd& tau,
                                     const Eigen::VectorXd& tau_pred, int dof);

double condition_number(const Eigen::MatrixXd& Y, double rcond = 1e-12);

Eigen::VectorXd model_dynamic_parameters(const RobotModel& rm);
int num_dynamic_parameters(const RobotModel& rm);
int num_total_parameters(const RobotModel& rm, bool include_friction);

std::vector<std::string> dynamic_parameter_block_names(const RobotModel& rm);
std::vector<std::string> dynamic_parameter_names(const RobotModel& rm);
std::vector<std::string> friction_parameter_names(int dof);
std::vector<std::string> total_parameter_names(const RobotModel& rm, bool include_friction);

}  // namespace ident
}  // namespace rebotarm
