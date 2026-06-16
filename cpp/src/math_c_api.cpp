#include "rebotarm/math_c_api.h"

#include "rebotarm/dynamics.hpp"
#include "rebotarm/identification.hpp"
#include "rebotarm/robot_model.hpp"

#include <Eigen/Dense>
#include <pinocchio/math/rpy.hpp>

#include <exception>
#include <memory>
#include <string>

struct RebotarmMathModel {
  std::unique_ptr<rebotarm::RobotModel> model;
};

static thread_local std::string g_last_error;

static int fail(const std::string& message) {
  g_last_error = message;
  return -1;
}

static bool valid_model(const RebotarmMathModel* model) {
  return model != nullptr && model->model != nullptr;
}

static Eigen::VectorXd vector_from_ptr(const double* data, int len) {
  Eigen::VectorXd out(len);
  for (int i = 0; i < len; ++i) out[i] = data[i];
  return out;
}

static Eigen::Matrix4d matrix_from_row_major(const double* data) {
  Eigen::Matrix4d out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) out(r, c) = data[r * 4 + c];
  return out;
}

static Eigen::MatrixXd matrix_from_row_major_dyn(const double* data, int rows, int cols) {
  Eigen::MatrixXd out(rows, cols);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) out(r, c) = data[r * cols + c];
  return out;
}

static void matrix_to_row_major(const Eigen::Matrix4d& in, double* out) {
  if (!out) return;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) out[r * 4 + c] = in(r, c);
}

static void matrix_to_row_major_dyn(const Eigen::MatrixXd& in, double* out, int rows, int cols) {
  if (!out) return;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) out[r * cols + c] = in(r, c);
}

static void vector_to_ptr(const Eigen::VectorXd& in, double* out, int out_len) {
  for (int i = 0; i < in.size() && i < out_len; ++i) out[i] = in[i];
}

extern "C" {

RebotarmMathModel* rebotarm_math_model_new(const char* urdf_path) {
  try {
    if (!urdf_path) {
      fail("urdf_path is null");
      return nullptr;
    }
    auto* handle = new RebotarmMathModel;
    handle->model = std::make_unique<rebotarm::RobotModel>(urdf_path);
    g_last_error.clear();
    return handle;
  } catch (const std::exception& e) {
    fail(e.what());
  } catch (...) {
    fail("unknown exception");
  }
  return nullptr;
}

void rebotarm_math_model_free(RebotarmMathModel* model) { delete model; }

int rebotarm_math_model_nq(const RebotarmMathModel* model) {
  if (!valid_model(model)) return -1;
  return model->model->nq();
}

int rebotarm_math_model_nv(const RebotarmMathModel* model) {
  if (!valid_model(model)) return -1;
  return model->model->nv();
}

int rebotarm_math_end_frame_id(const RebotarmMathModel* model) {
  try {
    if (!valid_model(model)) return fail("model is null");
    return model->model->end_effector_frame_id();
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_frame_id(const RebotarmMathModel* model, const char* frame_name) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!frame_name) return fail("frame_name is null");
    return model->model->frame_id(frame_name);
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_neutral(const RebotarmMathModel* model, double* out_q, int out_len) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!out_q) return fail("out_q is null");
    const Eigen::VectorXd q = model->model->neutral();
    if (out_len < q.size()) return fail("out_q too small");
    for (int i = 0; i < q.size(); ++i) out_q[i] = q[i];
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_fk(const RebotarmMathModel* model,
                     const double* q,
                     int q_len,
                     const char* frame_name,
                     double* out_xyz,
                     double* out_rpy,
                     double* out_T_row_major_4x4) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!q) return fail("q is null");
    if (q_len != model->model->nq()) return fail("q length mismatch");
    const std::string frame = frame_name ? frame_name : "";
    const auto [pos, rot, T] = model->model->fk(vector_from_ptr(q, q_len), frame);
    if (out_xyz) {
      out_xyz[0] = pos.x();
      out_xyz[1] = pos.y();
      out_xyz[2] = pos.z();
    }
    if (out_rpy) {
      const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(rot);
      out_rpy[0] = rpy.x();
      out_rpy[1] = rpy.y();
      out_rpy[2] = rpy.z();
    }
    matrix_to_row_major(T, out_T_row_major_4x4);
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_ik(const RebotarmMathModel* model,
                     const double* target_T_row_major_4x4,
                     const double* q_seed,
                     int q_len,
                     int frame_id,
                     int max_iter,
                     double tolerance,
                     double step_size,
                     double damping,
                     double* out_q,
                     RebotarmIkResult* out_result) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!target_T_row_major_4x4) return fail("target matrix is null");
    if (!q_seed) return fail("q_seed is null");
    if (!out_q) return fail("out_q is null");
    if (q_len != model->model->nq()) return fail("q length mismatch");
    rebotarm::IKParams params;
    params.max_iter = max_iter;
    params.tolerance = tolerance;
    params.step_size = step_size;
    params.damping = damping;
    const auto result = model->model->solve_ik_with_retry(
        matrix_from_row_major(target_T_row_major_4x4),
        vector_from_ptr(q_seed, q_len),
        frame_id,
        params,
        8);
    if (result.q.size() != q_len) return fail("IK result length mismatch");
    for (int i = 0; i < q_len; ++i) out_q[i] = result.q[i];
    if (out_result) {
      out_result->success = result.success ? 1 : 0;
      out_result->error = result.error;
      out_result->iterations = result.iterations;
    }
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_generalized_gravity(const RebotarmMathModel* model,
                                      const double* q,
                                      int q_len,
                                      double* out_tau,
                                      int out_len) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!q) return fail("q is null");
    if (!out_tau) return fail("out_tau is null");
    if (q_len != model->model->nq()) return fail("q length mismatch");
    const Eigen::VectorXd tau = rebotarm::dyn::generalized_gravity(*model->model, vector_from_ptr(q, q_len));
    if (out_len < tau.size()) return fail("out_tau too small");
    for (int i = 0; i < tau.size(); ++i) out_tau[i] = tau[i];
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_num_dynamic_parameters(const RebotarmMathModel* model) {
  try {
    if (!valid_model(model)) return fail("model is null");
    return rebotarm::ident::num_dynamic_parameters(*model->model);
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_num_total_parameters(const RebotarmMathModel* model, int include_friction) {
  try {
    if (!valid_model(model)) return fail("model is null");
    return rebotarm::ident::num_total_parameters(*model->model, include_friction != 0);
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_build_regression_matrix(const RebotarmMathModel* model,
                                          const double* q_samples_row_major,
                                          const double* v_samples_row_major,
                                          const double* a_samples_row_major,
                                          int samples,
                                          int dof,
                                          int include_friction,
                                          double coulomb_eps,
                                          double* out_Y_row_major,
                                          int out_rows,
                                          int out_cols) {
  try {
    if (!valid_model(model)) return fail("model is null");
    if (!q_samples_row_major || !v_samples_row_major || !a_samples_row_major) {
      return fail("sample matrix is null");
    }
    if (!out_Y_row_major) return fail("out_Y is null");
    if (samples <= 0) return fail("samples must be positive");
    if (dof != model->model->nq() || dof != model->model->nv()) return fail("dof mismatch");
    const Eigen::MatrixXd q = matrix_from_row_major_dyn(q_samples_row_major, samples, dof);
    const Eigen::MatrixXd v = matrix_from_row_major_dyn(v_samples_row_major, samples, dof);
    const Eigen::MatrixXd a = matrix_from_row_major_dyn(a_samples_row_major, samples, dof);
    const Eigen::MatrixXd Y = rebotarm::ident::build_regression_matrix(
        *model->model, q, v, a, include_friction != 0, coulomb_eps);
    if (out_rows < Y.rows() || out_cols < Y.cols()) return fail("out_Y too small");
    matrix_to_row_major_dyn(Y, out_Y_row_major, Y.rows(), Y.cols());
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_stack_tau_samples(const double* tau_samples_row_major,
                                    int samples,
                                    int dof,
                                    double* out_tau,
                                    int out_len) {
  try {
    if (!tau_samples_row_major) return fail("tau_samples is null");
    if (!out_tau) return fail("out_tau is null");
    const Eigen::MatrixXd tau_samples = matrix_from_row_major_dyn(tau_samples_row_major, samples, dof);
    const Eigen::VectorXd tau = rebotarm::ident::stack_tau_samples(tau_samples);
    if (out_len < tau.size()) return fail("out_tau too small");
    vector_to_ptr(tau, out_tau, out_len);
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_fit_least_squares(const double* Y_row_major,
                                    int rows,
                                    int cols,
                                    const double* tau,
                                    int tau_len,
                                    double rcond,
                                    double* out_beta,
                                    int out_beta_len,
                                    double* out_tau_pred,
                                    int out_tau_pred_len,
                                    RebotarmLsInfo* out_info) {
  try {
    if (!Y_row_major || !tau || !out_beta || !out_tau_pred) return fail("null argument");
    const Eigen::MatrixXd Y = matrix_from_row_major_dyn(Y_row_major, rows, cols);
    const Eigen::VectorXd tau_vec = vector_from_ptr(tau, tau_len);
    const auto result = rebotarm::ident::fit_least_squares(Y, tau_vec, rcond);
    if (out_beta_len < result.beta.size()) return fail("out_beta too small");
    if (out_tau_pred_len < result.tau_pred.size()) return fail("out_tau_pred too small");
    vector_to_ptr(result.beta, out_beta, out_beta_len);
    vector_to_ptr(result.tau_pred, out_tau_pred, out_tau_pred_len);
    if (out_info) {
      out_info->rank = result.rank;
      out_info->condition = result.condition;
      out_info->residual_norm = result.residual_norm;
    }
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_fit_base_parameters_qr(const double* Y_row_major,
                                         int rows,
                                         int cols,
                                         const double* tau,
                                         int tau_len,
                                         double rcond,
                                         double* out_beta,
                                         int out_beta_len,
                                         int* out_selected_columns,
                                         int out_selected_len,
                                         double* out_tau_pred,
                                         int out_tau_pred_len,
                                         RebotarmLsInfo* out_info) {
  try {
    if (!Y_row_major || !tau || !out_beta || !out_selected_columns || !out_tau_pred) {
      return fail("null argument");
    }
    const Eigen::MatrixXd Y = matrix_from_row_major_dyn(Y_row_major, rows, cols);
    const Eigen::VectorXd tau_vec = vector_from_ptr(tau, tau_len);
    const auto result = rebotarm::ident::fit_base_parameters_qr(Y, tau_vec, rcond);
    if (out_beta_len < result.beta_base.size()) return fail("out_beta too small");
    if (out_selected_len < result.selected_columns.size()) return fail("out_selected too small");
    if (out_tau_pred_len < result.tau_pred.size()) return fail("out_tau_pred too small");
    vector_to_ptr(result.beta_base, out_beta, out_beta_len);
    for (int i = 0; i < result.selected_columns.size(); ++i) {
      out_selected_columns[i] = result.selected_columns[i];
    }
    vector_to_ptr(result.tau_pred, out_tau_pred, out_tau_pred_len);
    if (out_info) {
      out_info->rank = result.rank;
      out_info->condition = result.condition;
      out_info->residual_norm = result.residual_norm;
    }
    return result.rank;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

int rebotarm_math_regression_metrics(const double* tau,
                                     const double* tau_pred,
                                     int len,
                                     int dof,
                                     RebotarmMetrics* out_metrics,
                                     double* out_per_joint_rmse,
                                     double* out_per_joint_mae,
                                     int out_joint_len) {
  try {
    if (!tau || !tau_pred || !out_metrics || !out_per_joint_rmse || !out_per_joint_mae) {
      return fail("null argument");
    }
    const Eigen::VectorXd tau_vec = vector_from_ptr(tau, len);
    const Eigen::VectorXd pred_vec = vector_from_ptr(tau_pred, len);
    const auto metrics = rebotarm::ident::regression_metrics(tau_vec, pred_vec, dof);
    if (out_joint_len < metrics.per_joint_rmse.size()) return fail("joint metric output too small");
    out_metrics->rmse = metrics.rmse;
    out_metrics->mae = metrics.mae;
    out_metrics->max_abs = metrics.max_abs;
    out_metrics->r2 = metrics.r2;
    vector_to_ptr(metrics.per_joint_rmse, out_per_joint_rmse, out_joint_len);
    vector_to_ptr(metrics.per_joint_mae, out_per_joint_mae, out_joint_len);
    return 0;
  } catch (const std::exception& e) {
    return fail(e.what());
  }
}

const char* rebotarm_math_last_error(void) { return g_last_error.c_str(); }

}
