#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RebotarmMathModel RebotarmMathModel;

typedef struct RebotarmIkResult {
  int success;
  double error;
  int iterations;
} RebotarmIkResult;

typedef struct RebotarmLsInfo {
  int rank;
  double condition;
  double residual_norm;
} RebotarmLsInfo;

typedef struct RebotarmMetrics {
  double rmse;
  double mae;
  double max_abs;
  double r2;
} RebotarmMetrics;

RebotarmMathModel* rebotarm_math_model_new(const char* urdf_path);
void rebotarm_math_model_free(RebotarmMathModel* model);

int rebotarm_math_model_nq(const RebotarmMathModel* model);
int rebotarm_math_model_nv(const RebotarmMathModel* model);
int rebotarm_math_end_frame_id(const RebotarmMathModel* model);
int rebotarm_math_frame_id(const RebotarmMathModel* model, const char* frame_name);

int rebotarm_math_neutral(const RebotarmMathModel* model, double* out_q, int out_len);

int rebotarm_math_fk(
    const RebotarmMathModel* model,
    const double* q,
    int q_len,
    const char* frame_name,
    double* out_xyz,
    double* out_rpy,
    double* out_T_row_major_4x4);

int rebotarm_math_ik(
    const RebotarmMathModel* model,
    const double* target_T_row_major_4x4,
    const double* q_seed,
    int q_len,
    int frame_id,
    int max_iter,
    double tolerance,
    double step_size,
    double damping,
    double* out_q,
    RebotarmIkResult* out_result);

int rebotarm_math_generalized_gravity(
    const RebotarmMathModel* model,
    const double* q,
    int q_len,
    double* out_tau,
    int out_len);

int rebotarm_math_inverse_dynamics(
    const RebotarmMathModel* model,
    const double* q,
    int q_len,
    const double* v,
    int v_len,
    const double* a,
    int a_len,
    double* out_tau,
    int out_len);

int rebotarm_math_num_dynamic_parameters(const RebotarmMathModel* model);
int rebotarm_math_num_total_parameters(const RebotarmMathModel* model, int include_friction);

int rebotarm_math_model_dynamic_parameters(
    const RebotarmMathModel* model,
    double* out_params,
    int out_len);

int rebotarm_math_build_regression_matrix(
    const RebotarmMathModel* model,
    const double* q_samples_row_major,
    const double* v_samples_row_major,
    const double* a_samples_row_major,
    int samples,
    int dof,
    int include_friction,
    double coulomb_eps,
    double* out_Y_row_major,
    int out_rows,
    int out_cols);

int rebotarm_math_stack_tau_samples(
    const double* tau_samples_row_major,
    int samples,
    int dof,
    double* out_tau,
    int out_len);

int rebotarm_math_fit_least_squares(
    const double* Y_row_major,
    int rows,
    int cols,
    const double* tau,
    int tau_len,
    double rcond,
    double* out_beta,
    int out_beta_len,
    double* out_tau_pred,
    int out_tau_pred_len,
    RebotarmLsInfo* out_info);

int rebotarm_math_fit_base_parameters_qr(
    const double* Y_row_major,
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
    RebotarmLsInfo* out_info);

int rebotarm_math_regression_metrics(
    const double* tau,
    const double* tau_pred,
    int len,
    int dof,
    RebotarmMetrics* out_metrics,
    double* out_per_joint_rmse,
    double* out_per_joint_mae,
    int out_joint_len);

const char* rebotarm_math_last_error(void);

#ifdef __cplusplus
}
#endif
