#include "mpccontroller.hpp"

#include "c_generated_code_p3000/acados_solver_p3000_mpc_model.h"

static constexpr int N   = 50;
static constexpr int NY  = 7;
static constexpr int NYE = 5;

static inline void zero(double* a, int n) { for (int i = 0; i < n; i++) a[i] = 0.0; }

template<int NX_, int NU_, int NP_>
MpcController<NX_, NU_, NP_>::MpcController()
{
  capsule_ = (void*) p3000_mpc_model_acados_create_capsule();
  if (!capsule_) return;
  int status = p3000_mpc_model_acados_create((p3000_mpc_model_solver_capsule*)capsule_);
  ok_ = (status == 0);
  last_u_.fill(0.0);
  p_.fill(0.0);
}

template<int NX_, int NU_, int NP_>
MpcController<NX_, NU_, NP_>::~MpcController()
{
  if (!capsule_) return;
  p3000_mpc_model_acados_free((p3000_mpc_model_solver_capsule*)capsule_);
  p3000_mpc_model_acados_free_capsule((p3000_mpc_model_solver_capsule*)capsule_);
  capsule_ = nullptr;
  ok_ = false;
}

template<int NX_, int NU_, int NP_>
void MpcController<NX_, NU_, NP_>::reset()
{
  last_u_.fill(0.0);
}

template<int NX_, int NU_, int NP_>
bool MpcController<NX_, NU_, NP_>::step(const std::array<double, NX_>& x_meas,
                                       const std::array<double, NX_>& x_ref,
                                       std::array<double, NU_>& u_out,
                                       MpcDiag* diag)
{
  u_out = last_u_;
  if (!ok_) { if (diag) diag->status = -999; return false; }

  auto* cap = (p3000_mpc_model_solver_capsule*)capsule_;
  ocp_nlp_in*     in     = p3000_mpc_model_acados_get_nlp_in(cap);
  ocp_nlp_out*    out    = p3000_mpc_model_acados_get_nlp_out(cap);
  ocp_nlp_solver* solver = p3000_mpc_model_acados_get_nlp_solver(cap);
  ocp_nlp_config* cfg    = p3000_mpc_model_acados_get_nlp_config(cap);
  ocp_nlp_dims*   dims   = p3000_mpc_model_acados_get_nlp_dims(cap);

  if (has_p_) {
    double p_stage[NP_];
    for (int i = 0; i < NP_; i++) p_stage[i] = p_[i];
    for (int k = 0; k <= N; k++) {
      p3000_mpc_model_acados_update_params(cap, k, p_stage, NP_);
    }
  }

  double yref[NY];
  zero(yref, NY);
  for (int i = 0; i < NX_; i++) yref[i] = x_ref[i];
  for (int k = 0; k < N; k++) ocp_nlp_cost_model_set(cfg, dims, in, k, "yref", yref);

  double yref_e[NYE];
  zero(yref_e, NYE);
  for (int i = 0; i < NX_; i++) yref_e[i] = x_ref[i];
  ocp_nlp_cost_model_set(cfg, dims, in, N, "yref", yref_e);

  double x0[NX_];
  for (int i = 0; i < NX_; i++) x0[i] = x_meas[i];
  ocp_nlp_out_set(cfg, dims, out, in, 0, "x", x0);

  double u_init[NU_];
  for (int i = 0; i < NU_; i++) u_init[i] = last_u_[i];
  for (int k = 0; k < N; k++) ocp_nlp_out_set(cfg, dims, out, in, k, "u", u_init);

  int status = p3000_mpc_model_acados_solve(cap);

  if (status == 0) {
    double u0[NU_];
    ocp_nlp_out_get(cfg, dims, out, 0, "u", u0);
    for (int i = 0; i < NU_; i++) u_out[i] = u0[i];
    last_u_ = u_out;
  }

  if (diag) {
    diag->status = status;

    double time_tot = 0.0, //kkt = 0.0;
    int sqp_iter = 0;

    ocp_nlp_get(solver, "time_tot", &time_tot);
    ocp_nlp_get(solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(solver, "kkt_norm_inf", &kkt);

    diag->solve_time_ms = time_tot * 1000.0;
    diag->iter = sqp_iter;
    diag->kkt = 0.0;
  }

  return (status == 0);
}

template class MpcController<5, 2, 11>;
