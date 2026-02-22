// mpc_controller.hpp
#pragma once
#include <array>

struct MpcDiag {
  int status = -1;
  int iter = 0;
  double solve_time_ms = 0.0;
  double kkt = 0.0;
};

template<int NX, int NU, int NP>
class MpcController {
public:
  MpcController();
  ~MpcController();

  void reset();

  void set_params(const std::array<double, NP>& p) { p_ = p; has_p_ = true; }

  bool step(const std::array<double, NX>& x_meas,
            const std::array<double, NX>& x_ref,
            std::array<double, NU>& u_out,
            MpcDiag* diag = nullptr);

private:
  void* capsule_ = nullptr;
  std::array<double, NU> last_u_{};
  std::array<double, NP> p_{};
  bool has_p_ = false;
  bool ok_ = false;
};
