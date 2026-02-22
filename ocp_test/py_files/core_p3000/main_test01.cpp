#include "mpccontroller.hpp"
#include <array>
#include <iostream>

static constexpr int NX = 5;
static constexpr int NU = 2;
static constexpr int NP = 11;

int main()
{
    MpcController<NX, NU, NP> mpc;

    std::array<double, NP> p{};
    p.fill(1.0);          // non-zero to avoid divide-by-zero inside your dynamics
    p[7] = 9.81;          // g
    mpc.set_params(p);

    std::array<double, NX> x{};
    std::array<double, NX> x_ref{};
    std::array<double, NU> u{};
    MpcDiag diag{};

    x.fill(0.0);
    x_ref.fill(0.0);

    for (int k = 0; k < 50; k++)
    {
        // make the reference change slowly so we can see something move
        x_ref[0] = 0.1 * k;

        bool ok = mpc.step(x, x_ref, u, &diag);

        std::cout
            << "k=" << k
            << " ok=" << ok
            << " status=" << diag.status
            << " u=[" << u[0] << "," << u[1] << "]"
            << " time_ms=" << diag.solve_time_ms
            << " iter=" << diag.iter
            << " kkt=" << diag.kkt
            << "\n";

        // totally fake update just to change x so next call is different
        x[0] += 0.01 * (u[0] - u[1]);
        x[4] += 0.01 * (u[0] + u[1]);
    }

    return 0;
}
