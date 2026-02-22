# P3000 MPC – Plumbing Integration Summary

## Objective

The goal of this phase was to:

- Take acados-generated C solver code  
- Wrap it inside a clean C++ `MpcController` class  
- Build it standalone (not inside ROS)  
- Verify solver plumbing works before integration  
- Ensure compilation, linking, and runtime execution are correct  

This phase focused strictly on **plumbing correctness**, not model accuracy.

---

## acados Build & Installation

### Source Location

~/main/Personal/Peer_works/ocp_test/acados


### Build Commands

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make -j$(nproc)
make install


### Install Output

acados/install/include
acados/install/lib


### Important Libraries

libacados.so
libhpipm.so
libblasfeo.so


---

## Standalone MPC Test Project

### Project Structure

core_p3000/
main_test01.cpp
mpccontroller.hpp
mpccontroller.cpp
c_generated_code_p3000/
CMakeLists.txt

---

## CMake Configuration

### Included Directories

- Project source directory  
- acados install include  
- hpipm include  
- blasfeo include  
- generated solver code directories  

### Linked Libraries
acados
hpipm
blasfeo
m
dl
pthread


### Excluded Generated Files

main_.c
acados_sim_solver_.c


These are demo executables and not required for standalone integration.

---

## MpcController Design

### Template Definition

```cpp
template<int NX, int NU, int NP>
class MpcController

Explicit Instantiation

template class MpcController<5, 2, 11>;

Fixes Applied
1. Template Mismatch Fix

Issue:
Used NX instead of NX_ inside template implementation.

Resolution:
Use template parameters consistently:


std::array<double, NX_>
double x0[NX_]

2. acados API Update (dims Required)

New acados API requires passing dims into setters.

Added:

ocp_nlp_dims* dims = p3000_mpc_model_acados_get_nlp_dims(cap);

ocp_nlp_cost_model_set(cfg, dims, in, ...)
ocp_nlp_out_set(cfg, dims, out, in, ...)
ocp_nlp_out_get(cfg, dims, out, ...)

3. Parameter Update Fix

Incorrect Method:

ocp_nlp_dynamics_model_set(..., "p", ...)

Error Produced:

sim_erk_model_set: wrong field: p


Correct Method:

p3000_mpc_model_acados_update_params(cap, stage, p_stage, NP_);

4. Diagnostics Adjustment

Removed unsupported field:

"kkt_norm_inf"


Used supported fields instead:

ocp_nlp_get(solver, "time_tot", ...)
ocp_nlp_get(solver, "sqp_iter", ...)

Runtime Test
Build
cmake --build . -j$(nproc)

Run
LD_LIBRARY_PATH=/path/to/acados/install/lib:$LD_LIBRARY_PATH ./test_mpc

Observations

Solver executes

SQP_RTI runs

QP error (ACADOS_NAN_DETECTED) occurs with unrealistic parameters

No crashes

step() callable repeatedly

Conclusion

Plumbing layer is structurally correct.

Current System State

✔ acados installed correctly

✔ Solver builds successfully

✔ CMake integration works

✔ MPC wrapper compiles

✔ Solver executes

✔ Parameters update correctly

✔ No include/link/runtime loader errors

Remaining issues are numerical/model related, not structural.

Verified Data Flow
main_test01.cpp
        ↓
MpcController::step()
        ↓
acados capsule
        ↓
generated solver
        ↓
SQP_RTI solve
        ↓
u0 extracted
        ↓
returned to caller


This confirms full data flow is operational.

Next Steps

Improve parameter realism

Validate cost reference handling

Add constraints if required

Integrate controller into ROS node as callback

Optionally remove template if dimensions remain fixed

Final Conclusion

The MPC controller is:

Properly wrapped in C++

Successfully compiled and linked

Correctly calling the generated acados solver

Structurally ready for ROS integration

The plumbing layer is complete and functional.
