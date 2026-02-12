# pr_3000_core

Core NMPC / OCP implementation for the Peer Robotics P3000 robot + trailer platform.

This repository contains:

- Python-based NMPC (CasADi + ACADOS)
- Automatic C code generation via ACADOS
- Standalone C solver build capability
- No MATLAB / Simulink dependency required

------------------------------------------------------------
PROJECT STRUCTURE OVERVIEW
------------------------------------------------------------

Repository layout (relevant parts only):

/ocp_test/py_files/core_p3000/cloop_test_1.py

------------------------------------------------------------
KEY UNDERSTANDING
------------------------------------------------------------

- Python is used only to define the model and generate the ACADOS solver.
- The folder c_generated_code_p3000/ is a fully standalone C project.
- The file libacados_ocp_solver_p3000_mpc_model.so is the compiled MPC solver.
- run_mpc_test is a local executable used to verify the C solver.
- MATLAB files are only for model verification and are NOT required.

Workflow:

Python (CasADi model + OCP) 
      ↓
ACADOS code generation
      ↓
C source files exported
      ↓
Shared libraries compiled (.so)
      ↓
Standalone executable linked (run_mpc_test)



------------------------------------------------------------
CLOSED-LOOP NMPC / OCP
------------------------------------------------------------

Primary development currently happens in Python.

Main entry file:

ocp_test/py_files/core_p3000/cloop_test_1.py

This script:
- Defines the P3000 robot + trailer dynamics
- Builds an ACADOS OCP solver
- Runs closed-loop simulation
- Automatically generates C solver code


------------------------------------------------------------
REQUIREMENTS
------------------------------------------------------------

- Python 3.8+
- CMake
- GCC or Clang
- Git (with submodule support)


------------------------------------------------------------
CLONE THE REPOSITORY (IMPORTANT)
------------------------------------------------------------

This repository uses ACADOS as a git submodule.

Clone with:

git clone --recurse-submodules <REPO_URL>

If already cloned without submodules:

git submodule update --init --recursive


------------------------------------------------------------
PYTHON ENVIRONMENT SETUP - 
------------------------------------------------------------

cd ocp_test/py_files/core_p3000
python3 -m venv .venv
source .venv/bin/activate
pip install numpy scipy matplotlib casadi


------------------------------------------------------------
BUILD ACADOS - it is a one time
------------------------------------------------------------

cd ocp_test/acados
mkdir build
cd build
cmake ..
make -j4

Add the following to your shell configuration :

export ACADOS_INSTALL_DIR=<path_to_ocp_test/acados>
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$ACADOS_INSTALL_DIR/interfaces/acados_template:$PYTHONPATH

Reload:

source ~/.bashrc


------------------------------------------------------------
RUN PYTHON CLOSED-LOOP NMPC
------------------------------------------------------------

cd ocp_test/py_files/core_p3000
python cloop_test_1.py

This will:
- Build the ACADOS solver
- Run closed-loop simulation
- Export C code into:

c_generated_code_p3000/


------------------------------------------------------------
C CODE GENERATION OVERVIEW
------------------------------------------------------------

When the solver is created in Python:

solver = AcadosOcpSolver(...)

ACADOS automatically:

1. Generates C source files for the model
2. Generates derivative and sensitivity code
3. Generates solver wrapper code
4. Compiles shared libraries
5. Exports everything into:

c_generated_code_p3000/

Important generated artifacts include:

- acados_solver_p3000_mpc_model.c/h  → MPC solver interface
- p3000_mpc_model_expl_ode_fun.c     → system dynamics
- libacados_ocp_solver_p3000_mpc_model.so → compiled solver library


------------------------------------------------------------
BUILD THE C SOLVER (STANDALONE)
------------------------------------------------------------

cd c_generated_code_p3000
make

This produces:

libacados_ocp_solver_p3000_mpc_model.so


------------------------------------------------------------
RUNNING THE GENERATED C SOLVER
------------------------------------------------------------

To test the solver in pure C:

gcc -O2 main_p3000_mpc_model.c \
  -I. \
  -I<ACADOS_INSTALL_DIR>/include \
  -I<ACADOS_INSTALL_DIR>/include/acados \
  -I<ACADOS_INSTALL_DIR>/include/blasfeo/include \
  -I<ACADOS_INSTALL_DIR>/include/hpipm/include \
  -L. -lacados_ocp_solver_p3000_mpc_model \
  -L<ACADOS_INSTALL_DIR>/lib \
  -lacados -lhpipm -lblasfeo -lm \
  -Wl,-rpath,'$ORIGIN' \
  -Wl,-rpath,<ACADOS_INSTALL_DIR>/lib \
  -o run_mpc_test

Run:

./run_mpc_test


------------------------------------------------------------
IMPORTANT NOTE ON EXAMPLE C MAIN
------------------------------------------------------------

The auto-generated file:

main_p3000_mpc_model.c

does NOT initialize:
- system parameters
- stage references
- initial state constraints

If parameters are left at zero, the solver may fail with:

ACADOS_NAN_DETECTED

This is expected and is NOT a linking or build issue.

For correct execution, parameters must be set using:

p3000_mpc_model_acados_update_params(...)

before calling the solver.


------------------------------------------------------------
CURRENT STATUS
------------------------------------------------------------

- Python closed-loop NMPC: Working
- ACADOS C code generation: Working
- Standalone C solver build: Working
- Pure C execution: Working (requires correct parameter initialization)
- No MATLAB dependency
- Ready for integration into ROS2 / embedded pipeline


------------------------------------------------------------
NOTES
------------------------------------------------------------

- generated_p300_solver.py is auto-generated — do not edit
- MATLAB / Simulink files are only for model verification
- All runtime logic is Python + C (ACADOS)


------------------------------------------------------------
LICENSE
------------------------------------------------------------

ACADOS is open-source.
Project licensing will follow ACADOS licensing requirements where applicable.

