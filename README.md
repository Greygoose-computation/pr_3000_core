# pr_3000_core
pr_3000_core_opc.

# Closed-Loop NMPC / OCP (Python Only for now)

This repository contains a Python-based closed-loop NMPC / OCP test.

You do NOT need MATLAB or Simulink.

The ONLY file you need to run is:

ocp_test/py_files/core_p3000/cloop_test_1.py

---

## Requirements

- Python 3.8+
- CMake and a C compiler (for acados)

---

## Clone the Repository (IMPORTANT)

This repository uses git submodules (acados).

git clone --recurse-submodules <REPO_URL>

If you already cloned without submodules:

git submodule update --init --recursive

---

## Setup

cd ocp_test/py_files/core_p3000
python3 -m venv .venv
source .venv/bin/activate
pip install numpy scipy matplotlib casadi

---

## Build acados

cd ocp_test/acados
mkdir build
cd build
cmake ..
make -j4

Add the following to your .bashrc or .zshrc:

export ACADOS_INSTALL_DIR=$(pwd)
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$ACADOS_INSTALL_DIR/interfaces/acados_template:$PYTHONPATH

Reload:

source ~/.bashrc

---

## Run

cd ocp_test/py_files/core_p3000
python cloop_test_1.py

---

## Notes

- generated_p300_solver.py is auto-generated — do not edit
- MATLAB / Simulink files are just for model verification not required at any stage of implementation 

---

## License

Since the OCP Acados is open-source , we will have to keep the src open-source as well

