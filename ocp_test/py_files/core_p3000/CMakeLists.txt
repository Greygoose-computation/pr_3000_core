cmake_minimum_required(VERSION 3.14)
project(test_p3000_mpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

set(ACADOS_INSTALL_DIR "/home/ananth/main/Personal/Peer_works/ocp_test/acados/install")
set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")

file(GLOB ACADOS_C_SOURCES
    ${CMAKE_SOURCE_DIR}/c_generated_code_p3000/*.c
    ${CMAKE_SOURCE_DIR}/c_generated_code_p3000/p3000_mpc_model_model/*.c
)

# Remove generated demo executables / sim main files (not needed for your test)
list(FILTER ACADOS_C_SOURCES EXCLUDE REGEX ".*/main_.*\\.c$")
list(FILTER ACADOS_C_SOURCES EXCLUDE REGEX ".*/acados_sim_solver_.*\\.c$")

add_executable(test_mpc
    ${CMAKE_SOURCE_DIR}/main_test01.cpp
    ${CMAKE_SOURCE_DIR}/mpccontroller.cpp
    ${ACADOS_C_SOURCES}
)

target_include_directories(test_mpc PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/hpipm/include
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${CMAKE_SOURCE_DIR}/c_generated_code_p3000
    ${CMAKE_SOURCE_DIR}/c_generated_code_p3000/p3000_mpc_model_model
)

target_link_directories(test_mpc PRIVATE
    ${ACADOS_LIB_DIR}
)

target_link_libraries(test_mpc PRIVATE
    acados
    hpipm
    blasfeo
    m
    dl
    pthread
)

# Optional: bake runtime library path so you don't need LD_LIBRARY_PATH
set_target_properties(test_mpc PROPERTIES
    BUILD_RPATH "${ACADOS_LIB_DIR}"
    INSTALL_RPATH "${ACADOS_LIB_DIR}"
)
