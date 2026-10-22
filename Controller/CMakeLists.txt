cmake_minimum_required(VERSION 3.16)
project(controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Pinocchio/OSQP were built locally into ~/.local; Pinocchio's own CMake
# config additionally needs ROS Foxy's urdfdom config on the prefix path.
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local" "/opt/ros/foxy")

find_package(pinocchio REQUIRED)
find_package(osqp REQUIRED)
find_package(Threads REQUIRED)

add_executable(controller
  src/main.cpp
  src/g1_model.cpp
  src/kinematics_thread.cpp
  src/leg_ik.cpp
  src/motion_gen_thread.cpp
  src/command_input_thread.cpp
  src/wbc_thread.cpp
  src/logging_thread.cpp
  src/attitude_mpc.cpp
)
target_link_libraries(controller PRIVATE pinocchio::pinocchio osqp::osqp Threads::Threads)

# Shared SysV message-queue protocol (Common/ipc_protocol.h).
target_include_directories(controller PRIVATE
  ${CMAKE_SOURCE_DIR}/../Common
  ${CMAKE_SOURCE_DIR}/include
)
target_compile_definitions(controller PRIVATE
  CONTROLLER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
)

# pinocchio/osqp were built locally into ~/.local; bake that into rpath so the
# binary runs without manually exporting LD_LIBRARY_PATH.
#
# libpinocchio.so's own rpath is just $ORIGIN, and since that's a DT_RUNPATH
# (not the older, transitively-inherited DT_RPATH), our rpath here can't
# reach past it to resolve *its* dependency on ROS Foxy's liburdfdom_model.so
# (needed once code actually calls into the URDF parser, e.g. LoadH1Model()).
# run_controller.sh exports LD_LIBRARY_PATH for that one instead, since
# LD_LIBRARY_PATH is consulted regardless of any object's rpath/runpath.
set_target_properties(controller PROPERTIES
  BUILD_RPATH "$ENV{HOME}/.local/lib"
  INSTALL_RPATH "$ENV{HOME}/.local/lib"
)

# IK 오프라인 검증 도구 (시뮬레이터 불필요).
add_executable(ik_check tools/ik_check.cpp src/g1_model.cpp src/leg_ik.cpp)
target_link_libraries(ik_check PRIVATE pinocchio::pinocchio Threads::Threads)
target_include_directories(ik_check PRIVATE ${CMAKE_SOURCE_DIR}/../Common ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(ik_check PRIVATE CONTROLLER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model")
set_target_properties(ik_check PROPERTIES BUILD_RPATH "$ENV{HOME}/.local/lib")

# 보행 패턴 오프라인 검증 도구 (header-only lipm_gait.h만 사용, 시뮬레이터/pinocchio 불필요).
add_executable(gait_check tools/gait_check.cpp)
target_include_directories(gait_check PRIVATE ${CMAKE_SOURCE_DIR}/include)

# 자세 MPC 오프라인 검증 도구 (OSQP만 필요).
add_executable(mpc_check tools/mpc_check.cpp src/attitude_mpc.cpp)
target_link_libraries(mpc_check PRIVATE osqp::osqp)
target_include_directories(mpc_check PRIVATE ${CMAKE_SOURCE_DIR}/../Common ${CMAKE_SOURCE_DIR}/include $ENV{HOME}/.local/include /usr/include/eigen3)
set_target_properties(mpc_check PROPERTIES BUILD_RPATH "$ENV{HOME}/.local/lib")

# FK/오도메트리 오프라인 검증 도구 (MuJoCo 정답과 비교). Mujoco를 먼저 빌드해야 함.
set(MUJOCO_ROOT "${CMAKE_SOURCE_DIR}/../Mujoco")
if(EXISTS "${MUJOCO_ROOT}/build/lib/libmujoco.so")
  add_executable(fk_check tools/fk_check.cpp src/g1_model.cpp src/leg_ik.cpp)
  target_link_libraries(fk_check PRIVATE pinocchio::pinocchio Threads::Threads
                        "${MUJOCO_ROOT}/build/lib/libmujoco.so")
  target_include_directories(fk_check PRIVATE ${CMAKE_SOURCE_DIR}/../Common ${CMAKE_SOURCE_DIR}/include
                             "${MUJOCO_ROOT}/mujoco_engine/include")
  target_compile_definitions(fk_check PRIVATE
    CONTROLLER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
    MUJOCO_G1_XML="${MUJOCO_ROOT}/model/g1/g1.xml")
  set_target_properties(fk_check PROPERTIES BUILD_RPATH "$ENV{HOME}/.local/lib;${MUJOCO_ROOT}/build/lib")

  # 헤드리스 폐루프 하네스 (MuJoCo + 실제 controller 바이너리, 별도 IPC 큐).
  add_executable(walk_sim tools/walk_sim.cpp ${MUJOCO_ROOT}/mujoco_engine/simulate/pid_ipc.cc)
  target_link_libraries(walk_sim PRIVATE Threads::Threads "${MUJOCO_ROOT}/build/lib/libmujoco.so")
  target_include_directories(walk_sim PRIVATE ${CMAKE_SOURCE_DIR}/../Common ${CMAKE_SOURCE_DIR}/include
                             "${MUJOCO_ROOT}/mujoco_engine/include" "${MUJOCO_ROOT}/mujoco_engine/simulate")
  target_compile_definitions(walk_sim PRIVATE
    MUJOCO_G1_XML="${MUJOCO_ROOT}/model/g1/g1.xml"
    CONTROLLER_BIN="${CMAKE_CURRENT_BINARY_DIR}/controller")
  set_target_properties(walk_sim PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}/build/lib")
  # 3D 창(view=1): Mujoco 빌드가 받아둔 GLFW(정적) + OpenGL/X11
  if(EXISTS "${MUJOCO_ROOT}/build/lib/libglfw3.a" AND EXISTS "${MUJOCO_ROOT}/build/_deps/glfw3-src/include/GLFW/glfw3.h")
    target_compile_definitions(walk_sim PRIVATE WALK_SIM_VIEWER)
    target_include_directories(walk_sim PRIVATE "${MUJOCO_ROOT}/build/_deps/glfw3-src/include")
    target_link_libraries(walk_sim PRIVATE "${MUJOCO_ROOT}/build/lib/libglfw3.a" GL X11 dl m)
  endif()

  add_executable(id_check tools/id_check.cpp src/g1_model.cpp src/leg_ik.cpp)
  target_link_libraries(id_check PRIVATE pinocchio::pinocchio Threads::Threads
                        "${MUJOCO_ROOT}/build/lib/libmujoco.so")
  target_include_directories(id_check PRIVATE ${CMAKE_SOURCE_DIR}/../Common ${CMAKE_SOURCE_DIR}/include
                             "${MUJOCO_ROOT}/mujoco_engine/include")
  target_compile_definitions(id_check PRIVATE
    CONTROLLER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
    MUJOCO_G1_XML="${MUJOCO_ROOT}/model/g1/g1.xml")
  set_target_properties(id_check PROPERTIES BUILD_RPATH "$ENV{HOME}/.local/lib;${MUJOCO_ROOT}/build/lib")
endif()
