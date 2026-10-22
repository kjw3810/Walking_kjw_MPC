cmake_minimum_required(VERSION 3.16)
project(mujoco_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# MuJoCo is vendored as source (mujoco_engine/, tracking upstream tag 3.1.2)
# and built as part of this project, so the engine itself can be edited.
# The GUI viewer (simulate) is built from source too, for the same reason.
# Pinned to 3.1.2 (C++17) rather than the latest release because this
# machine's compiler is GCC 9.4, which lacks C++20 support (e.g. `constinit`)
# that newer MuJoCo releases require.
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE ON CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(mujoco_engine)

# libmujoco.so references pthread symbols without a NEEDED entry for
# libpthread (Ubuntu 20.04 / glibc 2.31 still ships pthread separately from
# libc), so executables linking it must link Threads themselves too.
find_package(Threads REQUIRED)

add_executable(mujoco_sim src/main.cpp)
target_link_libraries(mujoco_sim PRIVATE mujoco::mujoco Threads::Threads)
target_compile_definitions(mujoco_sim PRIVATE
  MUJOCO_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
)
