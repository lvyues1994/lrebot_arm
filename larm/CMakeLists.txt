cmake_minimum_required(VERSION 3.25...3.31)
project(larm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(LARM_WARNINGS_AS_ERRORS "Treat compiler warnings in larm targets as errors" OFF)
option(LARM_BUILD_TESTS "Build the larm test suite" ${PROJECT_IS_TOP_LEVEL})
option(LARM_WITH_MUJOCO "Build the MuJoCo simulation backend and its tools" ON)
set(LARM_MUJOCO_ROOT "" CACHE PATH "Official MuJoCo release directory containing include/ and lib/")
set(LARM_ROBOT_PROFILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../robots/rebot_b601/rebot_b601_bringup/config/rebot_b601_rs.yaml"
    CACHE FILEPATH "Robot profile used by the integration tests")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(LarmCompilerWarnings)

if(LARM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    include(GoogleTest)
    include(LarmTesting)
endif()

add_subdirectory(core)
add_subdirectory(model)
add_subdirectory(motion)
add_subdirectory(hal)
add_subdirectory(control)

if(LARM_WITH_MUJOCO)
    include(LarmMujoco)
    add_subdirectory(sim)
    add_subdirectory(apps/sim_cli)
endif()

if(LARM_BUILD_TESTS AND LARM_WITH_MUJOCO)
    add_subdirectory(tests)
endif()
