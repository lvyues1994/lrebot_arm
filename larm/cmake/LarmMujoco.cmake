# Provides mujoco::mujoco. An installed CMake package wins; otherwise the
# official release layout under LARM_MUJOCO_ROOT is wrapped as an imported target.
if(TARGET mujoco::mujoco)
    return()
endif()

find_package(mujoco CONFIG QUIET)
if(TARGET mujoco::mujoco)
    return()
endif()

if(NOT LARM_MUJOCO_ROOT)
    message(FATAL_ERROR
        "MuJoCo not found. Run tools/fetch_mujoco.sh and configure with a preset, "
        "or set LARM_MUJOCO_ROOT to an official MuJoCo release directory.")
endif()

find_path(LARM_MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS "${LARM_MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(LARM_MUJOCO_LIBRARY mujoco
    PATHS "${LARM_MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${LARM_MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LARM_MUJOCO_INCLUDE_DIR}")

set(LARM_MUJOCO_COMPILE "${LARM_MUJOCO_ROOT}/bin/compile" CACHE FILEPATH "MuJoCo model compiler")
