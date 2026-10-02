# Provides mujoco::mujoco. An installed CMake package wins; otherwise the official release layout
# under LARM_MUJOCO_ROOT (default: the copy tools/fetch_mujoco.sh places in .deps) is wrapped.
if(TARGET mujoco::mujoco)
    return()
endif()

find_package(mujoco CONFIG QUIET)
if(TARGET mujoco::mujoco)
    return()
endif()

set(larm_default_mujoco "${CMAKE_CURRENT_LIST_DIR}/../../.deps/mujoco-3.8.0")
if(NOT LARM_MUJOCO_ROOT AND EXISTS "${larm_default_mujoco}/include/mujoco/mujoco.h")
    cmake_path(NORMAL_PATH larm_default_mujoco OUTPUT_VARIABLE larm_default_mujoco)
    set(LARM_MUJOCO_ROOT "${larm_default_mujoco}" CACHE PATH "" FORCE)
endif()
if(NOT LARM_MUJOCO_ROOT)
    message(FATAL_ERROR
        "MuJoCo not found. Run tools/fetch_mujoco.sh or set LARM_MUJOCO_ROOT to an official release directory.")
endif()

find_path(LARM_MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS "${LARM_MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(LARM_MUJOCO_LIBRARY mujoco
    PATHS "${LARM_MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${LARM_MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LARM_MUJOCO_INCLUDE_DIR}")

# Installed programs load this copy through an $ORIGIN-relative RPATH. The release's real file name
# (libmujoco.so.<version>) is also its soname.
file(REAL_PATH "${LARM_MUJOCO_LIBRARY}" larm_mujoco_runtime)
install(FILES "${larm_mujoco_runtime}" TYPE LIB)
