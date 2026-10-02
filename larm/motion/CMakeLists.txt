find_package(ruckig 0.9 REQUIRED)

file(GLOB_RECURSE larm_motion_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_motion STATIC ${larm_motion_sources})
add_library(larm::motion ALIAS larm_motion)
target_include_directories(larm_motion PUBLIC include)
target_link_libraries(larm_motion PUBLIC larm::core PRIVATE ruckig::ruckig)
larm_target_warnings(larm_motion)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_motion_tests SOURCES tests/PointToPointTest.cpp tests/StopPlannerTest.cpp
        LIBRARIES larm::motion)
endif()
