find_package(Threads REQUIRED)

file(GLOB_RECURSE larm_control_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_control STATIC ${larm_control_sources})
add_library(larm::control ALIAS larm_control)
target_include_directories(larm_control PUBLIC include)
target_link_libraries(larm_control PUBLIC larm::core larm::model larm::motion larm::hal Threads::Threads)
larm_target_warnings(larm_control)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_control_tests
        SOURCES tests/ControlCycleTest.cpp tests/RealtimeRunnerTest.cpp
        LIBRARIES larm::control)
endif()
