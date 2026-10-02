find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(yaml-cpp 0.7 REQUIRED)
find_package(tl-expected CONFIG REQUIRED)

file(GLOB_RECURSE larm_core_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_core STATIC ${larm_core_sources})
add_library(larm::core ALIAS larm_core)
target_include_directories(larm_core PUBLIC include)
target_link_libraries(larm_core PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp tl::expected)
larm_target_warnings(larm_core)

if(LARM_BUILD_TESTS)
    find_package(Threads REQUIRED)
    larm_add_test(larm_core_tests
        SOURCES tests/RobotProfileTest.cpp tests/SpscRingTest.cpp tests/LatestValueTest.cpp tests/PoseTest.cpp
        LIBRARIES larm::core Threads::Threads)
endif()
