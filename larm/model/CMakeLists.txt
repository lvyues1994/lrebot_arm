find_package(pinocchio 3.0 REQUIRED)

file(GLOB_RECURSE larm_model_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_model STATIC ${larm_model_sources})
add_library(larm::model ALIAS larm_model)
target_include_directories(larm_model PUBLIC include)
target_link_libraries(larm_model PUBLIC larm::core PRIVATE pinocchio::pinocchio)
larm_target_warnings(larm_model)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_model_tests SOURCES tests/RobotModelTest.cpp LIBRARIES larm::model)
endif()
