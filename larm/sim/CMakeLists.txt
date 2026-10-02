file(GLOB_RECURSE larm_sim_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_sim STATIC ${larm_sim_sources})
add_library(larm::sim ALIAS larm_sim)
target_include_directories(larm_sim PUBLIC include)
target_link_libraries(larm_sim PUBLIC larm::hal mujoco::mujoco)
larm_target_warnings(larm_sim)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_sim_tests SOURCES tests/SimulationTest.cpp tests/SceneMirrorTest.cpp
        LIBRARIES larm::sim larm::model)
endif()
