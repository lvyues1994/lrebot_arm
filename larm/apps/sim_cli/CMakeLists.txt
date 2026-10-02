add_executable(larm_sim_cli src/main.cpp)
target_link_libraries(larm_sim_cli PRIVATE larm::control larm::sim)
larm_target_warnings(larm_sim_cli)

if(LARM_BUILD_TESTS)
    set(target_pose "0.8,1.0,1.4,-0.5,0.4,1.0,0.04")
    add_test(NAME larm_sim_cli.runs_trajectory
        COMMAND larm_sim_cli --profile ${LARM_ROBOT_PROFILE} --target ${target_pose})
    add_test(NAME larm_sim_cli.cancel_stops
        COMMAND larm_sim_cli --profile ${LARM_ROBOT_PROFILE} --target ${target_pose} --cancel-after 0.5)
    add_test(NAME larm_sim_cli.is_deterministic
        COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:larm_sim_cli> -DPROFILE=${LARM_ROBOT_PROFILE}
                -DTARGET=${target_pose} -P ${CMAKE_CURRENT_SOURCE_DIR}/check_determinism.cmake)
endif()
