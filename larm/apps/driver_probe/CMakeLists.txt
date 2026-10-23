add_executable(larm_driver_probe src/main.cpp src/ReadOnlyBus.cpp)
target_link_libraries(larm_driver_probe PRIVATE larm::robstride larm::runtime larm::model)
larm_target_warnings(larm_driver_probe)
install(TARGETS larm_driver_probe DESTINATION lib/${PROJECT_NAME})

if(LARM_BUILD_TESTS)
    # Rehearsals against simulated motors; the hardware path is the same code.
    set(probe $<TARGET_FILE:larm_driver_probe> --profile ${LARM_ROBOT_PROFILE} --simulated)
    add_test(NAME larm_driver_probe_scan COMMAND ${probe} scan)
    add_test(NAME larm_driver_probe_hold COMMAND ${probe} hold 1 --confirm-power)
    add_test(NAME larm_driver_probe_jog COMMAND ${probe} jog joint1 0.15 --confirm-motion)
    add_test(NAME larm_driver_probe_joints_jog
             COMMAND ${probe} --joints joint1,joint2 jog joint2 0.05 --speed 0.05 --confirm-motion)
    add_test(NAME larm_driver_probe_joints_scan COMMAND ${probe} --joints joint1,joint2 scan)
    add_test(NAME larm_driver_probe_grip COMMAND ${probe} --joints gripper grip 0.005 --confirm-motion)
    add_test(NAME larm_driver_probe_pose COMMAND ${probe} pose ready 1 --confirm-motion)
    add_test(NAME larm_driver_probe_needs_confirmation COMMAND ${probe} hold 1)
    set_tests_properties(larm_driver_probe_needs_confirmation PROPERTIES WILL_FAIL TRUE)
    set_tests_properties(larm_driver_probe_joints_scan PROPERTIES PASS_REGULAR_EXPRESSION
                         "\"answered\":2,\"actuators\":2,.*\"ready\":true")
    set_tests_properties(larm_driver_probe_scan larm_driver_probe_hold larm_driver_probe_jog
                         larm_driver_probe_joints_jog larm_driver_probe_joints_scan larm_driver_probe_grip
                         larm_driver_probe_pose
                         PROPERTIES TIMEOUT 60)
endif()
