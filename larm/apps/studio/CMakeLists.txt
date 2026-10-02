add_executable(larm_studio src/main.cpp)
target_link_libraries(larm_studio PRIVATE larm::studio larm::ros lrclexec::lrclexec)
larm_target_warnings(larm_studio)
# Installed to lib/larm next to MuJoCo in lib; Qt is found where it was linked from.
set_target_properties(larm_studio PROPERTIES INSTALL_RPATH "$ORIGIN/.." INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS larm_studio DESTINATION lib/${PROJECT_NAME})

if(LARM_BUILD_TESTS)
    add_test(NAME larm_studio_script
        COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/studio_script_test.py
                --runtime $<TARGET_FILE:larm_runtime_node> --studio $<TARGET_FILE:larm_studio>
                --profile ${LARM_ROBOT_PROFILE} --screenshots ${CMAKE_CURRENT_BINARY_DIR}/screenshots)
    set_tests_properties(larm_studio_script PROPERTIES
        ENVIRONMENT "ROS_DOMAIN_ID=229;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        TIMEOUT 300)
endif()
