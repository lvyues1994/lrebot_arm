find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(larm_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

file(GLOB_RECURSE larm_ros_sources CONFIGURE_DEPENDS src/*.cpp src/*.h include/*.h)
add_library(larm_ros STATIC ${larm_ros_sources})
add_library(larm::ros ALIAS larm_ros)
target_include_directories(larm_ros PUBLIC include)
target_link_libraries(larm_ros
    PUBLIC larm::runtime rclcpp::rclcpp
    PRIVATE lrclexec::lrclexec co2::co2 rclcpp_action::rclcpp_action
            ${builtin_interfaces_TARGETS} ${control_msgs_TARGETS} ${geometry_msgs_TARGETS} ${larm_msgs_TARGETS}
            ${rosgraph_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS})
larm_target_warnings(larm_ros)

# Each suite runs in order on one robot in one process, so each is registered as a single test on
# its own DDS domain.
function(larm_ros_suite name source domain)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE
        larm::ros larm::sim lrclexec::lrclexec rclcpp_action::rclcpp_action GTest::gtest_main
        ${control_msgs_TARGETS} ${larm_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS}
        ${trajectory_msgs_TARGETS})
    larm_test_warnings(${name})
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES
        ENVIRONMENT "LARM_ROBOT_PROFILE=${LARM_ROBOT_PROFILE};ROS_DOMAIN_ID=${domain};ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        TIMEOUT 240)
endfunction()

if(LARM_BUILD_TESTS AND LARM_WITH_MUJOCO)
    larm_ros_suite(larm_ros_tests tests/RuntimeNodeTest.cpp 231)
    larm_ros_suite(larm_remote_session_tests tests/RemoteSessionTest.cpp 230)
endif()
