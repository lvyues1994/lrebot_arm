# larm_add_test(<name> SOURCES <files...> LIBRARIES <targets...>)
function(larm_add_test name)
    cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES" ${ARGN})
    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name} PRIVATE ${ARG_LIBRARIES} GTest::gtest_main)
    larm_test_warnings(${name})
    gtest_discover_tests(${name}
        DISCOVERY_MODE PRE_TEST
        PROPERTIES ENVIRONMENT "LARM_ROBOT_PROFILE=${LARM_ROBOT_PROFILE}")
endfunction()
