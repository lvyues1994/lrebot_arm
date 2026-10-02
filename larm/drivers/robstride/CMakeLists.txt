file(GLOB_RECURSE larm_robstride_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_robstride STATIC ${larm_robstride_sources})
add_library(larm::robstride ALIAS larm_robstride)
target_include_directories(larm_robstride PUBLIC include)
target_link_libraries(larm_robstride PUBLIC larm::hal larm::can)
larm_target_warnings(larm_robstride)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_robstride_tests
        SOURCES tests/CodecTest.cpp tests/DriverConfigTest.cpp tests/RobStrideDriverTest.cpp
        LIBRARIES larm::robstride)
    larm_add_test(larm_robstride_runtime_tests
        SOURCES tests/RobStrideRuntimeTest.cpp
        LIBRARIES larm::robstride larm::runtime larm::model)
endif()
