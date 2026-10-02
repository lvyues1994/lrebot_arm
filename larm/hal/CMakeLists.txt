file(GLOB_RECURSE larm_hal_sources CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(larm_hal STATIC ${larm_hal_sources})
add_library(larm::hal ALIAS larm_hal)
target_include_directories(larm_hal PUBLIC include)
target_link_libraries(larm_hal PUBLIC larm::core)
larm_target_warnings(larm_hal)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_hal_tests SOURCES tests/IdealBackendTest.cpp tests/MonotonicTimelineTest.cpp
        LIBRARIES larm::hal)
endif()
