# Stackful coroutines for the bus protocol, pinned; only the coroutine layer is built. To develop
# against a local checkout, configure with FETCHCONTENT_SOURCE_DIR_STACKFULL.
include(FetchContent)
FetchContent_Declare(stackfull
    GIT_REPOSITORY https://github.com/lvyues1994/stackfull.git
    GIT_TAG a78ba28b1d99292b01436feb485c5272a7fc20ea
    SYSTEM
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(stackfull)

file(GLOB_RECURSE larm_robstride_sources CONFIGURE_DEPENDS src/*.cpp src/*.h include/*.h)
add_library(larm_robstride STATIC ${larm_robstride_sources})
add_library(larm::robstride ALIAS larm_robstride)
target_include_directories(larm_robstride PUBLIC include)
target_link_libraries(larm_robstride PUBLIC larm::hal larm::can PRIVATE stackfull::coro)
larm_target_warnings(larm_robstride)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_robstride_tests
        SOURCES tests/CodecTest.cpp tests/DriverConfigTest.cpp tests/RobStrideDriverTest.cpp
        LIBRARIES larm::robstride)
    larm_add_test(larm_robstride_runtime_tests
        SOURCES tests/RobStrideRuntimeTest.cpp
        LIBRARIES larm::robstride larm::runtime larm::model)
endif()
