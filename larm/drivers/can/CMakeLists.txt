add_library(larm_can STATIC src/SocketCan.cpp include/larm/drivers/can/CanTransport.h)
add_library(larm::can ALIAS larm_can)
target_include_directories(larm_can PUBLIC include)
target_link_libraries(larm_can PUBLIC larm::core)
larm_target_warnings(larm_can)

if(LARM_BUILD_TESTS)
    larm_add_test(larm_can_tests SOURCES tests/SocketCanTest.cpp LIBRARIES larm::can)
endif()
