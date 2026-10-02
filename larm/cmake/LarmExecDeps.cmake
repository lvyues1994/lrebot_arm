# The sender/receiver libraries, pinned. To develop against a local checkout, configure with
# FETCHCONTENT_SOURCE_DIR_LEXEC, FETCHCONTENT_SOURCE_DIR_CO2 or FETCHCONTENT_SOURCE_DIR_LRCLEXEC.
# lrclexec reuses the lexec::lexec target provided here, so the program has a single lexec.
include(FetchContent)

FetchContent_Declare(lexec
    GIT_REPOSITORY https://github.com/lvyues1994/lexec.git
    GIT_TAG a2568cc5bf14f69addc7d0e04b1e5ed5c0b2173b
    SYSTEM)
FetchContent_Declare(co2
    GIT_REPOSITORY https://github.com/lvyues1994/coro.git
    GIT_TAG a265e5771b0e3ca68d6d473b9bb410fadddf4c23
    SYSTEM)

set(LEXEC_BUILD_TESTS OFF)
set(CO2_BUILD_TESTS OFF)
FetchContent_MakeAvailable(lexec co2)

# larm uses no standard execution policies. Without this, C++20 pulls <execution> and with it TBB
# into every user of lexec; TBB's headers then clash with Qt's `emit` macro. Set on the target so
# every consumer, lrclexec and lqtexec included, sees the same lexec.
target_compile_definitions(lexec INTERFACE LEXEC_NO_STD_EXECUTION_POLICY)

if(LARM_WITH_ROS)
    FetchContent_Declare(lrclexec
        GIT_REPOSITORY https://github.com/lvyues1994/lrclexec.git
        GIT_TAG 30880884d373e207c3f33dadf85ec61e2a9b1c5a
        SYSTEM)
    FetchContent_MakeAvailable(lrclexec)
endif()
