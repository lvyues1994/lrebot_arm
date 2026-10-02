# Cross-module tests: the control stack running against the MuJoCo backend.
larm_add_test(larm_integration_tests SOURCES ControlSimulationTest.cpp LIBRARIES larm::control larm::sim)

# Replacing malloc conflicts with sanitizer runtimes, which intercept it themselves.
if(NOT CMAKE_CXX_FLAGS MATCHES "-fsanitize")
    larm_add_test(larm_no_allocation_tests SOURCES NoAllocationTest.cpp LIBRARIES larm::control larm::sim)
    if(TARGET larm::robstride)
        target_link_libraries(larm_no_allocation_tests PRIVATE larm::robstride)
        target_compile_definitions(larm_no_allocation_tests PRIVATE LARM_HAS_ROBSTRIDE)
    endif()
endif()
