# Cross-module tests: the control stack running against the MuJoCo backend.
larm_add_test(larm_integration_tests SOURCES ControlSimulationTest.cpp LIBRARIES larm::control larm::sim)

# Replacing malloc conflicts with sanitizer runtimes, which intercept it themselves.
if(NOT CMAKE_CXX_FLAGS MATCHES "-fsanitize")
    larm_add_test(larm_no_allocation_tests SOURCES NoAllocationTest.cpp LIBRARIES larm::control larm::sim)
endif()
