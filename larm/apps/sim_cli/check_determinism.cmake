# Runs the CLI twice with identical inputs and requires identical final simulation state.
foreach(run first second)
    execute_process(
        COMMAND ${CLI} --profile ${PROFILE} --target ${TARGET}
        OUTPUT_VARIABLE output_${run}
        RESULT_VARIABLE result_${run})
    if(NOT result_${run} EQUAL 0)
        message(FATAL_ERROR "run ${run} failed (${result_${run}}):\n${output_${run}}")
    endif()
    string(REGEX MATCH "\"state_digest\": \"[0-9a-f]+\"" digest_${run} "${output_${run}}")
endforeach()

if(NOT digest_first OR NOT digest_first STREQUAL digest_second)
    message(FATAL_ERROR "simulation is not deterministic: '${digest_first}' vs '${digest_second}'")
endif()
message(STATUS "deterministic: ${digest_first}")
