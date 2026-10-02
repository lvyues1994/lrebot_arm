# -Wno-missing-field-initializers: designated initializers deliberately omit defaulted fields,
# which GCC 13 reports under -Wextra.
function(larm_target_warnings target)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
            -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wsuggest-override
            -Wzero-as-null-pointer-constant -Wimplicit-fallthrough -Werror=return-type
            -Wno-missing-field-initializers>
        $<$<BOOL:${LARM_WARNINGS_AS_ERRORS}>:-Werror>)
endfunction()

# Test bodies expand gtest macros whose comparisons trip the conversion warnings.
function(larm_test_warnings target)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wshadow -Werror=return-type -Wno-missing-field-initializers>
        $<$<BOOL:${LARM_WARNINGS_AS_ERRORS}>:-Werror>)
endfunction()
