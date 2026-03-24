# cmake/CheckConstructorRetention.cmake
# CTest script: verify that SimObject factory and EmbeddedPython global
# constructors survive per-subsystem whole-archive linking.
#
# Invoked via:
#   cmake -DNM=<path> -DGEM5_BINARY=<path> -P CheckConstructorRetention.cmake

if(NOT NM)
    message(FATAL_ERROR "NM executable not specified")
endif()
if(NOT GEM5_BINARY)
    message(FATAL_ERROR "GEM5_BINARY not specified")
endif()
if(NOT EXISTS "${GEM5_BINARY}")
    message(FATAL_ERROR "gem5 binary not found: ${GEM5_BINARY}")
endif()

# Run nm on the binary
execute_process(
    COMMAND ${NM} "${GEM5_BINARY}"
    OUTPUT_VARIABLE _nm_output
    ERROR_VARIABLE _nm_err
    RESULT_VARIABLE _nm_result
)

if(NOT _nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed: ${_nm_err}")
endif()

# Count global constructor symbols (GLOBAL__sub_I indicates a global
# constructor from a translation unit).
string(REGEX MATCHALL "GLOBAL__sub_I" _global_ctors "${_nm_output}")
list(LENGTH _global_ctors _ctor_count)

if(_ctor_count LESS 100)
    message(FATAL_ERROR
        "Only ${_ctor_count} global constructors found in gem5 binary. "
        "Expected at least 100. Whole-archive linking may have dropped "
        "SimObject factory or EmbeddedPython registrations.")
endif()

# Check for specific known symbols that must be present:
# SimObject-related symbols
string(FIND "${_nm_output}" "SimObject" _simobj_pos)
if(_simobj_pos EQUAL -1)
    message(FATAL_ERROR "No SimObject symbols found in gem5 binary")
endif()

# EmbeddedPython-related symbols (PySource .cc global constructors)
string(FIND "${_nm_output}" "EmbeddedPython" _embpy_pos)
if(_embpy_pos EQUAL -1)
    message(FATAL_ERROR "No EmbeddedPython symbols found in gem5 binary")
endif()

message(STATUS "Constructor retention check passed: ${_ctor_count} global constructors found")
