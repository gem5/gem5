set(GEM5_BUILD_VARIANT "X86" CACHE STRING "gem5 build variant (see build_opts/)")

# file(GLOB) is used here only for validation — to enumerate valid variant names
# from build_opts/. This is an accepted exception to the no-glob rule: we are
# not registering source files (where stale globs cause silent missing-file
# bugs), just checking that the user's variant string matches a known file.
file(GLOB _variant_files LIST_DIRECTORIES false
    "${PROJECT_SOURCE_DIR}/build_opts/*")
set(_valid_variants "")
foreach(_f ${_variant_files})
    cmake_path(GET _f FILENAME _name)
    list(APPEND _valid_variants "${_name}")
endforeach()

if(NOT GEM5_BUILD_VARIANT IN_LIST _valid_variants)
    message(FATAL_ERROR
        "Invalid GEM5_BUILD_VARIANT '${GEM5_BUILD_VARIANT}'. "
        "Valid values: ${_valid_variants}")
endif()

# Parse build_opts/<VARIANT> into CMake variables. The file format is a subset
# of Kconfig: KEY=y|n for booleans, KEY="value" or KEY=value for strings.
# These variables (USE_X86_ISA, RUBY, PROTOCOL, etc.) are consumed by
# gem5_add_source(CONDITION ...) calls in PR 2.
#
# No PARENT_SCOPE: include() runs in the caller's scope, so plain set() writes
# directly into CMakeLists.txt's scope without any propagation needed.
file(STRINGS "${PROJECT_SOURCE_DIR}/build_opts/${GEM5_BUILD_VARIANT}" _kconfig_lines)
foreach(_line ${_kconfig_lines})
    if(_line MATCHES "^([A-Za-z0-9_]+)=(.*)$")
        set(_key   "${CMAKE_MATCH_1}")
        set(_value "${CMAKE_MATCH_2}")

        string(REGEX REPLACE "^\"(.*)\"$" "\\1" _value "${_value}")

        if(_value STREQUAL "y")
            set(${_key} TRUE)
        elseif(_value STREQUAL "n")
            set(${_key} FALSE)
        else()
            set(${_key} "${_value}")
        endif()
    endif()
endforeach()

message(STATUS "gem5 build variant: ${GEM5_BUILD_VARIANT}")
