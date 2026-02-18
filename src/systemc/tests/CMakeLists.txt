# src/systemc/tests/CMakeLists.txt
# Converted from src/systemc/tests/SConscript
#
# Builds SystemC regression test executables by scanning the systemc/ and tlm/
# subdirectories for .cpp files, mirroring the dynamic discovery logic from
# the old SConscript.
#
# Each test directory becomes one executable that links against the gem5
# static library. Metadata is written to tests.json for use by verify.py.

# Guard: only built when GEM5_WITH_SYSTEMC_TESTS is ON
# (already checked by parent CMakeLists.txt, but be explicit)
if(NOT GEM5_WITH_SYSTEMC_TESTS)
    return()
endif()

set(_sc_tests_src_dir "${CMAKE_CURRENT_SOURCE_DIR}")
# Use build_dir/systemc/tests (without src/ prefix) to match the layout
# that verify.py expects: build_dir/systemc/tests/<path>/<name>.<flavor>
set(_sc_tests_bin_dir "${CMAKE_BINARY_DIR}/systemc/tests")

# ---------------------------------------------------------------------------
# Helper: scan a subdirectory tree for SystemC test directories.
#
# For each leaf directory containing .cpp files:
#   - If a DONTRUN file is present in any ancestor, skip the entire subtree.
#   - If there is exactly one .cpp file, the test name is its stem.
#   - If there are multiple .cpp files, look for a single .f file.
#     The .f file's stem is the test name, and it lists source files
#     (one per line, relative to the parent of the test directory).
#   - A COMPILE marker means the test is compile-only (no CTest run).
#   - A DEPS marker lists extra file dependencies.
# ---------------------------------------------------------------------------

# Collect all cpp files under the two scan roots
file(GLOB_RECURSE _systemc_cpps
    RELATIVE "${_sc_tests_src_dir}"
    "${_sc_tests_src_dir}/systemc/*.cpp"
)
file(GLOB_RECURSE _tlm_cpps
    RELATIVE "${_sc_tests_src_dir}"
    "${_sc_tests_src_dir}/tlm/*.cpp"
)
set(_all_cpps ${_systemc_cpps} ${_tlm_cpps})

# Discover directories that contain a DONTRUN marker (skip those subtrees)
file(GLOB_RECURSE _dontrun_files
    "${_sc_tests_src_dir}/systemc/DONTRUN"
    "${_sc_tests_src_dir}/tlm/DONTRUN"
)
set(_dontrun_dirs "")
foreach(_dr ${_dontrun_files})
    get_filename_component(_dr_dir "${_dr}" DIRECTORY)
    file(RELATIVE_PATH _dr_rel "${_sc_tests_src_dir}" "${_dr_dir}")
    list(APPEND _dontrun_dirs "${_dr_rel}")
endforeach()

# Group .cpp files by their containing directory
set(_test_dirs "")
foreach(_cpp ${_all_cpps})
    get_filename_component(_dir "${_cpp}" DIRECTORY)
    list(APPEND _test_dirs "${_dir}")
endforeach()
list(REMOVE_DUPLICATES _test_dirs)
list(SORT _test_dirs)

# Filter out directories under any DONTRUN subtree
foreach(_dr ${_dontrun_dirs})
    list(FILTER _test_dirs EXCLUDE REGEX "^${_dr}")
endforeach()

# Track test metadata for tests.json generation
set(_tests_json_entries "")

# Relaxed compile flags for test sources (mirrors SCons: remove -Wall -Wundef
# -Wextra -Werror)
set(_sc_test_removed_flags -Wall -Wundef -Wextra -Werror)

foreach(_test_dir ${_test_dirs})
    set(_abs_dir "${_sc_tests_src_dir}/${_test_dir}")

    # Collect .cpp files in this specific directory (not recursing further)
    file(GLOB _dir_cpps "${_abs_dir}/*.cpp")
    list(LENGTH _dir_cpps _num_cpps)
    if(_num_cpps EQUAL 0)
        continue()
    endif()

    # Determine test name and source list
    set(_test_name "")
    set(_test_sources "")

    if(_num_cpps EQUAL 1)
        # Single source: test name = stem of the .cpp file
        list(GET _dir_cpps 0 _single_cpp)
        get_filename_component(_test_name "${_single_cpp}" NAME_WE)
        set(_test_sources "${_single_cpp}")
    else()
        # Multiple sources: look for a .f file
        file(GLOB _f_files "${_abs_dir}/*.f")
        list(LENGTH _f_files _num_f)
        if(NOT _num_f EQUAL 1)
            message(WARNING "SystemC test: ${_test_dir} has ${_num_cpps} .cpp "
                "files and ${_num_f} .f files (expected 1). Skipping.")
            continue()
        endif()
        list(GET _f_files 0 _f_file)
        get_filename_component(_test_name "${_f_file}" NAME_WE)

        # Read source list from .f file. Paths are relative to the parent
        # of the test directory (i.e., the .f file says "subdir/file.cpp"
        # meaning <parent_of_test_dir>/subdir/file.cpp, which is
        # <test_dir>/../<line>).
        get_filename_component(_parent_dir "${_abs_dir}" DIRECTORY)
        file(STRINGS "${_f_file}" _f_lines)
        foreach(_line ${_f_lines})
            string(STRIP "${_line}" _line)
            if(NOT _line STREQUAL "")
                set(_src_path "${_parent_dir}/${_line}")
                get_filename_component(_src_path "${_src_path}" ABSOLUTE)
                list(APPEND _test_sources "${_src_path}")
            endif()
        endforeach()
    endif()

    if(_test_name STREQUAL "" OR NOT _test_sources)
        continue()
    endif()

    # Derive a unique CMake target name from the relative path
    # e.g. systemc/kernel/sc_event/test01 -> sc_test_systemc_kernel_sc_event_test01
    string(REPLACE "/" "_" _target_name "${_test_dir}")
    set(_target_name "sc_test_${_target_name}")

    # Check for COMPILE marker (compile-only, no run)
    set(_compile_only FALSE)
    if(EXISTS "${_abs_dir}/COMPILE")
        set(_compile_only TRUE)
    endif()

    # Check for DEPS marker
    set(_test_deps "")
    if(EXISTS "${_abs_dir}/DEPS")
        file(STRINGS "${_abs_dir}/DEPS" _dep_lines)
        foreach(_dep ${_dep_lines})
            string(STRIP "${_dep}" _dep)
            if(NOT _dep STREQUAL "")
                list(APPEND _test_deps "${_abs_dir}/${_dep}")
            endif()
        endforeach()
    endif()

    # Create the test executable (EXCLUDE_FROM_ALL so `ninja` alone does not
    # build hundreds of test binaries; use `ninja systemc_tests` instead).
    add_executable(${_target_name} EXCLUDE_FROM_ALL ${_test_sources})

    # Include paths: the tests/include directory has shared test headers
    target_include_directories(${_target_name} PRIVATE
        "${_sc_tests_src_dir}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${GEM5_GEN_DIR}"
    )

    # Link against gem5 (whole-archive to pull in global constructors)
    target_link_libraries(${_target_name} PRIVATE
        -Wl,--whole-archive gem5_all -Wl,--no-whole-archive
    )

    # Remove strict warning flags for test sources
    get_target_property(_existing_flags ${_target_name} COMPILE_OPTIONS)
    if(_existing_flags)
        foreach(_flag ${_sc_test_removed_flags})
            list(REMOVE_ITEM _existing_flags "${_flag}")
        endforeach()
        set_target_properties(${_target_name} PROPERTIES
            COMPILE_OPTIONS "${_existing_flags}")
    endif()
    # Also strip these flags via compile options to override any inherited ones
    foreach(_flag ${_sc_test_removed_flags})
        target_compile_options(${_target_name} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>)
    endforeach()
    target_compile_options(${_target_name} PRIVATE -Wno-all -Wno-undef -Wno-extra)

    add_dependencies(${_target_name} gem5_all)

    # Set output directory to mirror source tree layout.
    # Use <name>.opt suffix to match verify.py expectations (name.flavor)
    # and avoid conflicts with CTest's built-in "test" target.
    set_target_properties(${_target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${_sc_tests_bin_dir}/${_test_dir}"
        OUTPUT_NAME "${_test_name}.opt"
    )

    # File-level dependencies (DEPS marker)
    if(_test_deps)
        set_source_files_properties(${_test_sources} PROPERTIES
            OBJECT_DEPENDS "${_test_deps}")
    endif()

    # Register with CTest (unless compile-only)
    if(NOT _compile_only)
        add_test(
            NAME ${_target_name}
            COMMAND ${_target_name}
            WORKING_DIRECTORY "${_sc_tests_bin_dir}/${_test_dir}"
        )
    endif()

    # Accumulate JSON metadata for verify.py
    # Convert CMake booleans to JSON booleans (lowercase true/false)
    if(_compile_only)
        set(_compile_only_json "true")
    else()
        set(_compile_only_json "false")
    endif()
    string(APPEND _tests_json_entries
        "\"${_test_dir}/${_test_name}\": {\"name\": \"${_test_name}\", "
        "\"path\": \"${_test_dir}\", "
        "\"compile_only\": ${_compile_only_json}, "
        "\"deps\": []},\n"
    )
endforeach()

# ---------------------------------------------------------------------------
# Convenience target: build all SystemC regression tests
# ---------------------------------------------------------------------------
add_custom_target(systemc_tests)
foreach(_test_dir ${_test_dirs})
    string(REPLACE "/" "_" _target_name "${_test_dir}")
    set(_target_name "sc_test_${_target_name}")
    if(TARGET ${_target_name})
        add_dependencies(systemc_tests ${_target_name})
    endif()
endforeach()

# ---------------------------------------------------------------------------
# Generate tests.json (used by verify.py to discover test binaries)
# ---------------------------------------------------------------------------
# Strip the trailing comma from the last JSON entry
string(REGEX REPLACE ",\n$" "\n" _tests_json_entries "${_tests_json_entries}")

file(GENERATE
    OUTPUT "${_sc_tests_bin_dir}/tests.json"
    CONTENT "{${_tests_json_entries}}"
)
