# cmake/Gem5Targets.cmake
# Final link targets: gem5 executable, static/shared libraries, unit tests.
#
# This module must be included AFTER all add_subdirectory() calls and
# gem5_build_gem5py_m5(), so that all global source properties are populated.

include(CTest)

# ---------------------------------------------------------------------------
# Collect all sources into a single list
# ---------------------------------------------------------------------------
gem5_get_all_sources(_all_sources)

# Build the library source list: everything EXCEPT main.cc
set(_lib_sources ${_all_sources})
list(FILTER _lib_sources EXCLUDE REGEX "sim/main\\.cc$")

# date.cc is compiled as part of the library (embeds compile timestamp)
list(APPEND _lib_sources "${CMAKE_SOURCE_DIR}/src/base/date.cc")

# ---------------------------------------------------------------------------
# PySource object library (breaks gem5py_m5 dependency cycle)
# ---------------------------------------------------------------------------
# PySource .cc files are compiled separately so that gem5_all can depend on
# SimObject codegen targets (which depend on gem5py_m5) without creating a
# dependency cycle through shared source files.
get_property(_py_srcs GLOBAL PROPERTY GEM5_PYSOURCES)
add_library(gem5_pysources OBJECT ${_py_srcs})
target_include_directories(gem5_pysources PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${GEM5_GEN_DIR}"
)
target_link_libraries(gem5_pysources PRIVATE pybind11::pybind11)
set_target_properties(gem5_pysources PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Common dependencies (INTERFACE library to avoid duplication)
# ---------------------------------------------------------------------------
add_library(gem5_deps INTERFACE)

target_include_directories(gem5_deps INTERFACE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/ext"
    "${CMAKE_SOURCE_DIR}/include"
    "${GEM5_GEN_DIR}"
)

# Always-built ext libraries
target_link_libraries(gem5_deps INTERFACE
    gem5_ext_elf
    gem5_ext_fputils
    gem5_ext_libfdt
    gem5_ext_nomali
    gem5_ext_drampower
    gem5_ext_iostream3
    gem5_ext_magic_enum
    gem5_ext_softfloat
    gem5_ext_systemc
)

# Optional ext libraries
if(HAVE_DRAMSIM)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsim2)
endif()
if(HAVE_DRAMSIM3)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsim3)
endif()
if(HAVE_DRAMSYS)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsys)
endif()

# System / found libraries
target_link_libraries(gem5_deps INTERFACE
    ZLIB::ZLIB
    Python3::Python
    pybind11::embed
    Threads::Threads
)

if(HAVE_PROTOBUF)
    target_link_libraries(gem5_deps INTERFACE protobuf::libprotobuf)
endif()
if(HAVE_PNG)
    target_link_libraries(gem5_deps INTERFACE PNG::PNG)
endif()
if(HAVE_HDF5)
    target_link_libraries(gem5_deps INTERFACE ${HDF5_CXX_LIBRARIES})
    target_include_directories(gem5_deps INTERFACE ${HDF5_CXX_INCLUDE_DIRS})
endif()
if(HAVE_TCMALLOC)
    target_link_libraries(gem5_deps INTERFACE ${TCMALLOC_LIB})
endif()
if(HAVE_CAPSTONE)
    target_link_libraries(gem5_deps INTERFACE ${CAPSTONE_LIB})
    target_include_directories(gem5_deps INTERFACE ${CAPSTONE_INCLUDE_DIR})
endif()
if(HAVE_LIBRT)
    target_link_libraries(gem5_deps INTERFACE rt)
endif()

# -rdynamic so Python extensions can see gem5 symbols
if(NOT WIN32)
    target_link_options(gem5_deps INTERFACE -rdynamic)
endif()

# ---------------------------------------------------------------------------
# Code generation target dependencies
# ---------------------------------------------------------------------------
# Phase 1 targets: debug flags, ISA parser (no gem5py_m5 dependency)
get_property(_codegen_phase1 GLOBAL PROPERTY GEM5_CODEGEN_TARGETS_PHASE1)

# ---------------------------------------------------------------------------
# Static library: libgem5_all
# ---------------------------------------------------------------------------
add_library(gem5_all STATIC ${_lib_sources})
target_link_libraries(gem5_all PUBLIC gem5_deps)

# Link PySource objects into gem5_all
target_sources(gem5_all PRIVATE $<TARGET_OBJECTS:gem5_pysources>)

# Phase 1 codegen targets (debug flags, ISA parser): created in subdirectories.
# Target-level deps ensure ordering before compilation starts.
if(_codegen_phase1)
    add_dependencies(gem5_all ${_codegen_phase1})
endif()

# Phase 2 codegen (SimObject params/enums): custom commands are created at the
# top level via gem5_create_simobject_commands(), so file-level dependencies
# work correctly with Ninja (no cross-directory stubs).

set_target_properties(gem5_all PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(TARGET slicc_${CONF_PROTOCOL})
    add_dependencies(gem5_all slicc_${CONF_PROTOCOL})
endif()

# ---------------------------------------------------------------------------
# gem5 executable
# ---------------------------------------------------------------------------
add_executable(gem5 "${CMAKE_SOURCE_DIR}/src/sim/main.cc")
# Use --whole-archive to include all objects from gem5_all.
# Without this, the linker drops objects that are only referenced via
# global constructors (EmbeddedPython registrations, SimObject factories).
target_link_libraries(gem5 PRIVATE
    -Wl,--whole-archive gem5_all -Wl,--no-whole-archive)

# Stripped binary
add_custom_command(TARGET gem5 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:gem5>" "$<TARGET_FILE:gem5>.stripped"
    COMMAND strip "$<TARGET_FILE:gem5>.stripped"
    COMMENT "Creating stripped gem5 binary"
)

# ---------------------------------------------------------------------------
# Shared library: libgem5_shared
# ---------------------------------------------------------------------------
add_library(gem5_shared SHARED ${_lib_sources})
target_link_libraries(gem5_shared PUBLIC gem5_deps)
target_sources(gem5_shared PRIVATE $<TARGET_OBJECTS:gem5_pysources>)

if(_codegen_phase1)
    add_dependencies(gem5_shared ${_codegen_phase1})
endif()

if(TARGET slicc_${CONF_PROTOCOL})
    add_dependencies(gem5_shared slicc_${CONF_PROTOCOL})
endif()

# ---------------------------------------------------------------------------
# Unit tests (GTest)
# ---------------------------------------------------------------------------
# Discover all .test.cc files under src/ and create test executables.
# Each test links against the full gem5 static library + gtest.

# gem5 GTest logging support: provides gtestLogOutput used by many tests.
add_library(gem5_gtest_logging STATIC
    "${CMAKE_SOURCE_DIR}/src/base/gtest/logging.cc"
)
target_include_directories(gem5_gtest_logging PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/generated"
    "${CMAKE_SOURCE_DIR}/ext/googletest/googletest/include"
    "${CMAKE_SOURCE_DIR}/ext/googletest/googlemock/include"
)
target_link_libraries(gem5_gtest_logging PUBLIC gem5_ext_gtest)
set_target_properties(gem5_gtest_logging PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# gem5 GTest mock loggers: replaces Logger::getPanic()/getWarn()/etc. with
# mock versions that redirect output to gtestLogOutput instead of stderr.
# Must be linked with --whole-archive BEFORE gem5_all.
add_library(gem5_gtest_mock STATIC
    "${CMAKE_SOURCE_DIR}/src/base/gtest/logging_mock.cc"
)
target_include_directories(gem5_gtest_mock PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/generated"
    "${CMAKE_SOURCE_DIR}/ext/googletest/googletest/include"
    "${CMAKE_SOURCE_DIR}/ext/googletest/googlemock/include"
)
target_link_libraries(gem5_gtest_mock PUBLIC gem5_gtest_logging)
add_dependencies(gem5_gtest_mock gem5_all)
set_target_properties(gem5_gtest_mock PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

file(GLOB_RECURSE _test_sources "${CMAKE_SOURCE_DIR}/src/*.test.cc")

# Filter out tests from ISA/GPU directories that are not enabled in this build.
set(_isa_test_filters "")
if(NOT CONF_USE_ARM_ISA)
    list(APPEND _isa_test_filters "src/arch/arm/")
endif()
if(NOT CONF_USE_X86_ISA)
    list(APPEND _isa_test_filters "src/arch/x86/")
endif()
if(NOT CONF_USE_RISCV_ISA)
    list(APPEND _isa_test_filters "src/arch/riscv/")
endif()
if(NOT CONF_USE_SPARC_ISA)
    list(APPEND _isa_test_filters "src/arch/sparc/")
endif()
if(NOT CONF_USE_MIPS_ISA)
    list(APPEND _isa_test_filters "src/arch/mips/")
endif()
if(NOT CONF_USE_POWER_ISA)
    list(APPEND _isa_test_filters "src/arch/power/")
endif()
if(NOT CONF_BUILD_GPU)
    list(APPEND _isa_test_filters "src/arch/amdgpu/")
endif()
foreach(_filter ${_isa_test_filters})
    list(FILTER _test_sources EXCLUDE REGEX "${_filter}")
endforeach()

foreach(_test_src ${_test_sources})
    # Derive test name from path: src/base/bitfield.test.cc -> test_base_bitfield
    file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}/src" "${_test_src}")
    string(REPLACE "/" "_" _test_name "${_rel}")
    string(REPLACE ".test.cc" "" _test_name "${_test_name}")
    set(_test_name "test_${_test_name}")

    add_executable(${_test_name} EXCLUDE_FROM_ALL "${_test_src}")

    # The logging test uses skip_lib=True in SCons: it tests the REAL
    # Logger (which writes to cerr) rather than the mock (which redirects
    # to gtestLogOutput).  Link it without the mock library.
    if("${_test_name}" STREQUAL "test_base_logging")
        target_link_libraries(${_test_name} PRIVATE
            gem5_gtest_logging
            gem5_all
            gem5_ext_gmock
            gem5_ext_gtest
        )
    else()
        # Use --whole-archive for gem5_gtest_mock to force inclusion of mock
        # Logger symbols that would otherwise not be pulled from the archive.
        target_link_libraries(${_test_name} PRIVATE
            -Wl,--whole-archive gem5_gtest_mock -Wl,--no-whole-archive
            gem5_all
            gem5_ext_gmock
            gem5_ext_gtest
        )
    endif()
    target_include_directories(${_test_name} PRIVATE
        "${CMAKE_SOURCE_DIR}/ext/googletest/googletest/include"
        "${CMAKE_SOURCE_DIR}/ext/googletest/googlemock/include"
    )
    # Ensure all generated headers (enums, params from phase 2 codegen) are
    # available before compiling test sources.  The simplest way is to wait
    # for gem5_all to finish: it already depends on all codegen targets, and
    # tests link against it anyway.
    add_dependencies(${_test_name} gem5_all)
    add_test(NAME ${_test_name} COMMAND ${_test_name})
endforeach()

# Convenience target: build all unit tests
add_custom_target(gem5_tests)
foreach(_test_src ${_test_sources})
    file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}/src" "${_test_src}")
    string(REPLACE "/" "_" _test_name "${_rel}")
    string(REPLACE ".test.cc" "" _test_name "${_test_name}")
    set(_test_name "test_${_test_name}")
    add_dependencies(gem5_tests ${_test_name})
endforeach()

# ---------------------------------------------------------------------------
# Install targets
# ---------------------------------------------------------------------------
install(TARGETS gem5 RUNTIME DESTINATION bin)
install(TARGETS gem5_shared LIBRARY DESTINATION lib)
install(TARGETS gem5_all ARCHIVE DESTINATION lib)
