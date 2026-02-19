# util/tlm/CMakeLists.txt
# Standalone build for TLM (Transaction Level Modeling) integration examples.
#
# These examples demonstrate how to integrate gem5 models into a SystemC/TLM
# environment using the gem5 shared library.
#
# Prerequisites:
#   - A pre-built gem5 shared library (libgem5_shared.so)
#   - SystemC library (built as part of gem5's ext/systemc)
#
# Usage (standalone):
#   cmake -S util/tlm -B build/tlm \
#       -DGEM5_BUILD_DIR=/path/to/gem5/build_cxx
#   cmake --build build/tlm
#
# Note: This build requires the gem5 shared library to be built first.

cmake_minimum_required(VERSION 3.16)
project(gem5_tlm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
get_filename_component(GEM5_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(GEM5_BUILD_DIR "${GEM5_ROOT}/build_cxx" CACHE PATH "Path to gem5 build directory (must contain libgem5_shared and generated headers)")

# Validate that GEM5_BUILD_DIR exists and contains required files
if(NOT EXISTS "${GEM5_BUILD_DIR}")
    message(FATAL_ERROR
        "GEM5_BUILD_DIR not found: ${GEM5_BUILD_DIR}\n"
        "Build gem5 shared library first:\n"
        "  cmake -B ${GEM5_ROOT}/build_cxx -S ${GEM5_ROOT} "
        "-DGEM5_BUILD_VARIANT=ARM -DGEM5_WITH_CXX_CONFIG=ON "
        "-DGEM5_WITHOUT_PYTHON=ON\n"
        "  ninja -C ${GEM5_ROOT}/build_cxx gem5_shared\n"
        "Or specify a custom path: cmake -DGEM5_BUILD_DIR=/path/to/build ..."
    )
endif()

set(SYSTEMC_SRC_DIR "${GEM5_ROOT}/ext/systemc/src")
set(SYSTEMC_UTIL_DIR "${GEM5_ROOT}/util/systemc/gem5_within_systemc")

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
include_directories(
    ${GEM5_BUILD_DIR}
    ${GEM5_BUILD_DIR}/generated
    ${SYSTEMC_UTIL_DIR}
    ${SYSTEMC_SRC_DIR}
    ${GEM5_ROOT}/ext
    ${GEM5_ROOT}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/common
)

add_compile_definitions(
    SC_INCLUDE_DYNAMIC_PROCESSES
    TRACING_ON
)

# ---------------------------------------------------------------------------
# gem5_tlm library
# ---------------------------------------------------------------------------
# SystemC helper files (from gem5_within_systemc)
set(_systemc_helper_sources
    ${SYSTEMC_UTIL_DIR}/sc_gem5_control.cc
    ${SYSTEMC_UTIL_DIR}/sc_logger.cc
    ${SYSTEMC_UTIL_DIR}/sc_module.cc
    ${SYSTEMC_UTIL_DIR}/stats.cc
)

# TLM source files
set(_tlm_sources
    src/master_transactor.cc
    src/sc_ext.cc
    src/sc_master_port.cc
    src/sc_mm.cc
    src/sc_slave_port.cc
    src/sim_control.cc
    src/slave_transactor.cc
)

add_library(gem5_tlm STATIC
    ${_tlm_sources}
    ${_systemc_helper_sources}
)

# ---------------------------------------------------------------------------
# Common example library
# ---------------------------------------------------------------------------
add_library(tlm_examples_common STATIC
    examples/common/cli_parser.cc
    examples/common/report_handler.cc
)

# ---------------------------------------------------------------------------
# Find gem5 shared library and SystemC library
# ---------------------------------------------------------------------------
set(_gem5_lib "${GEM5_BUILD_DIR}/libgem5_shared${CMAKE_SHARED_LIBRARY_SUFFIX}")

if(NOT EXISTS "${_gem5_lib}")
    message(FATAL_ERROR
        "gem5 shared library not found: ${_gem5_lib}\n"
        "Build it first:\n"
        "  ninja -C ${GEM5_BUILD_DIR} gem5_shared"
    )
endif()

# Try to find systemc in the build tree
find_library(SYSTEMC_LIBRARY NAMES systemc
    PATHS
        "${CMAKE_CURRENT_BINARY_DIR}/build/systemc"
        "${GEM5_BUILD_DIR}"
        "${GEM5_ROOT}/ext/systemc"
    NO_DEFAULT_PATH
)
if(NOT SYSTEMC_LIBRARY)
    message(FATAL_ERROR
        "SystemC library (libsystemc) not found.\n"
        "Searched in:\n"
        "  ${CMAKE_CURRENT_BINARY_DIR}/build/systemc\n"
        "  ${GEM5_BUILD_DIR}\n"
        "  ${GEM5_ROOT}/ext/systemc\n"
        "Ensure that the gem5 build includes SystemC (ext/systemc)."
    )
endif()

# ---------------------------------------------------------------------------
# Example: master_port
# ---------------------------------------------------------------------------
add_executable(gem5_sc_master
    examples/master_port/main.cc
    examples/master_port/traffic_generator.cc
)
target_link_libraries(gem5_sc_master PRIVATE
    gem5_tlm
    tlm_examples_common
    "${_gem5_lib}"
    "${SYSTEMC_LIBRARY}"
)

# ---------------------------------------------------------------------------
# Example: slave_port
# ---------------------------------------------------------------------------
add_executable(gem5_sc_slave
    examples/slave_port/main.cc
    examples/slave_port/sc_target.cc
)
target_link_libraries(gem5_sc_slave PRIVATE
    gem5_tlm
    tlm_examples_common
    "${_gem5_lib}"
    "${SYSTEMC_LIBRARY}"
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS gem5_tlm ARCHIVE DESTINATION lib)
install(TARGETS gem5_sc_master gem5_sc_slave RUNTIME DESTINATION bin)
