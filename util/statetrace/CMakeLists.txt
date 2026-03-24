# util/statetrace/CMakeLists.txt
# Standalone build for the statetrace utility.
# Traces architectural state of a running process via ptrace.
#
# Usage (standalone):
#   cmake -S util/statetrace -B build/statetrace -DSTATETRACE_ARCH=amd64
#   cmake --build build/statetrace
#
# Supported architectures: amd64, arm, i686, sparc
# Cross-compilation: set CMAKE_CXX_COMPILER to the appropriate cross compiler.

cmake_minimum_required(VERSION 3.16)
project(statetrace LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Architecture selection
set(STATETRACE_ARCH "amd64" CACHE STRING "Target architecture for statetrace")
set_property(CACHE STATETRACE_ARCH PROPERTY STRINGS amd64 arm i686 sparc)

# Validate architecture
set(_valid_arches amd64 arm i686 sparc)
list(FIND _valid_arches "${STATETRACE_ARCH}" _arch_idx)
if(_arch_idx EQUAL -1)
    message(FATAL_ERROR
        "Invalid STATETRACE_ARCH: ${STATETRACE_ARCH}\n"
        "Valid values: ${_valid_arches}")
endif()

string(TOUPPER "${STATETRACE_ARCH}" _arch_upper)

add_executable(statetrace
    base/statetrace.cc
    base/tracechild.cc
    arch/${STATETRACE_ARCH}/tracechild.cc
)

target_include_directories(statetrace PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(statetrace PRIVATE
    __STATETRACE_${_arch_upper}__
)

target_compile_options(statetrace PRIVATE -O3 -ggdb)

install(TARGETS statetrace RUNTIME DESTINATION bin)
