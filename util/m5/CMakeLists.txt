# util/m5/CMakeLists.txt
# Standalone build for the m5 utility tool and libm5.
#
# The m5 utility provides a command-line interface for invoking gem5
# pseudo-instructions (magic instructions) from within simulated guest
# programs. It supports multiple target ABIs (x86, arm, arm64, etc.)
# and multiple call types (inst, addr, semi).
#
# Usage (standalone):
#   cmake -S util/m5 -B build/m5 [-DM5_ABI=x86]
#   cmake --build build/m5
#
# Or as part of the main gem5 build (integrated mode):
#   add_subdirectory(util/m5) from the top-level CMakeLists.txt
#
# Supported ABIs: x86, arm, arm64, riscv, sparc, thumb
# For cross-compilation, set CMAKE_C_COMPILER and CMAKE_CXX_COMPILER,
# or CMAKE_ASM_COMPILER as needed.

cmake_minimum_required(VERSION 3.16)
project(m5 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Locate gem5 root (two levels up from this directory)
get_filename_component(GEM5_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(M5_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# gem5 shared include directory
set(M5_COMMON_INCLUDE "${GEM5_ROOT}/include")

# ---------------------------------------------------------------------------
# ABI selection
# ---------------------------------------------------------------------------
set(M5_ABI "x86" CACHE STRING "Target ABI for the m5 utility")
set_property(CACHE M5_ABI PROPERTY STRINGS x86 arm arm64 riscv sparc thumb)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
option(M5_DEBUG_BUILD "Build with debug info and no optimizations" OFF)

if(M5_DEBUG_BUILD)
    add_compile_options(-O0 -g)
else()
    add_compile_options(-O2)
endif()

# ---------------------------------------------------------------------------
# ABI-specific configuration
# ---------------------------------------------------------------------------
# Each ABI defines:
#   - Call types (inst, addr, semi) and their implementation files
#   - Default call type
#   - M5OP_ADDR (for addr call type)
#   - Extra compiler flags

set(M5_CALL_TYPE_INST OFF)
set(M5_CALL_TYPE_ADDR OFF)
set(M5_CALL_TYPE_SEMI OFF)
set(M5_CALL_TYPE_DEFAULT "")
set(M5_ABI_SOURCES "")
set(M5_ABI_DEFINITIONS "")
set(M5_ABI_FLAGS "")

if(M5_ABI STREQUAL "x86")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_ADDR ON)
    set(M5_CALL_TYPE_DEFAULT "addr")
    set(M5_ABI_DEFINITIONS -DM5OP_ADDR=0xFFFF0000)
    set(M5_INST_IMPL abi/x86/m5op.S)
    set(M5_INST_VERIFIER abi/x86/verify_inst.cc)
    set(M5_ADDR_IMPL abi/x86/m5op_addr.S)

elseif(M5_ABI STREQUAL "arm")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_DEFAULT "inst")
    set(M5_ABI_FLAGS -march=armv7-a -mfpu=neon)
    set(M5_INST_IMPL abi/arm/m5op.S)
    set(M5_INST_VERIFIER abi/arm/verify_inst.cc)

elseif(M5_ABI STREQUAL "arm64")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_ADDR ON)
    set(M5_CALL_TYPE_SEMI ON)
    set(M5_CALL_TYPE_DEFAULT "addr")
    set(M5_ABI_DEFINITIONS -DM5OP_ADDR=0x10010000)
    set(M5_INST_IMPL abi/arm64/m5op.S)
    set(M5_INST_VERIFIER abi/arm64/verify_inst.cc)
    set(M5_ADDR_IMPL abi/arm64/m5op_addr.S)
    set(M5_SEMI_IMPL abi/arm64/m5op_semi.S)
    set(M5_SEMI_VERIFIER abi/arm64/verify_semi.cc)

elseif(M5_ABI STREQUAL "riscv")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_DEFAULT "inst")
    set(M5_INST_IMPL abi/riscv/m5op.S)
    set(M5_INST_VERIFIER abi/riscv/verify_inst.cc)

elseif(M5_ABI STREQUAL "sparc")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_DEFAULT "inst")
    set(M5_ABI_FLAGS -m64)
    set(M5_INST_IMPL abi/sparc/m5op.S)
    set(M5_INST_VERIFIER abi/sparc/verify_inst.cc)

elseif(M5_ABI STREQUAL "thumb")
    set(M5_CALL_TYPE_INST ON)
    set(M5_CALL_TYPE_DEFAULT "inst")
    set(M5_ABI_FLAGS -mthumb -march=armv7 -mfpu=neon)
    set(M5_INST_IMPL abi/thumb/m5op.S)
    set(M5_INST_VERIFIER abi/thumb/verify_inst.cc)

else()
    message(FATAL_ERROR "Unsupported M5_ABI: ${M5_ABI}")
endif()

# Collect m5op implementation files
set(_m5op_sources "")
if(M5_CALL_TYPE_INST AND DEFINED M5_INST_IMPL)
    list(APPEND _m5op_sources "${M5_SRC_DIR}/${M5_INST_IMPL}")
endif()
if(M5_CALL_TYPE_ADDR AND DEFINED M5_ADDR_IMPL)
    list(APPEND _m5op_sources "${M5_SRC_DIR}/${M5_ADDR_IMPL}")
endif()
if(M5_CALL_TYPE_SEMI AND DEFINED M5_SEMI_IMPL)
    list(APPEND _m5op_sources "${M5_SRC_DIR}/${M5_SEMI_IMPL}")
endif()

# ---------------------------------------------------------------------------
# Static library: libm5
# ---------------------------------------------------------------------------
add_library(m5_lib STATIC
    ${M5_SRC_DIR}/m5_mmap.c
    ${_m5op_sources}
)
target_include_directories(m5_lib PUBLIC
    ${M5_SRC_DIR}
    ${M5_COMMON_INCLUDE}
)
target_compile_definitions(m5_lib PUBLIC ${M5_ABI_DEFINITIONS})
if(M5_ABI_FLAGS)
    target_compile_options(m5_lib PUBLIC ${M5_ABI_FLAGS})
endif()
set_target_properties(m5_lib PROPERTIES OUTPUT_NAME m5)

# ---------------------------------------------------------------------------
# Command sources
# ---------------------------------------------------------------------------
set(_command_sources
    ${M5_SRC_DIR}/command/addsymbol.cc
    ${M5_SRC_DIR}/command/checkpoint.cc
    ${M5_SRC_DIR}/command/dumpresetstats.cc
    ${M5_SRC_DIR}/command/dumpstats.cc
    ${M5_SRC_DIR}/command/exit.cc
    ${M5_SRC_DIR}/command/fail.cc
    ${M5_SRC_DIR}/command/sum.cc
    ${M5_SRC_DIR}/command/initparam.cc
    ${M5_SRC_DIR}/command/loadsymbol.cc
    ${M5_SRC_DIR}/command/readfile.cc
    ${M5_SRC_DIR}/command/resetstats.cc
    ${M5_SRC_DIR}/command/writefile.cc
    ${M5_SRC_DIR}/command/workbegin.cc
    ${M5_SRC_DIR}/command/workend.cc
    ${M5_SRC_DIR}/command/hypercall.cc
)

# ---------------------------------------------------------------------------
# Call type sources with CALL_TYPE_IS_DEFAULT definitions
# ---------------------------------------------------------------------------
set(_ct_sources "")

if(M5_CALL_TYPE_INST)
    set(_inst_default "false")
    if(M5_CALL_TYPE_DEFAULT STREQUAL "inst")
        set(_inst_default "true")
    endif()
    list(APPEND _ct_sources "${M5_SRC_DIR}/call_type/inst.cc")
    set_source_files_properties("${M5_SRC_DIR}/call_type/inst.cc" PROPERTIES
        COMPILE_DEFINITIONS "CALL_TYPE_IS_DEFAULT=${_inst_default}")
endif()

if(M5_CALL_TYPE_ADDR)
    set(_addr_default "false")
    if(M5_CALL_TYPE_DEFAULT STREQUAL "addr")
        set(_addr_default "true")
    endif()
    list(APPEND _ct_sources "${M5_SRC_DIR}/call_type/addr.cc")
    set_source_files_properties("${M5_SRC_DIR}/call_type/addr.cc" PROPERTIES
        COMPILE_DEFINITIONS "CALL_TYPE_IS_DEFAULT=${_addr_default}")
endif()

if(M5_CALL_TYPE_SEMI)
    set(_semi_default "false")
    if(M5_CALL_TYPE_DEFAULT STREQUAL "semi")
        set(_semi_default "true")
    endif()
    list(APPEND _ct_sources "${M5_SRC_DIR}/call_type/semi.cc")
    set_source_files_properties("${M5_SRC_DIR}/call_type/semi.cc" PROPERTIES
        COMPILE_DEFINITIONS "CALL_TYPE_IS_DEFAULT=${_semi_default}")
endif()

# ---------------------------------------------------------------------------
# m5 executable (statically linked)
# ---------------------------------------------------------------------------
add_executable(m5
    ${M5_SRC_DIR}/m5.cc
    ${M5_SRC_DIR}/args.cc
    ${M5_SRC_DIR}/call_type.cc
    ${M5_SRC_DIR}/command.cc
    ${M5_SRC_DIR}/usage.cc
    ${_ct_sources}
    ${_command_sources}
)
target_include_directories(m5 PRIVATE ${M5_SRC_DIR})
target_link_libraries(m5 PRIVATE m5_lib)
target_link_options(m5 PRIVATE -no-pie -static)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS m5 RUNTIME DESTINATION bin)
install(TARGETS m5_lib ARCHIVE DESTINATION lib)
