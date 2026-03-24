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
# Call type OBJECT libraries with per-type CALL_TYPE_IS_DEFAULT definitions.
# We use OBJECT libraries instead of set_source_files_properties so that the
# per-file definitions don't leak into other targets (ABI tests) that compile
# the same source files with different flags.
# ---------------------------------------------------------------------------
set(_ct_objects "")

foreach(_ct_name inst addr semi)
    string(TOUPPER "${_ct_name}" _ct_upper)
    if(M5_CALL_TYPE_${_ct_upper})
        set(_is_default "false")
        if(M5_CALL_TYPE_DEFAULT STREQUAL "${_ct_name}")
            set(_is_default "true")
        endif()
        add_library(m5_ct_${_ct_name} OBJECT "${M5_SRC_DIR}/call_type/${_ct_name}.cc")
        target_include_directories(m5_ct_${_ct_name} PRIVATE ${M5_SRC_DIR} ${M5_COMMON_INCLUDE})
        target_compile_definitions(m5_ct_${_ct_name} PRIVATE
            ${M5_ABI_DEFINITIONS} "CALL_TYPE_IS_DEFAULT=${_is_default}")
        if(M5_ABI_FLAGS)
            target_compile_options(m5_ct_${_ct_name} PRIVATE ${M5_ABI_FLAGS})
        endif()
        list(APPEND _ct_objects $<TARGET_OBJECTS:m5_ct_${_ct_name}>)
    endif()
endforeach()

# ---------------------------------------------------------------------------
# m5 executable (statically linked)
# ---------------------------------------------------------------------------
add_executable(m5
    ${M5_SRC_DIR}/m5.cc
    ${M5_SRC_DIR}/args.cc
    ${M5_SRC_DIR}/call_type.cc
    ${M5_SRC_DIR}/command.cc
    ${M5_SRC_DIR}/usage.cc
    ${_ct_objects}
    ${_command_sources}
)
target_include_directories(m5 PRIVATE ${M5_SRC_DIR})
target_link_libraries(m5 PRIVATE m5_lib)
target_link_options(m5 PRIVATE -no-pie -static)

# ---------------------------------------------------------------------------
# Shared (PIC) m5op objects -- used by JNI and Lua wrappers
# ---------------------------------------------------------------------------
# The SCons build compiled the m5op .S files with -DM5OP_PIC for shared
# targets. We create a PIC object library that both JNI and Lua link against.
set(_m5op_pic_sources ${_m5op_sources} ${M5_SRC_DIR}/m5_mmap.c)
if(_m5op_pic_sources)
    add_library(m5op_pic OBJECT ${_m5op_pic_sources})
    target_include_directories(m5op_pic PUBLIC ${M5_SRC_DIR} ${M5_COMMON_INCLUDE})
    target_compile_definitions(m5op_pic PUBLIC ${M5_ABI_DEFINITIONS} M5OP_PIC)
    set_target_properties(m5op_pic PROPERTIES POSITION_INDEPENDENT_CODE ON)
    if(M5_ABI_FLAGS)
        target_compile_options(m5op_pic PUBLIC ${M5_ABI_FLAGS})
    endif()
endif()

# ---------------------------------------------------------------------------
# Java JNI wrapper (conditional on JDK availability)
# ---------------------------------------------------------------------------
find_package(Java COMPONENTS Development QUIET)
find_package(JNI QUIET)

if(Java_FOUND AND JNI_FOUND)
    message(STATUS "Java and JNI found -- building Java JNI wrapper")

    set(_jni_java_src_dir "${M5_SRC_DIR}/java")
    set(_jni_java_out_dir "${CMAKE_CURRENT_BINARY_DIR}/java")
    set(_jni_class_dir "${_jni_java_out_dir}")
    set(_jni_header "${_jni_java_out_dir}/gem5_Ops.h")

    # Compile Java sources and generate JNI header with javac -h.
    file(GLOB _java_sources "${_jni_java_src_dir}/gem5/*.java")
    # Filter out test sources if JUnit is not available.
    list(FILTER _java_sources EXCLUDE REGEX ".*Test\\.java$")

    set(_class_files "")
    foreach(_jsrc ${_java_sources})
        get_filename_component(_jname ${_jsrc} NAME_WE)
        list(APPEND _class_files "${_jni_class_dir}/gem5/${_jname}.class")
    endforeach()

    add_custom_command(
        OUTPUT ${_jni_header} ${_class_files}
        COMMAND ${Java_JAVAC_EXECUTABLE} -d "${_jni_class_dir}"
                -h "${_jni_java_out_dir}" ${_java_sources}
        DEPENDS ${_java_sources}
        COMMENT "Compiling Java sources and generating JNI header"
        VERBATIM
    )
    add_custom_target(m5_jni_header DEPENDS ${_jni_header})

    # Build the JNI shared library.
    add_library(gem5Ops SHARED ${M5_SRC_DIR}/java/gem5/ops.cc)
    add_dependencies(gem5Ops m5_jni_header)
    target_include_directories(gem5Ops PRIVATE
        ${M5_SRC_DIR}
        ${M5_COMMON_INCLUDE}
        ${_jni_java_out_dir}          # generated gem5_Ops.h
        ${JNI_INCLUDE_DIRS}
    )
    target_link_libraries(gem5Ops PRIVATE m5op_pic)

    # Set CALL_TYPE_DEFAULT and CALL_TYPE_*_ENABLED defines for JNI.
    target_compile_definitions(gem5Ops PRIVATE
        "CALL_TYPE_DEFAULT=${M5_CALL_TYPE_DEFAULT}"
    )
    foreach(_ct_name inst addr semi)
        string(TOUPPER "${_ct_name}" _ct_upper)
        if(M5_CALL_TYPE_${_ct_upper})
            target_compile_definitions(gem5Ops PRIVATE "CALL_TYPE_${_ct_name}_ENABLED=1")
        else()
            target_compile_definitions(gem5Ops PRIVATE "CALL_TYPE_${_ct_name}_ENABLED=0")
        endif()
    endforeach()

    # Build the JAR file containing class files and the shared library.
    set(_jar_file "${CMAKE_CURRENT_BINARY_DIR}/gem5Ops.jar")
    add_custom_command(
        OUTPUT ${_jar_file}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gem5Ops> "${_jni_class_dir}/"
        COMMAND ${Java_JAR_EXECUTABLE} cf ${_jar_file} -C "${_jni_class_dir}" .
        DEPENDS gem5Ops ${_class_files}
        COMMENT "Creating gem5Ops.jar"
        VERBATIM
    )
    add_custom_target(gem5Ops_jar ALL DEPENDS ${_jar_file})
else()
    if(NOT Java_FOUND)
        message(STATUS "Java not found -- skipping Java JNI wrapper")
    endif()
    if(NOT JNI_FOUND)
        message(STATUS "JNI not found -- skipping Java JNI wrapper")
    endif()
endif()

# ---------------------------------------------------------------------------
# Lua wrapper (conditional on Lua 5.1 availability)
# ---------------------------------------------------------------------------
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LUA51 QUIET lua51)
endif()
# Fallback: try CMake's FindLua if pkg-config didn't find lua51.
if(NOT LUA51_FOUND)
    find_package(Lua 5.1 QUIET)
    if(LUA_FOUND AND LUA_VERSION_STRING VERSION_LESS "5.2")
        set(LUA51_FOUND TRUE)
        set(LUA51_INCLUDE_DIRS "${LUA_INCLUDE_DIR}")
        set(LUA51_LIBRARIES "${LUA_LIBRARIES}")
    endif()
endif()

if(LUA51_FOUND)
    message(STATUS "Lua 5.1 found -- building Lua wrapper")
    add_library(gem5OpLua SHARED
        ${M5_SRC_DIR}/lua_gem5Op.cc
    )
    target_include_directories(gem5OpLua PRIVATE
        ${M5_SRC_DIR}
        ${M5_COMMON_INCLUDE}
        ${LUA51_INCLUDE_DIRS}
    )
    target_link_libraries(gem5OpLua PRIVATE m5op_pic)
    if(LUA51_LIBRARIES)
        target_link_libraries(gem5OpLua PRIVATE ${LUA51_LIBRARIES})
    endif()
else()
    message(STATUS "Lua 5.1 not found -- skipping Lua wrapper")
endif()

# ---------------------------------------------------------------------------
# Tests (native, using googletest via CTest)
# ---------------------------------------------------------------------------
option(M5_BUILD_TESTS "Build m5 utility unit tests" ON)

if(M5_BUILD_TESTS)
    enable_testing()

    # Bring in googletest from ext/.
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    add_subdirectory("${GEM5_ROOT}/ext/googletest" "${CMAKE_CURRENT_BINARY_DIR}/googletest" EXCLUDE_FROM_ALL)

    # Helper function matching the SCons GTest() pattern.
    # Creates a test executable and registers it with CTest.
    function(m5_add_gtest TEST_NAME)
        set(_srcs ${ARGN})
        if(NOT _srcs)
            # Default: <name>.cc and <name>.test.cc
            set(_srcs "${M5_SRC_DIR}/${TEST_NAME}.cc"
                      "${M5_SRC_DIR}/${TEST_NAME}.test.cc")
        endif()
        set(_target "m5_test_${TEST_NAME}")
        # Sanitise target name (replace / with _)
        string(REPLACE "/" "_" _target "${_target}")
        add_executable(${_target} ${_srcs})
        target_include_directories(${_target} PRIVATE ${M5_SRC_DIR} ${M5_COMMON_INCLUDE})
        target_compile_definitions(${_target} PRIVATE ${M5_ABI_DEFINITIONS})
        if(M5_ABI_FLAGS)
            target_compile_options(${_target} PRIVATE ${M5_ABI_FLAGS})
        endif()
        target_link_libraries(${_target} PRIVATE gtest gmock gtest_main)
        add_test(NAME "m5.${TEST_NAME}" COMMAND ${_target})
    endfunction()

    # --- Native tests (from SConscript.native) ---
    # args test
    m5_add_gtest(args)

    # call_type test
    m5_add_gtest(call_type
        "${M5_SRC_DIR}/call_type.test.cc"
        "${M5_SRC_DIR}/call_type.cc"
        "${M5_SRC_DIR}/args.cc"
    )

    # command test
    m5_add_gtest(command
        "${M5_SRC_DIR}/command.test.cc"
        "${M5_SRC_DIR}/command.cc"
        "${M5_SRC_DIR}/args.cc"
    )

    # Individual command tests
    set(_command_test_names
        addsymbol checkpoint dumpresetstats dumpstats exit fail
        initparam loadsymbol readfile resetstats sum writefile
    )
    foreach(_cmd ${_command_test_names})
        m5_add_gtest("commands/${_cmd}"
            "${M5_SRC_DIR}/command/${_cmd}.test.cc"
            "${M5_SRC_DIR}/command/${_cmd}.cc"
            "${M5_SRC_DIR}/args.cc"
            "${M5_SRC_DIR}/command.cc"
        )
    endforeach()

    # --- ABI-specific call type tests (from main SConscript) ---
    # These test each enabled call type with its m5op implementation and
    # optional verifier. The call_type/<name>.cc file is compiled via its own
    # OBJECT library with CALL_TYPE_IS_DEFAULT=false (matching the SCons
    # allow_default=False behavior).
    foreach(_ct_name inst addr semi)
        string(TOUPPER "${_ct_name}" _ct_upper)
        if(M5_CALL_TYPE_${_ct_upper})
            # Object lib for call_type/<name>.cc with CALL_TYPE_IS_DEFAULT=false.
            set(_ct_obj_target "m5_ct_test_${_ct_name}")
            add_library(${_ct_obj_target} OBJECT
                "${M5_SRC_DIR}/call_type/${_ct_name}.cc")
            target_include_directories(${_ct_obj_target} PRIVATE
                ${M5_SRC_DIR} ${M5_COMMON_INCLUDE})
            target_compile_definitions(${_ct_obj_target} PRIVATE
                ${M5_ABI_DEFINITIONS} "CALL_TYPE_IS_DEFAULT=false")
            if(M5_ABI_FLAGS)
                target_compile_options(${_ct_obj_target} PRIVATE ${M5_ABI_FLAGS})
            endif()

            set(_ct_test_srcs
                "${M5_SRC_DIR}/call_type/${_ct_name}.test.cc"
                $<TARGET_OBJECTS:${_ct_obj_target}>
                "${M5_SRC_DIR}/call_type.cc"
                "${M5_SRC_DIR}/args.cc"
                "${M5_SRC_DIR}/m5_mmap.c"
            )
            # Add the m5op implementation for this call type.
            if(DEFINED M5_${_ct_upper}_IMPL)
                list(APPEND _ct_test_srcs "${M5_SRC_DIR}/${M5_${_ct_upper}_IMPL}")
            endif()
            # Add the verifier if it exists.
            if(DEFINED M5_${_ct_upper}_VERIFIER)
                list(APPEND _ct_test_srcs "${M5_SRC_DIR}/${M5_${_ct_upper}_VERIFIER}")
            endif()

            set(_ct_target "m5_test_call_type_${_ct_name}")
            add_executable(${_ct_target} ${_ct_test_srcs})
            target_include_directories(${_ct_target} PRIVATE
                ${M5_SRC_DIR} ${M5_COMMON_INCLUDE})
            target_compile_definitions(${_ct_target} PRIVATE ${M5_ABI_DEFINITIONS})
            if(M5_ABI_FLAGS)
                target_compile_options(${_ct_target} PRIVATE ${M5_ABI_FLAGS})
            endif()
            target_link_libraries(${_ct_target} PRIVATE gtest gmock gtest_main)
            add_test(NAME "m5.call_type.${_ct_name}" COMMAND ${_ct_target})
        endif()
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS m5 RUNTIME DESTINATION bin)
install(TARGETS m5_lib ARCHIVE DESTINATION lib)
