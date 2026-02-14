# cmake/Gem5Kconfig.cmake
# Bridge between gem5's Kconfig-based build_opts/ and CMake variables.
#
# Usage:
#   -DGEM5_BUILD_VARIANT=X86   (default)
#   -DGEM5_BUILD_VARIANT=ALL
#   -DGEM5_BUILD_VARIANT=ARM
#   etc.
#
# This module runs kconfig_to_cmake.py at configure time to process
# build_opts/<variant> through src/Kconfig using ext/Kconfiglib,
# producing a .cmake file with CONF_* variables.
#
# The Kconfig files use $(VARIABLE) macros that must be resolved from
# environment variables.  We pass the results of CMake's dependency
# detection as -D KEY=VALUE arguments to the script.

set(GEM5_BUILD_VARIANT "X86" CACHE STRING
    "Build variant (corresponds to a file in build_opts/)")

# Validate that the variant file exists
set(_variant_file "${CMAKE_SOURCE_DIR}/build_opts/${GEM5_BUILD_VARIANT}")
if(NOT EXISTS "${_variant_file}")
    # List available variants for a helpful error message
    file(GLOB _available_variants
         RELATIVE "${CMAKE_SOURCE_DIR}/build_opts"
         "${CMAKE_SOURCE_DIR}/build_opts/*")
    list(SORT _available_variants)
    string(REPLACE ";" ", " _variants_str "${_available_variants}")
    message(FATAL_ERROR
        "Build variant '${GEM5_BUILD_VARIANT}' not found.\n"
        "No file at: ${_variant_file}\n"
        "Available variants: ${_variants_str}")
endif()

# Output file for generated CMake variables
set(_kconfig_output "${CMAKE_BINARY_DIR}/gem5_kconfig.cmake")

# Build the list of -D arguments to pass dependency detection results
# to the Python script.  Kconfig files use $(VAR) macros that resolve
# from environment variables; the script sets these before loading Kconfig.

# Helper: convert CMake bool/truthy to "y"/"n" for Kconfig
macro(_kconfig_bool_arg varname cmake_var)
    if(${cmake_var})
        list(APPEND _kconfig_defines "-D" "${varname}=y")
    else()
        list(APPEND _kconfig_defines "-D" "${varname}=n")
    endif()
endmacro()

set(_kconfig_defines "")

# -- Dependency detection results from FindGem5Dependencies.cmake --
_kconfig_bool_arg(HAVE_FENV       HAVE_FENV)
_kconfig_bool_arg(HAVE_PNG        HAVE_PNG)
_kconfig_bool_arg(HAVE_VALGRIND   HAVE_VALGRIND)
_kconfig_bool_arg(HAVE_POSIX_CLOCK HAVE_LIBRT)
_kconfig_bool_arg(HAVE_HDF5       HAVE_HDF5)
_kconfig_bool_arg(HAVE_PROTOBUF   HAVE_PROTOBUF)
_kconfig_bool_arg(HAVE_TUNTAP     HAVE_TUNTAP)
_kconfig_bool_arg(HAVE_CAPSTONE   HAVE_CAPSTONE)
_kconfig_bool_arg(HAVE_KVM        HAVE_KVM)

# HAVE_DEPRECATED_NAMESPACE: check if compiler supports [[gnu::deprecated]] on namespaces
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-Werror")
check_cxx_source_compiles("
    int main() { return 0; }
    namespace [[gnu::deprecated(\"test\")]] test_ns {}
" HAVE_DEPRECATED_NAMESPACE)
unset(CMAKE_REQUIRED_FLAGS)
_kconfig_bool_arg(HAVE_DEPRECATED_NAMESPACE HAVE_DEPRECATED_NAMESPACE)

# HAVE_SYSTEMC: we have it in ext/systemc
if(EXISTS "${CMAKE_SOURCE_DIR}/ext/systemc/src")
    list(APPEND _kconfig_defines "-D" "HAVE_SYSTEMC=y")
else()
    list(APPEND _kconfig_defines "-D" "HAVE_SYSTEMC=n")
endif()

# KVM_ISA: determine from host architecture
if(HAVE_KVM)
    if(GEM5_BIN_TARGET_ARCH STREQUAL "x86_64")
        list(APPEND _kconfig_defines "-D" "KVM_ISA=x86")
    elseif(GEM5_BIN_TARGET_ARCH STREQUAL "aarch64")
        list(APPEND _kconfig_defines "-D" "KVM_ISA=arm")
    else()
        list(APPEND _kconfig_defines "-D" "KVM_ISA=")
    endif()
else()
    list(APPEND _kconfig_defines "-D" "KVM_ISA=")
endif()

# MAIN_MENU_TEXT: used by the top-level mainmenu directive
list(APPEND _kconfig_defines "-D" "MAIN_MENU_TEXT=gem5 ${GEM5_BUILD_VARIANT}")

# FastModel paths (optional, usually empty)
list(APPEND _kconfig_defines "-D" "PVLIB_HOME=")
list(APPEND _kconfig_defines "-D" "MAXCORE_HOME=")
list(APPEND _kconfig_defines "-D" "ARMLMD_LICENSE_FILE=")

# HAVE_PERF_ATTR_EXCLUDE_HOST: check for the perf attribute
if(HAVE_KVM)
    include(CheckCXXSourceCompiles)
    check_cxx_source_compiles("
        #include <linux/perf_event.h>
        int main() {
            struct perf_event_attr attr;
            (void)attr.exclude_host;
            return 0;
        }
    " HAVE_PERF_ATTR_EXCLUDE_HOST)
    _kconfig_bool_arg(HAVE_PERF_ATTR_EXCLUDE_HOST HAVE_PERF_ATTR_EXCLUDE_HOST)
else()
    list(APPEND _kconfig_defines "-D" "HAVE_PERF_ATTR_EXCLUDE_HOST=n")
endif()

# Run the Python script to process Kconfig
message(STATUS "Processing Kconfig for variant: ${GEM5_BUILD_VARIANT}")
execute_process(
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/cmake/kconfig_to_cmake.py"
            "--kconfig" "${CMAKE_SOURCE_DIR}/src/Kconfig"
            "--defconfig" "${_variant_file}"
            "--output" "${_kconfig_output}"
            ${_kconfig_defines}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE _kconfig_result
    OUTPUT_VARIABLE _kconfig_stdout
    ERROR_VARIABLE  _kconfig_stderr
)

if(NOT _kconfig_result EQUAL 0)
    message(FATAL_ERROR
        "Kconfig processing failed for variant '${GEM5_BUILD_VARIANT}':\n"
        "${_kconfig_stderr}")
endif()

if(_kconfig_stdout)
    message(STATUS "${_kconfig_stdout}")
endif()

# Include the generated variables
include("${_kconfig_output}")

# Print a summary of key Kconfig-derived variables
message(STATUS "Kconfig: CONF_BUILD_ISA = ${CONF_BUILD_ISA}")
message(STATUS "Kconfig: CONF_USE_X86_ISA = ${CONF_USE_X86_ISA}")
message(STATUS "Kconfig: CONF_USE_ARM_ISA = ${CONF_USE_ARM_ISA}")
message(STATUS "Kconfig: CONF_USE_RISCV_ISA = ${CONF_USE_RISCV_ISA}")
message(STATUS "Kconfig: CONF_USE_SPARC_ISA = ${CONF_USE_SPARC_ISA}")
message(STATUS "Kconfig: CONF_USE_MIPS_ISA = ${CONF_USE_MIPS_ISA}")
message(STATUS "Kconfig: CONF_USE_POWER_ISA = ${CONF_USE_POWER_ISA}")
message(STATUS "Kconfig: CONF_RUBY = ${CONF_RUBY}")
message(STATUS "Kconfig: CONF_PROTOCOL = ${CONF_PROTOCOL}")
message(STATUS "Kconfig: CONF_BUILD_GPU = ${CONF_BUILD_GPU}")
