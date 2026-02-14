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

# Determine the top-level Kconfig file.  If EXTRAS directories are specified,
# generate a base Kconfig that sources the main gem5 Kconfig plus any Kconfig
# files found in EXTRAS directories (matching the SCons kconfig_base.py logic).
if(GEM5_EXTRAS)
    set(_kconfig_file "${CMAKE_BINARY_DIR}/Kconfig.base")
    set(_base_kconfig "# Auto-generated base Kconfig -- DO NOT EDIT!\n")
    string(APPEND _base_kconfig "source \"${CMAKE_SOURCE_DIR}/src/Kconfig\"\n")
    foreach(_extras_dir IN LISTS GEM5_EXTRAS)
        get_filename_component(_extras_abs "${_extras_dir}" ABSOLUTE)
        string(APPEND _base_kconfig "osource \"${_extras_abs}/Kconfig\"\n")
    endforeach()
    file(WRITE "${_kconfig_file}" "${_base_kconfig}")
    message(STATUS "Generated base Kconfig with EXTRAS: ${_kconfig_file}")
else()
    set(_kconfig_file "${CMAKE_SOURCE_DIR}/src/Kconfig")
endif()

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

# HAVE_SYSTEMC: we have it in ext/systemc, but disable on RISC-V and macOS
# (matching SCons src/systemc/SConsopts behavior)
if(EXISTS "${CMAKE_SOURCE_DIR}/ext/systemc/src")
    set(_systemc_available TRUE)
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^riscv")
        message(WARNING "SystemC may not work on RISC-V -- disabling")
        set(_systemc_available FALSE)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        message(WARNING "SystemC may not work on macOS -- disabling")
        set(_systemc_available FALSE)
    endif()
    if(_systemc_available)
        list(APPEND _kconfig_defines "-D" "HAVE_SYSTEMC=y")
    else()
        list(APPEND _kconfig_defines "-D" "HAVE_SYSTEMC=n")
    endif()
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

# FastModel paths (read from environment, matching SCons SConsopts behavior)
if(DEFINED ENV{PVLIB_HOME})
    list(APPEND _kconfig_defines "-D" "PVLIB_HOME=$ENV{PVLIB_HOME}")
else()
    list(APPEND _kconfig_defines "-D" "PVLIB_HOME=")
endif()
if(DEFINED ENV{MAXCORE_HOME})
    list(APPEND _kconfig_defines "-D" "MAXCORE_HOME=$ENV{MAXCORE_HOME}")
else()
    list(APPEND _kconfig_defines "-D" "MAXCORE_HOME=")
endif()
if(DEFINED ENV{ARMLMD_LICENSE_FILE})
    list(APPEND _kconfig_defines "-D" "ARMLMD_LICENSE_FILE=$ENV{ARMLMD_LICENSE_FILE}")
else()
    list(APPEND _kconfig_defines "-D" "ARMLMD_LICENSE_FILE=")
endif()

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
            "--kconfig" "${_kconfig_file}"
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

# ---------------------------------------------------------------------------
# Kconfig interactive tool targets
# ---------------------------------------------------------------------------
# These targets mirror the SCons kconfig actions (menuconfig, defconfig,
# oldconfig, savedefconfig).  They invoke the kconfiglib scripts from
# ext/Kconfiglib/ with the same environment variables used at configure time.
#
# Usage:
#   cmake --build <build_dir> --target menuconfig
#   cmake --build <build_dir> --target oldconfig
#   cmake --build <build_dir> --target savedefconfig
#   cmake -DGEM5_DEFCONFIG_FILE=<path> <source_dir> && \
#       cmake --build <build_dir> --target defconfig

set(_kconfig_config "${CMAKE_BINARY_DIR}/.config")
set(_kconfiglib_dir "${CMAKE_SOURCE_DIR}/ext/Kconfiglib")

# Build environment variable list for kconfiglib tools by extracting
# KEY=VALUE pairs from _kconfig_defines (which has "-D" "KEY=VALUE" pairs).
set(_kconfig_env_vars "CONFIG_=")
foreach(_item IN LISTS _kconfig_defines)
    if(NOT _item STREQUAL "-D")
        list(APPEND _kconfig_env_vars "${_item}")
    endif()
endforeach()

# -- menuconfig: interactive curses-based configuration editor --
add_custom_target(menuconfig
    COMMAND "${CMAKE_COMMAND}" -E env
        ${_kconfig_env_vars}
        "KCONFIG_CONFIG=${_kconfig_config}"
        "MENUCONFIG_STYLE=aquatic"
        "PYTHONPATH=${CMAKE_SOURCE_DIR}/ext/Kconfiglib/import"
        "${Python3_EXECUTABLE}" "${_kconfiglib_dir}/menuconfig.py"
        "${_kconfig_file}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Running Kconfig menuconfig for ${GEM5_BUILD_VARIANT}..."
    USES_TERMINAL
)

# -- oldconfig: update .config, prompting for new symbols --
add_custom_target(oldconfig
    COMMAND "${CMAKE_COMMAND}" -E env
        ${_kconfig_env_vars}
        "KCONFIG_CONFIG=${_kconfig_config}"
        "PYTHONPATH=${CMAKE_SOURCE_DIR}/ext/Kconfiglib/import"
        "${Python3_EXECUTABLE}" "${_kconfiglib_dir}/oldconfig.py"
        "${_kconfig_file}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Running Kconfig oldconfig for ${GEM5_BUILD_VARIANT}..."
    USES_TERMINAL
)

# -- savedefconfig: save current .config as a minimal defconfig --
set(GEM5_SAVEDEFCONFIG_FILE "${CMAKE_BINARY_DIR}/defconfig" CACHE FILEPATH
    "Output path for the 'savedefconfig' target")

add_custom_target(savedefconfig
    COMMAND "${CMAKE_COMMAND}" -E env
        ${_kconfig_env_vars}
        "KCONFIG_CONFIG=${_kconfig_config}"
        "PYTHONPATH=${CMAKE_SOURCE_DIR}/ext/Kconfiglib/import"
        "${Python3_EXECUTABLE}" "${_kconfiglib_dir}/savedefconfig.py"
        "--kconfig" "${_kconfig_file}"
        "--out" "${GEM5_SAVEDEFCONFIG_FILE}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Saving defconfig to ${GEM5_SAVEDEFCONFIG_FILE}..."
    USES_TERMINAL
)

# -- defconfig: apply a defconfig file to create .config --
set(GEM5_DEFCONFIG_FILE "" CACHE FILEPATH
    "Input defconfig file for the 'defconfig' target (e.g. build_opts/ARM)")

add_custom_target(defconfig
    COMMAND "${CMAKE_COMMAND}" -E env
        ${_kconfig_env_vars}
        "KCONFIG_CONFIG=${_kconfig_config}"
        "PYTHONPATH=${CMAKE_SOURCE_DIR}/ext/Kconfiglib/import"
        "${Python3_EXECUTABLE}" "${_kconfiglib_dir}/defconfig.py"
        "--kconfig" "${_kconfig_file}"
        "${GEM5_DEFCONFIG_FILE}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Applying defconfig from ${GEM5_DEFCONFIG_FILE}..."
    USES_TERMINAL
)
