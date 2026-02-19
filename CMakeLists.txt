cmake_minimum_required(VERSION 3.20)
project(gem5 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Generate compile_commands.json for IDE/editor integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add cmake/ to the module search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------------------
# Build type (debug / opt / fast)
# ---------------------------------------------------------------------------
include(Gem5BuildTypes)

# ---------------------------------------------------------------------------
# Compiler detection and version checks
# ---------------------------------------------------------------------------
set(GEM5_GCC FALSE)
set(GEM5_CLANG FALSE)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(GEM5_GCC TRUE)
    set(_gcc_min_version "11")
    set(_gcc_max_version "14.2")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS _gcc_min_version OR
       CMAKE_CXX_COMPILER_VERSION VERSION_GREATER _gcc_max_version)
        message(WARNING
            "Detected GCC version ${CMAKE_CXX_COMPILER_VERSION} is not "
            "officially supported.\n"
            "gem5 supports GCC v${_gcc_min_version} up to v${_gcc_max_version}.")
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(GEM5_CLANG TRUE)
    set(_clang_min_version "14")
    set(_clang_max_version "19")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS _clang_min_version OR
       CMAKE_CXX_COMPILER_VERSION VERSION_GREATER _clang_max_version)
        message(WARNING
            "Detected Clang version ${CMAKE_CXX_COMPILER_VERSION} is not "
            "officially supported.\n"
            "gem5 supports Clang v${_clang_min_version} up to "
            "v${_clang_max_version}.")
    endif()
else()
    message(FATAL_ERROR
        "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}\n"
        "gem5 requires GCC or Clang.")
endif()

# Detect host target architecture
execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} --verbose
    OUTPUT_VARIABLE _compiler_verbose
    ERROR_VARIABLE  _compiler_verbose
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(_compiler_verbose MATCHES "Target: x86_64")
    set(GEM5_BIN_TARGET_ARCH "x86_64")
elseif(_compiler_verbose MATCHES "Target: aarch64")
    set(GEM5_BIN_TARGET_ARCH "aarch64")
else()
    set(GEM5_BIN_TARGET_ARCH "unknown")
endif()

# ---------------------------------------------------------------------------
# Global compiler flags (matching SCons: shared GCC/Clang flags)
# ---------------------------------------------------------------------------
add_compile_options(
    -pipe
    -fno-strict-aliasing
    -Wall
    -Wundef
    -Wextra
    -Wno-sign-compare
    -Wno-unused-parameter
)

# Treat warnings as errors for gem5's own code only (not ext/ third-party).
# Applied to gem5 targets in Gem5Targets.cmake via target_compile_options().
set(GEM5_WERROR_FLAGS
    -Werror
    -Wno-error=deprecated-declarations
    -Wno-error=deprecated
    CACHE INTERNAL "Warning-as-error flags for gem5 targets"
)
# ISA-parser-generated code contains self-assignments (e.g. resTemp = resTemp)
# that Clang warns about. Whitelist these since the generated code cannot be
# changed.
if(GEM5_CLANG)
    list(APPEND GEM5_WERROR_FLAGS
        -Wno-error=self-assign
        -Wno-error=self-assign-overloaded
        -Wno-error=parentheses-equality
    )
endif()

# Link flag: --as-needed (GNU ld only; not accepted by macOS ld64)
include(CheckLinkerFlag)
check_linker_flag(CXX "-Wl,--as-needed" _has_as_needed)
if(_has_as_needed)
    add_link_options(-Wl,--as-needed)
endif()

# Clang-specific warnings to disable
if(GEM5_CLANG)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-Wno-c99-designator _has_c99_designator)
    if(_has_c99_designator)
        add_compile_options(-Wno-c99-designator)
    endif()
    check_cxx_compiler_flag(-Wno-defaulted-function-deleted _has_defaulted_func_deleted)
    if(_has_defaulted_func_deleted)
        add_compile_options(-Wno-defaulted-function-deleted)
    endif()
endif()

# FreeBSD needs extra include paths and libthr
if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    add_compile_options(-I/usr/local/include)
    link_libraries(thr)
endif()

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------

# LTO
option(GEM5_WITH_LTO "Enable Link-Time Optimization" OFF)
if(GEM5_WITH_LTO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    message(STATUS "Link-Time Optimization: enabled")
endif()

# Sanitizers
option(GEM5_WITH_ASAN "Enable AddressSanitizer" OFF)
option(GEM5_WITH_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
if(GEM5_WITH_ASAN OR GEM5_WITH_UBSAN)
    set(_sanitizers "")
    if(GEM5_WITH_UBSAN)
        list(APPEND _sanitizers "undefined")
    endif()
    if(GEM5_WITH_ASAN)
        list(APPEND _sanitizers "address")
    endif()
    list(JOIN _sanitizers "," _sanitizer_flags)

    add_compile_options(-fsanitize=${_sanitizer_flags} -fno-omit-frame-pointer)
    add_link_options(-fsanitize=${_sanitizer_flags})

    if(GEM5_GCC)
        add_link_options(-static-libubsan -static-libasan)
    elseif(GEM5_CLANG)
        add_link_options(-static-libsan)
    endif()

    # Use medium code model on x86_64 to handle enlarged binary sizes
    if(GEM5_BIN_TARGET_ARCH STREQUAL "x86_64")
        add_compile_options(-mcmodel=medium)
        add_link_options(-mcmodel=medium)
    endif()

    message(STATUS "Sanitizers enabled: ${_sanitizer_flags}")
endif()

# Profiler support
option(GEM5_WITH_GPROF "Enable gprof profiler support" OFF)
option(GEM5_WITH_PPROF "Enable pprof profiler support" OFF)
if(GEM5_WITH_GPROF AND GEM5_WITH_PPROF)
    message(FATAL_ERROR "Only one profiler (gprof or pprof) can be enabled at a time")
endif()
if(GEM5_WITH_GPROF)
    add_compile_options(-g -pg)
    add_link_options(-pg)
endif()
if(GEM5_WITH_PPROF)
    add_compile_options(-g)
    if(_has_as_needed)
        add_link_options(-Wl,--no-as-needed -lprofiler -Wl,--as-needed)
    else()
        add_link_options(-lprofiler)
    endif()
endif()

# Debug compression and fission
option(GEM5_NO_COMPRESS_DEBUG "Disable debug section compression" OFF)
option(GEM5_DEBUG_FISSION "Enable debug fission (-gsplit-dwarf)" OFF)
if(NOT GEM5_NO_COMPRESS_DEBUG)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-gz _has_gz)
    if(_has_gz)
        add_compile_options(-gz)
        add_link_options(-gz)
    endif()
endif()
if(GEM5_DEBUG_FISSION)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-gsplit-dwarf _has_split_dwarf)
    if(_has_split_dwarf)
        add_compile_options(-gsplit-dwarf)
        add_link_options(-gsplit-dwarf)
    else()
        message(FATAL_ERROR "Debug fission (-gsplit-dwarf) not supported by toolchain")
    endif()
endif()

# Python embedding
option(GEM5_WITHOUT_PYTHON "Build without Python configuration support" OFF)

# C++ config
option(GEM5_WITH_CXX_CONFIG "Build with support for C++-based configuration" OFF)

# CHI-TLM integration (requires external libarmtlmchi.so)
set(GEM5_TLM_PATH "" CACHE PATH "Path to the AMBA CHI TLM library (libarmtlmchi.so)")
# Auto-detect from environment variable if not set via CMake option
if(NOT GEM5_TLM_PATH OR GEM5_TLM_PATH STREQUAL "")
    if(DEFINED ENV{TLM_PATH} AND NOT "$ENV{TLM_PATH}" STREQUAL "" AND NOT "$ENV{TLM_PATH}" STREQUAL ".")
        set(GEM5_TLM_PATH "$ENV{TLM_PATH}" CACHE PATH "" FORCE)
        message(STATUS "Setting TLM_PATH from environment: ${GEM5_TLM_PATH}")
    endif()
endif()
if(GEM5_TLM_PATH AND NOT GEM5_TLM_PATH STREQUAL "" AND NOT GEM5_TLM_PATH STREQUAL ".")
    set(CONF_BUILD_TLM TRUE)
else()
    set(CONF_BUILD_TLM FALSE)
endif()

# ---------------------------------------------------------------------------
# EXTRAS: external source directories injected into the build
# ---------------------------------------------------------------------------
# Equivalent of SCons EXTRAS variable.  Pass a semicolon-separated list of
# absolute paths to external directories that should be compiled into gem5.
# Each directory may contain a CMakeLists.txt (included via add_subdirectory)
# and/or a Kconfig (sourced into the Kconfig tree).
#
# Example:
#   cmake -DGEM5_EXTRAS="/path/to/ext1;/path/to/ext2" ..
set(GEM5_EXTRAS "" CACHE STRING
    "Semicolon-separated list of external source directories to include in the build")

# Validate EXTRAS paths early
if(GEM5_EXTRAS)
    foreach(_extras_dir IN LISTS GEM5_EXTRAS)
        get_filename_component(_extras_abs "${_extras_dir}" ABSOLUTE)
        if(NOT IS_DIRECTORY "${_extras_abs}")
            message(FATAL_ERROR
                "GEM5_EXTRAS directory does not exist: ${_extras_abs}")
        endif()
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Dependency detection
# ---------------------------------------------------------------------------
include(FindGem5Dependencies)

# ---------------------------------------------------------------------------
# Kconfig / build variant processing
# ---------------------------------------------------------------------------
include(Gem5Kconfig)

# ---------------------------------------------------------------------------
# Source registration and code generation utilities
# ---------------------------------------------------------------------------
include(Gem5Sources)
include(Gem5CodeGen)
include(Gem5Py)

# ---------------------------------------------------------------------------
# Config headers and switching headers (must come after Kconfig)
# ---------------------------------------------------------------------------
include(Gem5ConfigHeaders)
include(Gem5SwitchingHeaders)

# Generated sources include path (for debug/*.hh, params/*.hh, enums/*.hh, config/*.hh)
include_directories("${GEM5_GEN_DIR}")

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
# ext/ directory for third-party headers
include_directories("${CMAKE_SOURCE_DIR}/ext")
# Shared top-level headers
include_directories("${CMAKE_SOURCE_DIR}/include")
# Source tree (for non-duplicated source builds)
include_directories("${CMAKE_SOURCE_DIR}/src")

# SystemC/TLM include paths (matching SCons: systemc_home/include and ext/)
if(CONF_USE_SYSTEMC)
    include_directories("${CMAKE_SOURCE_DIR}/src/systemc/ext/systemc_home/include")
    include_directories("${CMAKE_SOURCE_DIR}/src/systemc/ext")
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(ext)

# ---------------------------------------------------------------------------
# Build-time helper executables (requires pybind11 from ext/)
# ---------------------------------------------------------------------------
# gem5py: simple Python interpreter for build scripts
gem5_build_gem5py()
# gem5py_m5: full interpreter with embedded m5 modules -- built later
# after all gem5_add_pysource() registrations are complete.
# Call gem5_build_gem5py_m5() at the end of source processing.

add_subdirectory(src)

# ---------------------------------------------------------------------------
# EXTRAS source directories
# ---------------------------------------------------------------------------
# Include external source directories specified via GEM5_EXTRAS.
# Each directory with a CMakeLists.txt is added as a subdirectory, allowing
# external projects to register sources, SimObjects, etc. using the same
# gem5_add_source() / gem5_add_simobject() API as in-tree code.
if(GEM5_EXTRAS)
    foreach(_extras_dir IN LISTS GEM5_EXTRAS)
        get_filename_component(_extras_abs "${_extras_dir}" ABSOLUTE)
        get_filename_component(_extras_name "${_extras_abs}" NAME)
        if(EXISTS "${_extras_abs}/CMakeLists.txt")
            message(STATUS "Including EXTRAS directory: ${_extras_abs}")
            add_subdirectory("${_extras_abs}"
                "${CMAKE_BINARY_DIR}/extras/${_extras_name}")
        else()
            message(STATUS "EXTRAS directory has no CMakeLists.txt, skipping: ${_extras_abs}")
        endif()
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Generate m5/defines.py and m5/info.py (must be after Kconfig and before gem5py_m5)
# ---------------------------------------------------------------------------
gem5_generate_defines_py()
gem5_generate_info_py()

# ---------------------------------------------------------------------------
# Create PySource custom commands at the top level
# ---------------------------------------------------------------------------
# Must be called after all gem5_add_pysource() registrations (from
# add_subdirectory calls) and before gem5_build_gem5py_m5().
# Custom commands must be in the same directory scope as the targets that
# consume their outputs (gem5py_m5, gem5_pysources) to avoid Ninja stub issues.
gem5_create_pysource_commands()

# ---------------------------------------------------------------------------
# Build gem5py_m5 after all Python source registrations
# ---------------------------------------------------------------------------
# gem5py_m5 links all marshalled Python .py.cc files + embedded infrastructure.
# It must be built after all add_subdirectory() calls that register PySource files.
gem5_build_gem5py_m5()

# ---------------------------------------------------------------------------
# Create SimObject param/enum custom commands at the top level
# ---------------------------------------------------------------------------
# Must be called after gem5_build_gem5py_m5() since these commands use gem5py_m5.
# Same Ninja cross-directory workaround as PySource commands above.
gem5_create_simobject_commands()

# ---------------------------------------------------------------------------
# Create protobuf custom commands at the top level
# ---------------------------------------------------------------------------
# Must be called after add_subdirectory(src) where .proto files are registered.
# Same Ninja cross-directory workaround as PySource commands above.
gem5_create_proto_commands()

# ---------------------------------------------------------------------------
# Final link targets (gem5 executable, libraries, unit tests)
# ---------------------------------------------------------------------------
include(Gem5Targets)

# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "========== gem5 CMake Build Configuration ==========")
message(STATUS "  Build variant:     ${GEM5_BUILD_VARIANT}")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE} -- ${GEM5_BUILD_TYPE_DESCRIPTION}")
message(STATUS "  Compiler:          ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Host arch:         ${GEM5_BIN_TARGET_ARCH}")
message(STATUS "  Linker:            ${GEM5_LINKER}")
message(STATUS "  Python:            ${Python3_VERSION} (${Python3_EXECUTABLE})")
message(STATUS "  ZLIB:              ${ZLIB_VERSION_STRING}")
message(STATUS "  Protobuf:          ${HAVE_PROTOBUF}")
message(STATUS "  libpng:            ${HAVE_PNG}")
message(STATUS "  HDF5:              ${HAVE_HDF5}")
message(STATUS "  tcmalloc:          ${HAVE_TCMALLOC}")
message(STATUS "  Capstone:          ${HAVE_CAPSTONE}")
message(STATUS "  KVM:               ${HAVE_KVM}")
message(STATUS "  Valgrind:          ${HAVE_VALGRIND}")
message(STATUS "  LTO:               ${GEM5_WITH_LTO}")
message(STATUS "  ASAN:              ${GEM5_WITH_ASAN}")
message(STATUS "  UBSAN:             ${GEM5_WITH_UBSAN}")
message(STATUS "  Without Python:    ${GEM5_WITHOUT_PYTHON}")
message(STATUS "  C++ Config:        ${GEM5_WITH_CXX_CONFIG}")
message(STATUS "  CHI-TLM:           ${CONF_BUILD_TLM}")
if(GEM5_EXTRAS)
    message(STATUS "  EXTRAS:            ${GEM5_EXTRAS}")
endif()
message(STATUS "====================================================")
message(STATUS "")
