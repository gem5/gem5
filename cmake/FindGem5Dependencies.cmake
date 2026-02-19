# cmake/FindGem5Dependencies.cmake
# Dependency detection module for gem5, replicating the logic from
# site_scons/gem5_scons/configure.py and SConstruct.

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CMakePushCheckState)

# ---------------------------------------------------------------------------
# Required dependencies
# ---------------------------------------------------------------------------

# ZLIB
find_package(ZLIB REQUIRED)

# Python3 (interpreter for build scripts + development libs for embedding)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Python3 interpreter: ${Python3_EXECUTABLE} (${Python3_VERSION})")
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

# ---------------------------------------------------------------------------
# Optional dependencies
# ---------------------------------------------------------------------------

# Protobuf
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
    set(HAVE_PROTOBUF TRUE)
    message(STATUS "Protobuf found: ${Protobuf_VERSION}")
else()
    set(HAVE_PROTOBUF FALSE)
    message(STATUS "Protobuf not found -- proto trace support disabled")
endif()

# libpng
find_package(PNG QUIET)
if(PNG_FOUND)
    set(HAVE_PNG TRUE)
    message(STATUS "libpng found: ${PNG_VERSION_STRING}")
else()
    set(HAVE_PNG FALSE)
    message(STATUS "libpng not found -- VNC frame capture disabled")
endif()

# HDF5 (C++ bindings)
find_package(HDF5 QUIET COMPONENTS CXX)
if(HDF5_FOUND)
    set(HAVE_HDF5 TRUE)
    message(STATUS "HDF5 found: ${HDF5_VERSION}")
else()
    set(HAVE_HDF5 FALSE)
    message(STATUS "HDF5 not found -- HDF5 stats disabled")
endif()

# tcmalloc
option(GEM5_WITH_TCMALLOC "Link with tcmalloc for performance" ON)
set(HAVE_TCMALLOC FALSE)
if(GEM5_WITH_TCMALLOC)
    find_library(TCMALLOC_LIB NAMES tcmalloc_minimal tcmalloc)
    if(TCMALLOC_LIB)
        set(HAVE_TCMALLOC TRUE)
        message(STATUS "tcmalloc found: ${TCMALLOC_LIB}")
    else()
        message(STATUS "tcmalloc not found -- you can get a 12% performance "
                "improvement by installing tcmalloc "
                "(libgoogle-perftools-dev on Ubuntu)")
    endif()
endif()

# Capstone disassembly framework
find_library(CAPSTONE_LIB NAMES capstone)
find_path(CAPSTONE_INCLUDE_DIR NAMES capstone/capstone.h)
if(CAPSTONE_LIB AND CAPSTONE_INCLUDE_DIR)
    set(HAVE_CAPSTONE TRUE)
    message(STATUS "Capstone found: ${CAPSTONE_LIB}")
else()
    set(HAVE_CAPSTONE FALSE)
    message(STATUS "Capstone not found -- disassembly support disabled")
endif()

# ---------------------------------------------------------------------------
# System header checks
# ---------------------------------------------------------------------------

check_include_file("valgrind/valgrind.h" HAVE_VALGRIND)
if(HAVE_VALGRIND)
    message(STATUS "valgrind/valgrind.h found")
else()
    message(STATUS "valgrind/valgrind.h not found")
endif()

check_include_file("linux/kvm.h" HAVE_KVM)
if(HAVE_KVM)
    message(STATUS "linux/kvm.h found -- KVM support available")
else()
    message(STATUS "linux/kvm.h not found -- KVM support disabled")
endif()

# On x86_64, KVM requires struct kvm_xsave in kernel headers (matching SCons check)
if(HAVE_KVM AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    include(CheckTypeSize)
    cmake_push_check_state(RESET)
    set(CMAKE_EXTRA_INCLUDE_FILES "linux/kvm.h")
    check_type_size("struct kvm_xsave" KVM_XSAVE_SIZE LANGUAGE C)
    cmake_pop_check_state()
    if(NOT HAVE_KVM_XSAVE_SIZE)
        message(WARNING "KVM on x86 requires xsave support in kernel headers -- disabling KVM")
        set(HAVE_KVM FALSE)
    endif()
endif()

check_include_file("linux/if_tun.h" HAVE_TUNTAP)
if(HAVE_TUNTAP)
    message(STATUS "linux/if_tun.h found")
else()
    message(STATUS "linux/if_tun.h not found")
endif()

check_include_file("fenv.h" HAVE_FENV)

# execinfo.h (glibc backtrace support)
check_include_file("execinfo.h" HAVE_EXECINFO)
if(HAVE_EXECINFO)
    set(GEM5_BACKTRACE_IMPL "glibc")
else()
    set(GEM5_BACKTRACE_IMPL "none")
    message(STATUS "execinfo.h not found -- no backtrace support")
endif()

# librt (POSIX clock_gettime, needed on older glibc)
check_library_exists(rt clock_gettime "" HAVE_LIBRT)

# POSIX clock support: clock_gettime may be in libc or librt
include(CheckSymbolExists)
set(CMAKE_REQUIRED_LIBRARIES "")
if(HAVE_LIBRT)
    set(CMAKE_REQUIRED_LIBRARIES "rt")
endif()
check_symbol_exists(clock_gettime "time.h" HAVE_POSIX_CLOCK)
unset(CMAKE_REQUIRED_LIBRARIES)

# ---------------------------------------------------------------------------
# Linker selection
# ---------------------------------------------------------------------------

set(GEM5_LINKER "default" CACHE STRING "Linker to use (default/bfd/gold/lld/mold)")
set_property(CACHE GEM5_LINKER PROPERTY STRINGS default bfd gold lld mold)

if(NOT GEM5_LINKER STREQUAL "default")
    # Test that the chosen linker is actually usable
    include(CheckLinkerFlag)
    check_linker_flag(CXX "-fuse-ld=${GEM5_LINKER}" LINKER_${GEM5_LINKER}_WORKS)
    if(LINKER_${GEM5_LINKER}_WORKS)
        add_link_options("-fuse-ld=${GEM5_LINKER}")
        message(STATUS "Using linker: ${GEM5_LINKER}")
    else()
        message(FATAL_ERROR "Requested linker '${GEM5_LINKER}' is not supported by the toolchain")
    endif()
else()
    message(STATUS "Using default linker")
endif()
