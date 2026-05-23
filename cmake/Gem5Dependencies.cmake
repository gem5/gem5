include(CheckIncludeFile)
include(CheckSourceCompiles)
include(CMakePushCheckState)

# --- Required ---

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ZLIB REQUIRED)

target_link_libraries(gem5_deps INTERFACE
    Python3::Python
    ZLIB::ZLIB)

# --- Optional ---

# Protobuf ships a cmake config on modern installs (protobuf >= 3.6) but older
# distro packages only provide a FindProtobuf module. Try CONFIG first so the
# imported target names are consistent; fall back to MODULE for legacy installs.
find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
    find_package(Protobuf QUIET)
endif()
if(Protobuf_FOUND)
    set(GEM5_HAS_PROTOBUF TRUE)
    target_link_libraries(gem5_deps INTERFACE protobuf::libprotobuf)
else()
    set(GEM5_HAS_PROTOBUF FALSE)
endif()

# PNG
find_package(PNG QUIET)
if(PNG_FOUND)
    set(GEM5_HAS_PNG TRUE)
    target_link_libraries(gem5_deps INTERFACE PNG::PNG)
else()
    set(GEM5_HAS_PNG FALSE)
endif()

# HDF5
find_package(HDF5 QUIET COMPONENTS CXX)
if(HDF5_FOUND)
    set(GEM5_HAS_HDF5 TRUE)
    target_link_libraries(gem5_deps INTERFACE hdf5::hdf5_cpp)
else()
    set(GEM5_HAS_HDF5 FALSE)
endif()

# tcmalloc is opt-in (unlike the other optional deps which are auto-detected)
# because linking it replaces the system allocator globally, which can mask
# bugs and cause surprises in debug builds.
option(GEM5_WITH_TCMALLOC "Link against tcmalloc" OFF)
if(GEM5_WITH_TCMALLOC)
    find_library(_tcmalloc_path NAMES tcmalloc tcmalloc_minimal)
    if(_tcmalloc_path)
        set(GEM5_HAS_TCMALLOC TRUE)
        add_library(tcmalloc::tcmalloc UNKNOWN IMPORTED)
        set_target_properties(tcmalloc::tcmalloc PROPERTIES
            IMPORTED_LOCATION "${_tcmalloc_path}")
        target_link_libraries(gem5_deps INTERFACE tcmalloc::tcmalloc)
    else()
        message(FATAL_ERROR "GEM5_WITH_TCMALLOC=ON but tcmalloc library not found")
    endif()
else()
    set(GEM5_HAS_TCMALLOC FALSE)
endif()

# Capstone does not provide a cmake config on most distros, so find_package
# often fails even when the library is present. Fall back to a raw find_library
# so that system installs without a cmake config are still detected.
find_package(capstone QUIET)
if(capstone_FOUND)
    set(GEM5_HAS_CAPSTONE TRUE)
    target_link_libraries(gem5_deps INTERFACE capstone::capstone)
else()
    find_library(_capstone_path NAMES capstone)
    if(_capstone_path)
        set(GEM5_HAS_CAPSTONE TRUE)
        add_library(capstone::capstone UNKNOWN IMPORTED)
        set_target_properties(capstone::capstone PROPERTIES
            IMPORTED_LOCATION "${_capstone_path}")
        target_link_libraries(gem5_deps INTERFACE capstone::capstone)
    else()
        set(GEM5_HAS_CAPSTONE FALSE)
    endif()
endif()

# --- System header checks ---

# check_include_file sets its result variable to "1" on success and leaves it
# empty (not FALSE) on failure. Use private _GEM5_HAS_* names here, then
# normalize to TRUE/FALSE below so downstream consumers get consistent booleans.
check_include_file("valgrind/valgrind.h" _GEM5_HAS_VALGRIND)
check_include_file("linux/kvm.h"         _GEM5_HAS_KVM)
check_include_file("linux/if_tun.h"      _GEM5_HAS_IF_TUN)
check_include_file("fenv.h"              _GEM5_HAS_FENV)
check_include_file("execinfo.h"          _GEM5_HAS_EXECINFO)

foreach(_hdr VALGRIND KVM IF_TUN FENV EXECINFO)
    if(_GEM5_HAS_${_hdr})
        set(GEM5_HAS_${_hdr} TRUE)
    else()
        set(GEM5_HAS_${_hdr} FALSE)
    endif()
endforeach()

# kvm_xsave was added in Linux 3.12. Kernels that have linux/kvm.h but predate
# that struct would compile gem5 without KVM xsave support even though the
# header is present, so we need an explicit struct-existence check.
if(_GEM5_HAS_KVM AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    cmake_push_check_state(RESET)
    set(CMAKE_REQUIRED_FLAGS "-include linux/kvm.h")
    check_source_compiles(CXX
        "int main() { struct kvm_xsave s; (void)s; return 0; }"
        GEM5_KVM_XSAVE_OK)
    cmake_pop_check_state()
endif()

# --- Linker selection ---

set(GEM5_LINKER "" CACHE STRING "Linker to use: bfd, gold, lld, mold, or empty for default")
if(GEM5_LINKER)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-fuse-ld=${GEM5_LINKER}" _linker_flag_ok)
    if(_linker_flag_ok)
        target_link_options(gem5_deps INTERFACE "-fuse-ld=${GEM5_LINKER}")
    else()
        message(FATAL_ERROR "Linker '${GEM5_LINKER}' not supported by the compiler")
    endif()
endif()

# --- Dependency summary ---

message(STATUS "gem5 dependency summary:")
message(STATUS "  Python3     : ${Python3_VERSION} (${Python3_EXECUTABLE})")
message(STATUS "  ZLIB        : ${ZLIB_VERSION_STRING}")
message(STATUS "  Protobuf    : ${GEM5_HAS_PROTOBUF}")
message(STATUS "  PNG         : ${GEM5_HAS_PNG}")
message(STATUS "  HDF5        : ${GEM5_HAS_HDF5}")
message(STATUS "  tcmalloc    : ${GEM5_HAS_TCMALLOC}")
message(STATUS "  Capstone    : ${GEM5_HAS_CAPSTONE}")
message(STATUS "  valgrind.h  : ${GEM5_HAS_VALGRIND}")
message(STATUS "  linux/kvm.h : ${GEM5_HAS_KVM}")
message(STATUS "  if_tun.h    : ${GEM5_HAS_IF_TUN}")
message(STATUS "  fenv.h      : ${GEM5_HAS_FENV}")
message(STATUS "  execinfo.h  : ${GEM5_HAS_EXECINFO}")
