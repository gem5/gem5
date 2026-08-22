# FORCE is intentional: gem5's build type flags are non-negotiable and must
# not be weakened by a stale cache entry from a previous configure. Developers
# who need extra flags (e.g. -fsanitize=address, -march=native) should append
# them via CMAKE_CXX_FLAGS, which CMake layers on top of these per-type flags.
set(CMAKE_CXX_FLAGS_GEM5_DEBUG "-O0 -ggdb3"
    CACHE STRING "Flags for GEM5_DEBUG build" FORCE)
set(CMAKE_CXX_FLAGS_GEM5_OPT "-O3 -g"
    CACHE STRING "Flags for GEM5_OPT build" FORCE)
set(CMAKE_CXX_FLAGS_GEM5_FAST "-O3"
    CACHE STRING "Flags for GEM5_FAST build" FORCE)

set(CMAKE_C_FLAGS_GEM5_DEBUG "-O0 -ggdb3"
    CACHE STRING "Flags for GEM5_DEBUG C build" FORCE)
set(CMAKE_C_FLAGS_GEM5_OPT "-O3 -g"
    CACHE STRING "Flags for GEM5_OPT C build" FORCE)
set(CMAKE_C_FLAGS_GEM5_FAST "-O3"
    CACHE STRING "Flags for GEM5_FAST C build" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "GEM5_OPT" CACHE STRING "gem5 build type" FORCE)
    message(STATUS "CMAKE_BUILD_TYPE not set; defaulting to GEM5_OPT")
endif()

set(_gem5_valid_build_types GEM5_DEBUG GEM5_OPT GEM5_FAST)
if(NOT CMAKE_BUILD_TYPE IN_LIST _gem5_valid_build_types)
    message(FATAL_ERROR
        "Invalid CMAKE_BUILD_TYPE '${CMAKE_BUILD_TYPE}'. "
        "Valid values: ${_gem5_valid_build_types}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "GEM5_DEBUG")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Debug: -O0 -ggdb3, GEM5_DEBUG, TRACING_ON=1")
elseif(CMAKE_BUILD_TYPE STREQUAL "GEM5_OPT")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Optimized: -O3, debug info, TRACING_ON=1")
elseif(CMAKE_BUILD_TYPE STREQUAL "GEM5_FAST")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Fast: -O3, NDEBUG, TRACING_ON=0")
endif()

message(STATUS "gem5 build type: ${GEM5_BUILD_TYPE_DESCRIPTION}")
