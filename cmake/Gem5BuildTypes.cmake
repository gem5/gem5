# cmake/Gem5BuildTypes.cmake
# Custom build type definitions for gem5 (debug, opt, fast)

# Define the set of valid build types
set(GEM5_VALID_BUILD_TYPES "GEM5_DEBUG" "GEM5_OPT" "GEM5_FAST")

# Default to GEM5_OPT if no build type specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "GEM5_OPT" CACHE STRING
        "Build type: GEM5_DEBUG, GEM5_OPT, or GEM5_FAST" FORCE)
endif()

# Validate the build type
string(TOUPPER "${CMAKE_BUILD_TYPE}" _build_type_upper)
if(NOT _build_type_upper IN_LIST GEM5_VALID_BUILD_TYPES)
    message(FATAL_ERROR
        "Invalid CMAKE_BUILD_TYPE '${CMAKE_BUILD_TYPE}'.\n"
        "Valid options: ${GEM5_VALID_BUILD_TYPES}")
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${GEM5_VALID_BUILD_TYPES})

# --- GEM5_DEBUG: -O0 -ggdb3, GEM5_DEBUG defined, TRACING_ON=1 ---
set(CMAKE_C_FLAGS_GEM5_DEBUG          "-O0 -ggdb3" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_GEM5_DEBUG        "-O0 -ggdb3" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_GEM5_DEBUG "" CACHE STRING "" FORCE)

# --- GEM5_OPT: -O3 -g, TRACING_ON=1 ---
set(CMAKE_C_FLAGS_GEM5_OPT          "-O3 -g" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_GEM5_OPT        "-O3 -g" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_GEM5_OPT "" CACHE STRING "" FORCE)

# --- GEM5_FAST: -O3, NDEBUG defined, TRACING_ON=0 ---
set(CMAKE_C_FLAGS_GEM5_FAST          "-O3" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_GEM5_FAST        "-O3" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_GEM5_FAST "" CACHE STRING "" FORCE)

# Apply compile definitions based on build type
if(_build_type_upper STREQUAL "GEM5_DEBUG")
    add_compile_definitions(GEM5_DEBUG TRACING_ON=1)
elseif(_build_type_upper STREQUAL "GEM5_OPT")
    add_compile_definitions(TRACING_ON=1)
elseif(_build_type_upper STREQUAL "GEM5_FAST")
    add_compile_definitions(NDEBUG TRACING_ON=0)
endif()

# Make build type visible for status messages
set(GEM5_BUILD_TYPE_DESCRIPTION "")
if(_build_type_upper STREQUAL "GEM5_DEBUG")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Debug (-O0, full debug info, tracing enabled)")
elseif(_build_type_upper STREQUAL "GEM5_OPT")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Optimized (-O3, debug info, tracing enabled)")
elseif(_build_type_upper STREQUAL "GEM5_FAST")
    set(GEM5_BUILD_TYPE_DESCRIPTION "Fast (-O3, no debug/tracing, NDEBUG)")
endif()
