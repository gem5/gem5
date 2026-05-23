cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not allowed. "
        "Use 'cmake --preset <preset>' or set a separate build directory with -B.")
endif()

project(gem5 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# gem5_deps is the single INTERFACE target that every gem5 OBJECT library links
# against privately. All include paths, compile features, and per-build-type
# definitions propagate through this target — never via global directives such
# as include_directories() or add_compile_definitions(), which would pollute
# ext/ and any future sub-project.
#
# Defined here, before any include(), so Gem5Dependencies can call
# target_link_libraries(gem5_deps ...) during its own include pass.
add_library(gem5_deps INTERFACE)
target_include_directories(gem5_deps INTERFACE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR})            # placeholder for generated headers (PR 3)

# C++20 is applied per-target via the feature requirement, not via the global
# CMAKE_CXX_STANDARD, so ext/ libraries are not forced into C++20.
target_compile_features(gem5_deps INTERFACE cxx_std_20)

# Mirrors src/SConscript lines 679-681. Generator expressions are used instead
# of add_compile_definitions() to keep definitions scoped to targets that link
# gem5_deps, not to every target in the project.
target_compile_definitions(gem5_deps INTERFACE
    $<$<CONFIG:GEM5_DEBUG>:GEM5_DEBUG;TRACING_ON=1>
    $<$<CONFIG:GEM5_OPT>:TRACING_ON=1>
    $<$<CONFIG:GEM5_FAST>:NDEBUG;TRACING_ON=0>)

include(Gem5BuildTypes)        # validates CMAKE_BUILD_TYPE, sets CMAKE_CXX_FLAGS_GEM5_*
include(Gem5Dependencies)      # Python3, zlib (required); protobuf, png, HDF5 (optional)
include(Gem5Kconfig)           # validates GEM5_BUILD_VARIANT, parses build_opts/ -> CMake vars

include(Gem5Sources)           # gem5_add_source(), _gem5_ensure_object_target()
include(Gem5CodeGen)           # stub -- PR 3
include(Gem5Targets)           # stub -- PR 3
include(Gem5ConfigHeaders)     # stub -- PR 3
include(Gem5SwitchingHeaders)  # stub -- PR 3

# add_subdirectory(ext)        # PR 2
# add_subdirectory(src)        # PR 2