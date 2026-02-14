# ext/CMakeLists.txt
# Build all external (bundled) libraries for gem5.
#
# Libraries are split into three groups:
#   1. Always-built libraries (sources ship with gem5)
#   2. Complex always-built libraries (M4 codegen, arch detection, etc.)
#   3. Conditional libraries (require cloned submodules / subdirectories)

find_package(Threads REQUIRED)

# ── 1. Simple always-built libraries ─────────────────────────────────────────
add_subdirectory(fputils)
add_subdirectory(libfdt)
add_subdirectory(nomali)
add_subdirectory(drampower)
add_subdirectory(iostream3)
add_subdirectory(magic_enum)

# ── 2. Complex always-built libraries ────────────────────────────────────────
add_subdirectory(libelf)
add_subdirectory(softfloat)
add_subdirectory(systemc)

# googletest ships its own CMakeLists.txt which we do NOT want to use
# (it pulls in install targets, testing infra, etc.).  Instead we include
# a gem5-specific wrapper that builds only what we need.
include(${CMAKE_CURRENT_SOURCE_DIR}/googletest/CMakeLists.txt.gem5)

# pybind11 ships its own CMake build -- add it so targets are available.
add_subdirectory(pybind11)

# ── 3. Conditional libraries (optional cloned submodules) ────────────────────
# Each of these sets HAVE_<LIB> in the parent scope if the submodule is present.

add_subdirectory(dramsim2)
add_subdirectory(dramsim3)
add_subdirectory(dramsys)
