# PR 1 — CMake Infrastructure: Implementation Plan

## Scope
Pure addition, zero deletions. `cmake --preset opt-x86` must configure successfully
(dependency summary prints); `cmake --build` is not expected to produce a binary —
no sources are registered until PR 2.

---

## Files to create (11 total)

```
CMakeLists.txt                    top-level, new
CMakePresets.json                 new
cmake/Gem5BuildTypes.cmake        real content
cmake/FindGem5Dependencies.cmake  real content
cmake/Gem5Kconfig.cmake           real content (parse-only; full Kconfig tool in PR 3)
cmake/Gem5Sources.cmake           real content
cmake/Gem5CodeGen.cmake           stub (PR 3)
cmake/Gem5Targets.cmake           stub (PR 3)
cmake/Gem5ConfigHeaders.cmake     stub (PR 3)
cmake/Gem5SwitchingHeaders.cmake  stub (PR 3)
.gitignore                        add build_cmake/
```

---

## `CMakeLists.txt`

Order matters — `gem5_deps` must be defined before `Gem5Sources` is included
so that OBJECT libraries can `target_link_libraries` against it at include time.

```cmake
cmake_minimum_required(VERSION 3.20)
project(gem5 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(Gem5BuildTypes)        # validates CMAKE_BUILD_TYPE, sets CMAKE_CXX_FLAGS_GEM5_*
include(FindGem5Dependencies)  # Python3, zlib (required); protobuf, png, HDF5 (optional)
include(Gem5Kconfig)           # validates GEM5_BUILD_VARIANT, parses build_opts/ -> CMake vars

# Must be defined before Gem5Sources so OBJECT libs can target_link_libraries against it
add_library(gem5_deps INTERFACE)
target_include_directories(gem5_deps INTERFACE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR})            # placeholder for generated headers (PR 3)
target_compile_features(gem5_deps INTERFACE cxx_std_20)
target_compile_definitions(gem5_deps INTERFACE
    $<$<CONFIG:GEM5_DEBUG>:GEM5_DEBUG;TRACING_ON=1>
    $<$<CONFIG:GEM5_OPT>:TRACING_ON=1>
    $<$<CONFIG:GEM5_FAST>:NDEBUG;TRACING_ON=0>)

include(Gem5Sources)           # gem5_add_source(), _gem5_ensure_object_target()
include(Gem5CodeGen)           # stub -- PR 3
include(Gem5Targets)           # stub -- PR 3
include(Gem5ConfigHeaders)     # stub -- PR 3
include(Gem5SwitchingHeaders)  # stub -- PR 3

# add_subdirectory(ext)        # PR 2
# add_subdirectory(src)        # PR 2
```

---

## `CMakePresets.json`

Version 6. 15 presets total: 3 hidden base presets + 12 named ISA presets.

### Hidden base presets

| Name | Inherits | Sets |
|---|---|---|
| `default` | — | generator=Ninja, binaryDir=`build_cmake/${presetName}`, `CMAKE_EXPORT_COMPILE_COMMANDS=ON` |
| `_debug-base` | default | `CMAKE_BUILD_TYPE=GEM5_DEBUG` |
| `_opt-base` | default | `CMAKE_BUILD_TYPE=GEM5_OPT` |
| `_fast-base` | default | `CMAKE_BUILD_TYPE=GEM5_FAST` |

### Named presets (each also sets `GEM5_BUILD_VARIANT`)

| Name | Inherits | `GEM5_BUILD_VARIANT` |
|---|---|---|
| `debug-x86` | `_debug-base` | `X86` |
| `opt-x86` | `_opt-base` | `X86` |
| `fast-x86` | `_fast-base` | `X86` |
| `debug-arm` | `_debug-base` | `ARM` |
| `opt-arm` | `_opt-base` | `ARM` |
| `fast-arm` | `_fast-base` | `ARM` |
| `debug-riscv` | `_debug-base` | `RISCV` |
| `opt-riscv` | `_opt-base` | `RISCV` |
| `fast-riscv` | `_fast-base` | `RISCV` |
| `debug-all` | `_debug-base` | `ALL` |
| `opt-all` | `_opt-base` | `ALL` |
| `fast-all` | `_fast-base` | `ALL` |

Other ISAs (MIPS, POWER, SPARC, NULL, etc.) are reached via
`-DGEM5_BUILD_VARIANT=MIPS` without a named preset.

Build outputs land in e.g. `build_cmake/opt-x86/`.

---

## `cmake/Gem5BuildTypes.cmake`

- Set `CMAKE_CXX_FLAGS_GEM5_DEBUG = "-O0 -ggdb3"` (CACHE STRING FORCE)
- Set `CMAKE_CXX_FLAGS_GEM5_OPT   = "-O3 -g"`     (CACHE STRING FORCE)
- Set `CMAKE_CXX_FLAGS_GEM5_FAST  = "-O3"`          (CACHE STRING FORCE)
- Same for `CMAKE_C_FLAGS_GEM5_*`
- If `CMAKE_BUILD_TYPE` is empty, default to `GEM5_OPT` with `message(STATUS ...)`
- Validate `CMAKE_BUILD_TYPE` ∈ `{GEM5_DEBUG, GEM5_OPT, GEM5_FAST}`;
  `message(FATAL_ERROR)` with the valid list on mismatch
- Set `GEM5_BUILD_TYPE_DESCRIPTION` string for configure summary
  (e.g. `"Optimized: -O3, debug info, TRACING_ON=1"`)

Compile definitions (`TRACING_ON`, `GEM5_DEBUG`, `NDEBUG`) are applied to
`gem5_deps` in `CMakeLists.txt` via generator expressions — not here —
to avoid any global `add_compile_definitions()`.

Mirrors SCons (`src/SConscript` lines 679–681):
```
debug → GEM5_DEBUG, TRACING_ON=1
opt   → TRACING_ON=1, -g
fast  → NDEBUG, TRACING_ON=0
```

---

## `cmake/FindGem5Dependencies.cmake`

### Required (FATAL_ERROR if absent)
- `find_package(Python3 REQUIRED COMPONENTS Interpreter Development)`
- `find_package(ZLIB REQUIRED)`

### Optional (sets `GEM5_HAS_*` booleans used by Gem5Kconfig and build summary)
- **Protobuf** — CONFIG mode first, MODULE fallback (handles modern and legacy installs)
- **PNG** — `find_package(PNG)`
- **HDF5** — `COMPONENTS CXX`
- **tcmalloc** — `GEM5_WITH_TCMALLOC` option + `find_library` fallback
- **Capstone** — `find_package(capstone)` or `find_library` fallback

### System header checks (`check_include_file`)
- `valgrind/valgrind.h`
- `linux/kvm.h` (x86_64 also validates `kvm_xsave` struct via `check_cxx_source_compiles`)
- `linux/if_tun.h`
- `fenv.h`
- `execinfo.h`

### Linker selection
- `GEM5_LINKER` option: `bfd | gold | lld | mold | ""`
- Appends `-fuse-ld=${GEM5_LINKER}` if set; validates with `try_compile`

Print a one-line found/missing summary for each dependency at end of file.

---

## `cmake/Gem5Kconfig.cmake`

Parse-only in PR 1. Full Kconfig tool integration (menuconfig, oldconfig, defconfig
targets; `GEM5_KCONFIG_OVERRIDE`; Ruby protocol normalization) deferred to PR 3.

```
set(GEM5_BUILD_VARIANT "X86" CACHE STRING "gem5 build variant (see build_opts/)")

Validation:
  file(GLOB _variants LIST_DIRECTORIES false "${PROJECT_SOURCE_DIR}/build_opts/*")
  extract basenames -> _valid_variants list
  if GEM5_BUILD_VARIANT not in list -> message(FATAL_ERROR) with valid list

Parsing:
  file(STRINGS "${PROJECT_SOURCE_DIR}/build_opts/${GEM5_BUILD_VARIANT}" _lines)
  foreach line:
    regex match "^([A-Za-z0-9_]+)=(.*)$"  -> key, value
    strip surrounding quotes from value
    if value == "y"  -> set(${key} TRUE  PARENT_SCOPE)
    if value == "n"  -> set(${key} FALSE PARENT_SCOPE)
    else             -> set(${key} "${value}" PARENT_SCOPE)
```

Result: `USE_X86_ISA`, `RUBY`, `PROTOCOL`, `NUMBER_BITS_PER_SET`, `BUILD_ISA`, etc.
available as CMake variables for PR 2 `target_sources()` conditions.

---

## `cmake/Gem5Sources.cmake`

Addresses ripopov's OBJECT library and no-globbing review comments directly.

### Global property
```cmake
define_property(GLOBAL PROPERTY GEM5_OBJECT_LIBS
    BRIEF_DOCS "List of all per-directory gem5 OBJECT targets"
    FULL_DOCS  "Accumulated by gem5_add_source(); consumed by Gem5Targets in PR 3")
```

### Internal: `_gem5_ensure_object_target(<out_var>)`
```
- Compute rel = CMAKE_CURRENT_SOURCE_DIR relative to PROJECT_SOURCE_DIR/src
- Replace "/" with "_" -> suffix
- target name = "gem5_obj_${suffix}"
  Examples:
    src/base/           -> gem5_obj_base
    src/cpu/o3/         -> gem5_obj_cpu_o3
    src/mem/ruby/network/ -> gem5_obj_mem_ruby_network
- If target does not yet exist:
    add_library(gem5_obj_${suffix} OBJECT)
    target_link_libraries(gem5_obj_${suffix} PRIVATE gem5_deps)
    set_property(GLOBAL APPEND PROPERTY GEM5_OBJECT_LIBS gem5_obj_${suffix})
- Set out_var to target name
```

### Public: `gem5_add_source(file [CONDITION <expr>] [APPEND_FLAGS <flags>])`
```
- Call _gem5_ensure_object_target(target)
- If CONDITION:    target_sources(${target} PRIVATE $<${CONDITION}:${file}>)
- If no CONDITION: target_sources(${target} PRIVATE ${file})
- If APPEND_FLAGS: set_source_files_properties(${file} PROPERTIES COMPILE_FLAGS ...)
```

### Public: `gem5_add_sources(file... [CONDITION <expr>])`
Loops over files, calls `gem5_add_source` for each.

### Utility: `gem5_get_all_object_sources(<out_var>)` (used by Gem5Targets in PR 3)
```
- get_property(libs GLOBAL PROPERTY GEM5_OBJECT_LIBS)
- For each lib: append $<TARGET_OBJECTS:${lib}> to result
- Set out_var to result list
```

---

## Four stub files

Each stub uses the same pattern (example for Gem5CodeGen):

```cmake
# cmake/Gem5CodeGen.cmake
#
# Code generation pipeline: ISA parser, SimObject params/enums, protobuf,
# debug flags, binary blobs, Python marshalling.
#
# Implemented in PR 3.
```

- `Gem5Targets.cmake` — final gem5 executable, libgem5_shared, unit test targets
- `Gem5ConfigHeaders.cmake` — config.hh generation from Kconfig variables
- `Gem5SwitchingHeaders.cmake` — ISA-specific header redirects

---

## `.gitignore`

Add: `build_cmake/`

---

## Reviewer-facing notes for PR description

1. **Smoke test**: `cmake --preset opt-x86` configures and prints a dependency
   summary. `cmake --build` is intentionally not expected to succeed — no sources
   are registered until PR 2.

2. **No global directives**: zero uses of `include_directories()`,
   `add_compile_options()`, or `add_compile_definitions()`. All propagation is
   through the `gem5_deps INTERFACE` target. Addresses ripopov's global-directives
   comment.

3. **OBJECT libraries**: one per source directory, created lazily on first
   `gem5_add_source()` call, named `gem5_obj_${dir_underscored}`. Addresses
   ripopov's OBJECT library comment.

4. **No globbing**: all source registration is explicit via `gem5_add_source()`.
   Addresses ripopov's globbing comment.

5. **Per-ISA presets**: each gem5 build targets one ISA. `ALL` is the
   multi-ISA special case, not the default — different from how SCons handles it.

6. **C++ standard is 20**: PR 2969 incorrectly used C++17. gem5's SConstruct
   line 597–598 explicitly requires `-std=c++20`.
