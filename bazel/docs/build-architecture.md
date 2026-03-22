# gem5 Build System Architecture

This document describes the dual build system (CMake + Bazel) for gem5,
with focus on the SimObject generation pipeline and modular target hierarchy.

## Overview

gem5 supports two build systems operating on the same source tree:

- **CMake + Ninja** (primary): native CMake build with Ninja generator.
  Configuration via Kconfig (`build_opts/<VARIANT>`). Build directory is
  separate from source (out-of-tree).

- **Bazel** (overlay): LLVM-style overlay pattern with BUILD files under
  `bazel/gem5-overlay/` merged into the source tree at build time via
  `configure.bzl`. Configuration via `string_flag`/`bool_flag` in
  `bazel/configs/BUILD.bazel` and `.bazelrc` presets.

Both produce the same `gem5` binary with identical SimObject sets.

## SimObject Generation Pipeline

SimObject parameter structs and enum bindings are generated from Python
class definitions (`.py` files scattered across `src/`) at build time.

### Generation is pure Python

The generation scripts (`build_tools/sim_object_param_struct_hh.py`,
`sim_object_param_struct_cc.py`, `enum_hh.py`, `enum_cc.py`) use only
the `m5.SimObject`, `m5.params`, and `m5.util.pybind` Python modules.
No C++ `_m5` module is needed. This enables standalone generation via
`build_tools/gen_all_params.py` using system Python.

### Generated artifacts per SimObject class

| Artifact | Path | Purpose |
|---|---|---|
| Param header | `params/<Name>.hh` | C++ parameter struct declaration |
| Pybind source | `python/_m5/param_<Name>.cc` | pybind11 bindings for Python |
| Enum header | `enums/<Name>.hh` | C++ enum declaration |
| Enum source | `enums/<Name>.cc` | Enum string conversion code |

### Shared generation tool: `gen_all_params.py`

`build_tools/gen_all_params.py` is a standalone script usable by both
CMake and Bazel:

```
python3 build_tools/gen_all_params.py \
    --src-root . --output-dir <dir> \
    [--simobj-files file1.py file2.py ...] \
    [--output-type params_hdrs|params_srcs|enums_hdrs|enums_srcs|all] \
    [--file-filter 'pattern1,pattern2'] \
    [--file-exclude 'pattern1,pattern2']
```

Key features:
- Discovers SimObject files from `simobject_py_files.bzl` (canonical source)
- Custom import hook resolves `m5.objects.*` dependencies on demand
- Retry-with-rollback for dependency-ordered loading
- `--file-filter`/`--file-exclude` enable per-bucket source generation
- Fail-closed: exits nonzero on any load or generation failure

## Config-Aligned Bucket Compilation

Generated param/enum `.cc` files are compiled in per-bucket OBJECT libraries
rather than a single monolithic library. Buckets align with configuration
axes so disabled ISAs/features are never compiled or linked.

### Bucket table

| Bucket | Config guard (CMake) | Config guard (Bazel) |
|---|---|---|
| `common` | always | always |
| `x86` | `CONF_USE_X86_ISA` | `//configs:x86_isa_enabled` |
| `arm` | `CONF_USE_ARM_ISA` | `//configs:arm_isa_enabled` |
| `riscv` | `CONF_USE_RISCV_ISA` | `//configs:riscv_isa_enabled` |
| `mips` | `CONF_USE_MIPS_ISA` | `//configs:mips_isa_enabled` |
| `power` | `CONF_USE_POWER_ISA` | `//configs:power_isa_enabled` |
| `sparc` | `CONF_USE_SPARC_ISA` | `//configs:sparc_isa_enabled` |
| `ruby` | `CONF_RUBY` | `//configs:ruby_enabled` |
| `gpu` | `CONF_BUILD_GPU` | `//configs:gpu_enabled` |
| `systemc` | `CONF_USE_SYSTEMC` | `//configs:systemc_enabled` |
| `kvm` | `CONF_USE_KVM` | `//configs:kvm_enabled` |
| `test_objects` | `CONF_USE_TEST_OBJECTS` | `//configs:test_objects_enabled` |

### Enum ownership rule

Each enum is assigned to the same bucket as the SimObject `.py` file
that defines it. This prevents duplicate definitions across buckets.

## CMake Target Hierarchy

```
Source .cc files
    |
    v
Per-directory OBJECT libraries (gem5_obj_base, gem5_obj_sim, ...)
    |
    v
Subsystem STATIC libraries (gem5_base, gem5_sim, gem5_mem, gem5_cpu, ...)
    |
    v                              Generated param/enum .cc
gem5_all (thin aggregation)  <---  Per-bucket OBJECT libs (gem5_gen_common, gem5_gen_x86, ...)
    |                              +
    v                              gem5_generated (debug flags, blobs)
gem5 executable                    +
(per-subsystem whole-archive)      gem5_pysources (embedded Python bytecode)
```

### How `_gem5_derive_bucket()` classifies SimObjects

Called from `gem5_add_simobject()` during CMakeLists.txt traversal:

- `src/arch/<isa>/` or `src/dev/<isa>/` -> `<isa>` bucket
- `src/dev/lupio/` -> `riscv`
- `src/mem/ruby/` -> `ruby`
- `src/gpu-compute/`, `src/dev/hsa/`, `src/dev/amdgpu/`, `src/arch/amdgpu/` -> `gpu`
- `src/systemc/` -> `systemc`
- `src/cpu/kvm/` -> `kvm`
- `src/test_objects/` -> `test_objects`
- Everything else -> `common`

Override with explicit `BUCKET` parameter for non-standard locations.

### Key CMake files

| File | Purpose |
|---|---|
| `cmake/Gem5CodeGen.cmake` | SimObject codegen, `_gem5_derive_bucket()`, per-bucket OBJECT libs |
| `cmake/Gem5Targets.cmake` | Final targets, whole-archive linking |
| `cmake/Gem5Subsystems.cmake` | Subsystem STATIC library definitions |
| `cmake/Gem5Py.cmake` | Python embedding (gem5py, gem5py_m5) |
| `cmake/Gem5Kconfig.cmake` | Kconfig processing for build variants |

## Bazel Target Hierarchy

```
Source .cc files
    |
    v
Per-subsystem cc_library (//src/sim:sim, //src/cpu:cpu, ...)
    |
    v                                SimObject .py files
gem5_all cc_library  <---  Per-bucket generation (active path):
    |                      sim_object_params_bucket (headers)
    v                      sim_object_params_bucket_srcs (per-bucket .cc)
gem5 binary                + gem5_simobject_pysources (embedded Python)
                           + slicc_sim_object_params (SLICC bridge, gem5py_m5)
```

**Current state:** Bazel uses per-bucket generation via `gen_all_params.py`
with system Python and `--source-bucket` class-based filtering. Each class's
bucket is derived from the defining `.py` file's path. `gem5_all` depends on
`sim_object_params_bucket` (headers) and `sim_object_params_bucket_srcs`
(per-bucket compiled sources with `select()` gating).

**SLICC bridge:** SLICC-generated SimObjects (protocol controllers like
`L1Cache_Controller`) are handled by a separate `slicc_sim_object_params`
target that uses `gem5py_m5`, because SLICC `.py` files are build-time
generated and not available to `gen_all_params.py`. The per-bucket header
target depends on this bridge for protocol controller param headers.

**Feature-gated files:** CHI, KVM, protobuf, capstone, and DRAMsim3
SimObject files are included in `_ALL_SIMOBJ_PY_SRCS` via `select()`
expressions, so they're only processed when their corresponding config
flag is enabled.

### Overlay pattern

BUILD files live under `bazel/gem5-overlay/` and are merged with
the source tree at build time by `configure.bzl`. This avoids
polluting the source tree with BUILD files.

### Key Bazel files

| File | Purpose |
|---|---|
| `bazel/MODULE.bazel` | Bzlmod module definition |
| `bazel/.bazelrc` | Composable build config presets |
| `bazel/configure.bzl` | Overlay repository rule |
| `bazel/extensions.bzl` | Module extension (source/ext library setup) |
| `bazel/gem5-overlay/rules/sim_object.bzl` | SimObject generation rules |
| `bazel/gem5-overlay/rules/simobject_py_files.bzl` | Canonical SimObject file inventory |
| `bazel/gem5-overlay/BUILD.bazel` | Top-level target definitions |
| `bazel/configs/BUILD.bazel` | Configuration flags |

## How To: Add a New SimObject

### CMake

In the directory's `CMakeLists.txt`:

```cmake
gem5_add_simobject(MyNewObject.py
    SIM_OBJECTS MyNewObject
    ENUMS MyNewEnum
)
```

The bucket is auto-derived from the directory. Use `BUCKET <name>` to
override for non-standard locations:

```cmake
gem5_add_simobject(MySpecialObject.py
    SIM_OBJECTS MySpecialObject
    BUCKET gpu
)
```

### Bazel

1. Add the `.py` file path to the appropriate list in
   `bazel/gem5-overlay/rules/simobject_py_files.bzl`
   (e.g., `SIMOBJECT_PY_FILES_BASE` for common, `SIMOBJECT_PY_FILES_ARM`
   for ARM-specific).

2. If the file defines a new ISA-specific SimObject whose param struct
   references constructors from that ISA's library, ensure it's in the
   ISA-specific list (not BASE).

## Configuration System

### CMake

Configuration flows through Kconfig:
1. User specifies `-DGEM5_BUILD_VARIANT=X86`
2. `cmake/Gem5Kconfig.cmake` processes `build_opts/X86` through
   `src/Kconfig` to generate `CONF_*` CMake variables
3. Subsystems and per-bucket libs are conditionally created based on
   `CONF_*` variables

### Bazel

Configuration uses native Bazel flags:
- `string_flag`: `primary_isa`, `ruby_protocol`
- `bool_flag`: `use_x86_isa`, `ruby_enabled`, `gpu_enabled`, etc.
- `.bazelrc` presets: `--config=x86`, `--config=arm`, `--config=opt`, etc.
- Composable: `bazel build @gem5_sources//:gem5 --config=opt --config=x86`

## Design Rationale

### Why config-aligned buckets (not per-module or monolithic)

- **Per-module (~276 targets)**: Requires maintaining a full py_library
  dependency graph for SimObject inheritance. Too verbose for practical
  maintenance.
- **Monolithic (1 target)**: Compiles disabled ISA/feature params,
  wastes build time, and links unused code.
- **Config-aligned buckets (~12 targets)**: Maps naturally to existing
  configuration flags. Disabled ISAs are never compiled. Each bucket
  is small enough for fast incremental rebuilds.

### Why system Python for Bazel param generation

The C++ `gem5py_m5` embedded interpreter is heavyweight (~30s build)
and unnecessary for non-SLICC param generation. `gen_all_params.py`
uses system Python with `--source-bucket` class-based filtering: each
SimObject class's bucket is derived from the defining `.py` file's path
(same logic as CMake's `_gem5_derive_bucket()`). This correctly handles
multi-class-per-file SimObjects (e.g., `BaseRoutingUnit` defined in
`SimpleNetwork.py` under `src/mem/ruby/` gets bucket `ruby`).

SLICC-generated SimObjects still require `gem5py_m5` because SLICC `.py`
files are build-time generated. This is handled by the `slicc_sim_object_params`
bridge target.

### CMake design note: central vs subsystem linking

The original plan called for per-bucket OBJECT libraries to be linked
by their owning subsystem STATIC libraries (e.g., `gem5_arch_x86`
links `gem5_gen_x86`). The actual implementation injects all per-bucket
OBJECT library objects directly into `gem5_all` and the gem5 executable
via `$<TARGET_OBJECTS:...>` in `Gem5Targets.cmake`. This is functionally
equivalent and simpler: bucket OBJECT libraries don't need subsystem
ownership metadata, and the whole-archive link ensures all factory
constructors are preserved regardless of which target includes them.

### Comparison with bazel branch (PR #3004)

The bazel branch (powerjg) demonstrated per-SimObject modular targets
with sandbox-based generation. Key differences from this implementation:

| Aspect | bazel branch | This implementation |
|---|---|---|
| Granularity | Per-SimObject (~300 targets) | Per-bucket (~12 targets) |
| Source tree | BUILD files in src/ | Overlay pattern |
| SimObject.py changes | 179-line patch (duck-typing) | Zero changes |
| Import resolution | Per-target Python sandbox | Shared import hook |
| Coverage | ~143 SimObjects | All (~650 SimObjects) |

### Future: per-module targets

When a full `py_library` dependency graph for SimObject `.py` files
exists, per-module targets become practical. This would give finest-grained
incremental builds and Bazel remote caching per SimObject. The current
bucket approach is the stepping stone.
