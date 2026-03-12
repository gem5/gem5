# Walkthrough: Dynamic SimObject Loading and Build Stability

We have completed the refactoring of gem5's build system to support dynamic SimObject loading and resolved critical build issues encountered with complex configurations.

## Core Accomplishments

### 1. Dynamic SimObject Loading Mechanism
- **Sub-Modularity**: SimObject parameter registration code is now compiled into individual Python extensions (`_m5_param_<Type>`) rather than being statically linked into `_m5`.
- **On-Demand Loading**: Modified [getCCParams](file:///Users/jlowepower/Code/gem5/gem5-bazel/src/python/m5/SimObject.py#1227-1312) in [src/python/m5/SimObject.py](file:///Users/jlowepower/Code/gem5/gem5-bazel/src/python/m5/SimObject.py) to dynamically import these extensions when required.
- **Pointer Sharing**: Enabled communication between standalone extensions and the core `_m5` module via a Python capsule pointer (`_mod_ptr`).

### 2. Sandbox Reliability & Class Identity
- **Unique Sandboxes**: Refactored [bazel/rules.bzl](file:///Users/jlowepower/Code/gem5/gem5-bazel/bazel/rules.bzl) to create a dedicated sandbox directory for each parameter generation target, preventing path conflicts.
- **Canonical Paths**: Ensured that `src/python/m5/` is mapped to `m5/` at the sandbox root, maintaining consistent Python imports across all build stages.
- **Identity Fallbacks**: Added name-based class comparison to `SingleTypeParamDesc.convert` in `base_params.py` to handle cases where classes might be imported via different paths (e.g., duplicate imports).

### 3. SimObject Registration Fixes
- **all_objects.py**: Implemented automated generation of `m5.objects.all_objects` to ensure all SimObjects are discoverable for registration.
- **Metaclass Robustness**: Updated `MetaSimObject.__init__` to explicitly check `cls._init_called` in its local dictionary, ensuring subclasses of already-initialized parents are correctly processed.
- **CheckerCPU Visibility**: Added missing dependencies for `DummyChecker` and `CheckerCPU` to ensure they are available during parameter generation.

### 4. Build Environment & Code Generation
- **buildEnv Flags**: Expanded `m5.defines.buildEnv` to include `HAVE_ZLIB` and `HAVE_TUNTAP`, preventing `KeyError` exceptions in SimObject definitions.
- **GDB XML Generator**: Fixed `create_gdbxml.py` to match the expectations of `remote_gdb.cc` (wrapping data in the `Blobs` namespace and using the `_len` suffix for size).
- **zlib Canonicalization**: Resolved external dependency issues by consistently referencing `@zlib` (via Bzlmod).

## Changes Made

- **Minor CPU Integration**:
    - Added `BaseMinorCPU` and associated SimObjects to `bazel/sim_objects.bzl`.
    - Updated `src/cpu/minor/BUILD` with `gem5_sim_object` and `gem5_enum_generator` rules.
    - Added `sim_object_header` to Minor CPU targets to fix C++ header dependencies.
- **TimingExpr Integration**:
    - Added `TimingExpr` SimObjects and `enum_TimingExprOp` to `src/cpu/BUILD` and `src/cpu/init.py`.
    - Integrated `TimingExpr` into the main `cpu_init` dependency list.
- **Build System Fixes**:
    - Modified `gem5_sim_object` macro in `bazel/rules.bzl` to correctly handle `None` for `sim_object_header`.
    - Added `PROTOCOL` to `m5.defines.buildEnv` to support Ruby protocol imports.
    - Initialized `m5.options` in `src/python/m5/__init__.py` to provide a default `OptionParser` object for scripts.
- **Verification Fixes**:
    - Fixed import paths in `bazel/configs/test.py` to use `m5.objects`.
    - Corrected `m5.options` usage in `test.py`.

## Verification Results

### Automated Tests

- **BaseMinorCPU Build**: Successfully built the Minor CPU shim.
  ```bash
  bazel build //src/python/m5/objects:BaseMinorCPU
  ```
- **Simulation Test**: Successfully ran the `//bazel/configs:test` target.
  ```bash
  bazel run //bazel/configs:test
  ```
  Result: `Beginning simulation!`, `Exiting @ tick ... because simulate() limit reached`, and `Exit code: 0`.

### Summary

The `ModuleNotFoundError` for `BaseMinorCPU` has been resolved by properly integrating the Minor CPU components into the Bazel build system and addressing several underlying build system and Python framework issues that were revealed during the process.

### Multi-Target Build Success
We successfully built multiple complex configurations, including full-system ARM targets:
```bash
bazel build //bazel/configs:test2 //bazel/configs:test3 //bazel/configs:arm_hello_se
```
**Status: SUCCESS** (87 actions executed in the final incremental build).

### Simulation Proof
The generic `test` target continues to function correctly with dynamic loading:
````carousel
```bash
$ bazel run //bazel/configs:test
Global frequency set at 1000000000000 ticks per second
Beginning simulation!
Exiting @ tick 18446744073709551615 because simulate() limit reached
```
<!-- slide -->
```bash
$ find bazel-out/... -name "_m5_param_*.so"
.../_main/src/sim/_m5_param_Root.so
.../_main/src/learning_gem5/part2/_m5_param_HelloObject.so
...
```
````

## Benefits
- **Granular Dependencies**: Only build and load the SimObjects needed for a specific configuration.
- **Build Performance**: Smaller `_m5` binary and reduced linking overhead for individual SimObjects.
- **Scalability**: New SimObjects can be added to the Bazel build with minimal impact on the core system.
