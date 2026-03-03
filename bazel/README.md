# gem5 Bazel Build

This directory contains the Bazel build system for gem5, designed to coexist
with the CMake build. The Bazel build uses an LLVM-style overlay pattern where
BUILD files are maintained separately from source files.

## Requirements

- Bazel 9.0.0+
- Python 3 development headers
- GCC or Clang with C++17 support
- zlib development headers

## Quick Start

```bash
cd bazel
bazel build @gem5_sources//:gem5 --config=opt --config=x86
```

## Build Configurations

Configs are composable (build type and ISA are orthogonal):

### Build Types
- `--config=dbg` - Debug build (-O0, -ggdb3, tracing enabled)
- `--config=opt` - Optimized build (-O3, -g, tracing enabled)
- `--config=fast` - Fast build (-O3, NDEBUG, tracing disabled)

### ISA Selection
- `--config=x86` - X86 ISA (default)
- `--config=arm` - ARM ISA
- `--config=riscv` - RISC-V ISA
- `--config=mips` - MIPS ISA
- `--config=power` - POWER ISA
- `--config=sparc` - SPARC ISA
- `--config=all` - All ISAs enabled
- `--config=vega_x86` - X86 with VEGA GPU

### Compiler Selection
- `--config=clang` - Build with Clang/LLD
- `--config=gcc` - Build with GCC

### Sanitizers
- `--config=asan` - Address Sanitizer
- `--config=ubsan` - Undefined Behavior Sanitizer

## Architecture

The build uses an overlay mechanism inspired by LLVM's `utils/bazel/`:

1. `configure.bzl` contains a repository rule that symlinks the gem5 source
   tree and overlays BUILD files from `gem5-overlay/`.
2. `extensions.bzl` sets up external dependency repositories from `ext/`.
3. Custom rules in `rules/` handle gem5-specific code generation (ISA parser,
   SLICC, debug flags, SimObject params, etc.).
4. Configuration flags in `configs/` replace Kconfig with Bazel-native
   `bool_flag`/`string_flag` and `config_setting` rules.

## Directory Structure

```
bazel/
+-- MODULE.bazel          Module definition and dependencies
+-- .bazelrc              Build configuration presets
+-- .bazelversion         Required Bazel version
+-- configure.bzl         Overlay repository rule
+-- extensions.bzl        Module extensions for repos
+-- python_detect.bzl     System Python detection
+-- rules/                Custom Starlark rules
+-- configs/              Build flags and config settings
+-- gem5-overlay/         BUILD file overlays for src/
+-- third_party_build/    BUILD files for ext/ libraries
```

## Running Tests

```bash
cd bazel
bazel test @gem5_sources//:gem5_tests --config=opt --config=x86
```

## Downstream Integration

### As a Local Dependency

```starlark
# downstream MODULE.bazel
bazel_dep(name = "gem5")
local_path_override(
    module_name = "gem5",
    path = "third_party/gem5/bazel",
)
```

## Feature Flags

Optional features can be enabled via `--flag_name=value`:

```bash
# Enable KVM
bazel build @gem5_sources//:gem5 --config=opt --config=x86 \
  --@gem5_sources//configs:use_kvm=True

# Enable GPU (VEGA)
bazel build @gem5_sources//:gem5 --config=opt --config=vega_x86

# Disable Ruby
bazel build @gem5_sources//:gem5 --config=opt --config=x86 \
  --@gem5_sources//configs:use_ruby=False
```
