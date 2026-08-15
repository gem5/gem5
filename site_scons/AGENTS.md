# SCons Agent Notes

This directory contains gem5's SCons extensions, source registration helpers,
configure probes, Kconfig integration, Python path setup, and git-hook support.
Small changes here can affect every build variant.

## Build Logic

Keep emitters and actions separated: emitters should discover targets and
dependencies, while actions should write files. Avoid unstable generated
output, implicit environment mutation, or dependency edges that change between
builds.

For configure checks, inspect the generated `scons_config.log` in the relevant
build directory before assigning cause. Headline failures can hide compiler
flag, include path, linker, Python, or pkg-config details.

## Registration And Generated Files

SCons registration controls C++ sources, Python sources, SimObjects, enums,
debug flags, Kconfig files, and generated build products. If output under
`build/` is wrong, change the registration rule, generator, or source input
rather than generated files.

Keep dependency and compiler support policy aligned with `SConstruct`,
`.github/workflows`, `util/dockerfiles`, and docs.

## Validation

For SCons edits, run `python3 -m py_compile` on touched Python files and
treat it as a syntax check only. `scons -Q --help` evaluates the top-level
`SConstruct` and imported site initialization, so it is useful for that narrow
startup and top-level option-registration path. With no build target it does
not read build-specific `SConsopts` or `SConscript` files, run configure
probes, or validate source and generator registration.

Use the narrowest explicit build target that reaches the changed logic when
registration or configure behavior matters. A dry run with
`scons -n <target> --debug=explain` can help inspect dependency decisions after
configure state is current, but it does not execute actions and can fail if a
configure test needs to update its output.
