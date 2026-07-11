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
`scons -Q --help` when option parsing, registration, or configure behavior may
be affected. For dependency-edge or generator changes, use targeted
`scons -n ... --debug=explain` and `sconsign` checks before broad rebuilds.
