# Build Tools Agent Notes

This directory contains helper generators used by SCons to produce C++, Python,
headers, enums, debug flags, config data, and embedded blobs. Outputs usually
land under `build/` and should not be edited directly.

## Generator Changes

Treat generator output as a public build contract. Small formatting or ordering
changes can force broad rebuilds, perturb generated diffs, or break downstream
includes. Keep output deterministic by sorting unordered inputs and avoiding
host-specific paths, timestamps, or dictionary iteration where order matters.

When changing a generator, identify every SCons rule that calls it and the
generated files it owns. Update the source definition, SCons rule, and
generator together when their contracts change.

## Common Generated Surfaces

SimObject params and pybind glue are generated from Python SimObject
declarations. Enums, debug flags, Kconfig data, C++ config support, and
embedded Python blobs each have dedicated generator scripts here. If generated
output is wrong, fix the relevant source declaration or generator rather than
`build/` artifacts.

## Validation

Run `python3 -m py_compile` on touched generator files. Prefer a targeted SCons
build of the generated target or a dry run with `--debug=explain` before
running broad builds.
