# Python And Stdlib Agent Notes

This tree contains Python infrastructure for gem5. The `m5` package provides
core simulator configuration machinery, SimObject support, params, proxies,
events, ticks, and embedded-Python integration. The `gem5` package contains
the gem5 standard library, including components, boards, processors, memory
systems, cache hierarchies, resources, and simulator helpers.

## Standard Library Stability

`src/python/gem5` is user-facing stdlib code and has a higher API-stability bar
than much of the rest of gem5. Users should generally be able to run older
configuration files on newer gem5 releases without surprising breakage.

Do not remove or rename stdlib APIs without a deprecation path. Provide at
least one release with a user-visible warning before removal, and keep
deprecations longer when migration risk is high. Existing examples include
warnings in `gem5/simulate/simulator.py`, `gem5/resources/resource.py`,
`gem5/resources/workload.py`, and `gem5/utils/simpoint.py`.

Use `m5.util.warn` for runtime deprecation warnings and keep replacement
guidance concrete. For renamed SimObject params, use `DeprecatedParam` from
`m5.params.deprecated_params` so old config files continue to map to the new
parameter name with a warning.

## SimObjects And Components

Python SimObject declarations are part of the C++/Python binding contract.
Changes to params, enums, ports, or class names may require updates to C++,
SCons registration, generated params, pybind output, tests, and documentation.
Do not edit generated params or bindings under `build/`; change the SimObject
definition or generator instead.

For stdlib components, preserve composability. Boards, processors, memory
systems, and cache hierarchies should expose clear constructor parameters and
avoid hidden global state. Prefer adding small extension points over special
cases in a prebuilt board unless the behavior is truly board-specific.

## Validation

For Python-only edits, run `python3 -m py_compile` on touched files when
practical. For SimObject, param, enum, or pybind-facing edits, also use a
targeted SCons build or `scons -Q --help` when build registration may be
affected.
