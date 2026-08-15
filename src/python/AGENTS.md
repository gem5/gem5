# Python And Stdlib Agent Notes

This tree contains Python infrastructure for gem5. The `m5` package provides
core simulator configuration machinery, SimObject support, params, proxies,
events, ticks, and embedded-Python integration. The `gem5` package contains
the gem5 standard library, including components, boards, processors, memory
systems, cache hierarchies, resources, and simulator helpers.

## Standard Library Stability

`src/python/gem5` is the user-facing standard library, so compatibility is an
explicit review concern. The repository contains deprecation mechanisms, but
does not define one universal deprecation period for every stdlib API. Follow
the policy established by the affected subsystem and its maintainers; do not
invent a fixed release count. Existing deprecation examples include warnings
in `gem5/simulate/simulator.py`, `gem5/resources/resource.py`,
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

## Pybind11 Bindings

gem5 uses pybind11 to expose C++ simulator APIs to its embedded Python
interpreter. Core hand-written bindings live under `src/python/pybind11` and
are registered in `src/python/SConscript`. Subsystem-specific bindings may live
beside their C++ implementation and use `EmbeddedPyBind` to add `_m5`
submodules.

Bindings for SimObject params, enums, and `cxx_exports` are generated from
Python declarations by scripts under `build_tools`; update those declarations
or generators rather than generated files under `build/`. Keep Python-facing
names, C++ signatures, ownership and return-value policies, SCons registration,
and tests synchronized when changing a binding.

## Validation

For Python-only edits, run `python3 -m py_compile` on touched files when
practical, but remember that it checks syntax rather than imports or runtime
behavior. For SimObject, param, enum, or pybind-facing edits, build the
affected gem5 target so the generated bindings are compiled. `scons -Q --help`
only evaluates the top-level SConstruct and site initialization; it does not
process the build-specific SConscripts or generate and compile bindings.
