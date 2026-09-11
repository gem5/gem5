# SConscript to Bazel BUILD converter

This tool converts gem5 SConscript to the conceptually equivalent Bazel BUILD
file for gem5 to adopt Bazel. So the intention is not to build a complete BUILD
file generater being able to parse any SConscripts. The scope is thus limited to
the existing gem5 SConscripts, and the converter may evolve as new SConscripts
are added.

See `build_tools/bazel/README.md` for a larger picture and the Bazel target
layout scheme.
