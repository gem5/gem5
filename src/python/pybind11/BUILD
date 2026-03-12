load("//bazel:rules.bzl", "gem5_pybind_library")

package(
    default_visibility = ["//visibility:public"],
)

# TODO(jlowepower): There are many things here that cause difficult
# dependencies and bloat the build. Some of the worst offenders are:
# - cpu/probes/pc_count_pair
# - base/loader/elf_object (many, many transitive dependencies)
# - base/inet (dependencies on dev/net)
gem5_pybind_library(
    name = "gem5_pybind11_core",
    srcs = [
        "core.cc",
    ],
    deps = [
        ":pybind",
        "//src/base:addr_range",
        "//src/base:inet",
        "//src/base:inet_hh",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:socket",
        "//src/base:temperature",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/loader:elf_object",
        "//src/cpu/probes:pc_count_pair",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:drain",
        "//src/sim:serialize",
        "//src/sim:sim_object_hh",
    ],
)

gem5_pybind_library(
    name = "debug",
    srcs = ["debug.cc"],
    deps = [
        ":pybind",
        "//src/base:compiler",
        "//src/base:debug",
        "//src/base:output",
        "//src/base:trace",
        "//src/sim:debug",
    ],
)

gem5_pybind_library(
    name = "pybind",
    hdrs = ["pybind.hh"],
)

gem5_pybind_library(
    name = "stats",
    srcs = ["stats.cc"],
    deps = [
        "//bazel/generated/config",
        "//src/base:statistics_hh",
        "//src/base/stats:text",
        "//src/sim:stat_control",
        "//src/sim:stat_register",
    ],
)

gem5_pybind_library(
    name = "event",
    srcs = ["event.cc"],
    deps = [
        "//src/base:logging",
        "//src/sim:eventq",
        "//src/sim:sim_events",
        "//src/sim:sim_exit",
        "//src/sim:simulate",
    ],
)

gem5_pybind_library(
    name = "object_file",
    srcs = ["object_file.cc"],
    deps = [
        ":pybind",
        "//src/base/loader:object_file",
        "//src/sim:init",
    ],
)

gem5_pybind_library(
    name = "port",
    srcs = ["port.cc"],
    deps = [
        ":pybind",
        "//src/mem:port_proxy",
    ],
)
