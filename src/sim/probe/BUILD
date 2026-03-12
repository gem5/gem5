load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

py_library(
    name = "probe_init",
    srcs = ["__init__.py"],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_ProbeVerbose",
    flag = "ProbeVerbose",
)

gem5_cc_library(
    name = "pmu",
    hdrs = ["pmu.hh"],
    deps = [
        ":probe",
    ],
)

gem5_cc_library(
    name = "mem",
    hdrs = ["mem.hh"],
    deps = [
        ":probe",
        "//src/mem:packet_hh",
    ],
)

py_library(
    name = "Probe_py",
    srcs = ["Probe.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "probe_listener_object_sim_object",
    extra_deps = [
        ":probe_listener_object",
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":probe",
    sim_object_module = "src.sim.probe.Probe",
    sim_object_name = "ProbeListenerObject",
    sim_object_py_lib = ":Probe_py",
)

gem5_cc_library(
    name = "probe_listener_object",
    srcs = ["probe_listener_object.cc"],
    hdrs = ["probe_listener_object.hh"],
    deps = [
        ":probe",
        ":probe_listener_object_sim_object",
        "//src/base:compiler",
        "//src/base:named",
        "//src/base:trace",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "probe",
    srcs = ["probe.cc"],
    hdrs = ["probe.hh"],
    deps = [
        ":debug_ProbeVerbose",
        "//src/base:compiler",
        "//src/base:named",
        "//src/base:trace",
    ],
)
