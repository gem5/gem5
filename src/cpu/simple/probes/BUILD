load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

py_library(
    name = "probes_init",
    srcs = ["__init__.py"],
    deps = [
        ":SimPoint_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "SimPoint_py",
    srcs = ["SimPoint.py"],
    deps = [
        "//src/python/m5/objects:Probe",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "sim_point_sim_object",
    extra_param_deps = [
        "//src/sim/probe:probe_listener_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":simpoint",
    sim_object_module = "src.cpu.simple.probes.SimPoint",
    sim_object_name = "SimPoint",
    sim_object_py_lib = ":SimPoint_py",
)

gem5_cc_library(
    name = "simpoint",
    srcs = ["simpoint.cc"],
    hdrs = ["simpoint.hh"],
    deps = [
        ":sim_point_sim_object",
        "//src/base:output",
        "//src/cpu:simple_thread",
        "//src/sim/probe:probe_listener_object",
    ],
)
