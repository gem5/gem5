load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

py_library(
    name = "probes_init",
    srcs = ["__init__.py"],
    deps = [
        ":PcCountTracker_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_PcCountTracker",
    flag = "PcCountTracker",
)

gem5_cc_library(
    name = "pc_count_pair",
    hdrs = ["pc_count_pair.hh"],
    deps = [
        "//src/base:types",
    ],
)

py_library(
    name = "PcCountTracker_py",
    srcs = ["PcCountTracker.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:Probe",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

gem5_sim_object(
    name = "pc_count_tracker_sim_object",
    extra_deps = [
        "//src/sim:sim_object",
        "//src/cpu:base",
        ":pc_count_tracker_manager",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/cpu:base_cpu_sim_object",
        "//src/sim/probe:probe_listener_object_sim_object",
        ":pc_count_tracker_manager_sim_object",
        ":pc_count_pair",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:BaseCPU",
    ],
    sim_object_header = ":pc_count_tracker",
    sim_object_module = "src.cpu.probes.PcCountTracker",
    sim_object_name = "PcCountTracker",
    sim_object_py_lib = ":PcCountTracker_py",
)

gem5_cc_library(
    name = "pc_count_tracker",
    srcs = ["pc_count_tracker.cc"],
    hdrs = ["pc_count_tracker.hh"],
    deps = [
        ":pc_count_tracker_manager",
        ":pc_count_tracker_sim_object",
        "//src/sim/probe:probe_listener_object",
    ],
)

gem5_sim_object(
    name = "pc_count_tracker_manager_sim_object",
    extra_deps = [
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":pc_count_pair",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:Probe",
    ],
    sim_object_header = ":pc_count_tracker_manager",
    sim_object_module = "src.cpu.probes.PcCountTracker",
    sim_object_name = "PcCountTrackerManager",
    sim_object_py_lib = ":PcCountTracker_py",
)

gem5_cc_library(
    name = "pc_count_tracker_manager",
    srcs = ["pc_count_tracker_manager.cc"],
    hdrs = ["pc_count_tracker_manager.hh"],
    deps = [
        ":debug_PcCountTracker",
        ":pc_count_pair",
        ":pc_count_tracker_manager_sim_object",
        "//src/cpu:base",
        "//src/sim:sim_exit",
    ],
)
