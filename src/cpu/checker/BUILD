load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

py_library(
    name = "checker_init",
    srcs = ["__init__.py"],
    deps = [
        ":CheckerCPU_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "CheckerCPU_py",
    srcs = ["CheckerCPU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "cpu_hh",
    hdrs = ["cpu.hh"],
    deps = [
        ":checker_cpu_sim_object",
        "//src/arch/generic:pcstate",
        "//src/base:statistics_hh",
        "//src/cpu:base_hh",
        "//src/cpu:debug_Checker",
        "//src/cpu:exec_context",
        "//src/cpu:inst_res",
        "//src/cpu:pc_event",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:static_inst",
        "//src/mem:request",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "thread_context_hh",
    hdrs = ["thread_context.hh"],
    deps = [
        ":cpu_hh",
        "//src/arch/generic:pcstate",
        "//src/cpu:debug_Checker",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "thread_context",
    deps = [":thread_context_hh"],
)

gem5_cc_library(
    name = "cpu_impl",
    hdrs = ["cpu_impl.hh"],
    deps = [
        ":cpu_hh",
        "//src/arch/generic:decoder",
        "//src/base:refcnt",
        "//src/cpu:debug_Checker",
        "//src/cpu:exetrace_hh",
        "//src/cpu:null_static_inst",
        "//src/cpu:reg_class",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/sim:full_system",
        "//src/sim:sim_object_hh",
        "//src/sim:stats",
    ],
)

gem5_cc_library(
    name = "cpu",
    srcs = ["cpu.cc"],
    hdrs = ["cpu.hh"],
    deps = [
        ":checker_cpu_sim_object",
        ":cpu_hh",
        "//src/arch/generic:pcstate",
        "//src/arch/generic:tlb",
        "//src/base:statistics_hh",
        "//src/cpu:base_hh",
        "//src/cpu:debug_Checker",
        "//src/cpu:exec_context",
        "//src/cpu:inst_res",
        "//src/cpu:pc_event",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/cpu:utils",
        "//src/mem:request",
        "//src/sim:eventq",
        "//src/sim:full_system",
    ],
)

gem5_sim_object(
    name = "checker_cpu_sim_object",
    extra_deps = [
        ":cpu",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/cpu:base_cpu_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:BaseCPU",
    ],
    sim_object_header = ":cpu_hh",
    sim_object_module = "src.cpu.checker.CheckerCPU",
    sim_object_name = "CheckerCPU",
    sim_object_py_lib = ":CheckerCPU_py",
)
