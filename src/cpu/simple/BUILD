load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

py_library(
    name = "simple_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseAtomicSimpleCPU_py",
        ":BaseNonCachingSimpleCPU_py",
        ":BaseSimpleCPU_py",
        ":BaseTimingSimpleCPU_py",
        "//src/cpu/simple/probes:probes_init",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_SimpleCPU",
    flag = "SimpleCPU",
)

py_library(
    name = "BaseSimpleCPU_py",
    srcs = ["BaseSimpleCPU.py"],
    deps = [
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
        "//src/python/m5/objects:DummyChecker",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "BaseAtomicSimpleCPU_py",
    srcs = ["BaseAtomicSimpleCPU.py"],
    deps = [
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
        "//src/python/m5/objects:DummyChecker",
        "//src/python/m5/objects:SimPoint",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "BaseTimingSimpleCPU_py",
    srcs = ["BaseTimingSimpleCPU.py"],
    deps = [
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BaseSimpleCPU",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "BaseNonCachingSimpleCPU_py",
    srcs = ["BaseNonCachingSimpleCPU.py"],
    deps = [
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseAtomicSimpleCPU",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "exec_context",
    hdrs = ["exec_context.hh"],
    deps = [
        ":base_hh",
        "//src/base:types",
        "//src/cpu:base_hh",
        "//src/cpu:exec_context",
        "//src/cpu:reg_class",
        "//src/cpu:static_inst_fwd",
        "//src/cpu:translation",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_simple_cpu_sim_object",
        ":exec_context",
        "//src/arch/generic:decoder",
        "//src/arch/generic:pcstate",
        "//src/base:cprintf",
        "//src/base:inifile",
        "//src/base:logging",
        "//src/base:pollevent",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/loader:symtab",
        "//src/cpu:base",
        "//src/cpu:debug_Decode",
        "//src/cpu:debug_ExecFaulting",
        "//src/cpu:debug_Fetch",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:debug_Quiesce",
        "//src/cpu:exec_context",
        "//src/cpu:exetrace",
        "//src/cpu:exetrace_hh",
        "//src/cpu:null_static_inst",
        "//src/cpu:pc_event",
        "//src/cpu:simple_thread",
        "//src/cpu:smt",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/cpu/checker:cpu",
        "//src/cpu/checker:thread_context",
        "//src/cpu/checker:thread_context_hh",
        "//src/cpu/pred:bpred_unit",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem:request",
        "//src/sim:byteswap",
        "//src/sim:debug",
        "//src/sim:eventq",
        "//src/sim:faults",
        "//src/sim:full_system",
        "//src/sim:sim_events",
        "//src/sim:sim_object",
        "//src/sim:stats",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "base_hh",
    hdrs = ["base.hh"],
    deps = [
        "//src/arch/generic:pcstate",
        "//src/base:statistics_hh",
        "//src/cpu:base_hh",
        "//src/cpu:exec_context",
        "//src/cpu:pc_event_hh",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:static_inst",
        "//src/cpu/checker:cpu",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem:request",
        "//src/sim:eventq",
        "//src/sim:full_system",
        "//src/sim:system",
    ],
)

gem5_sim_object(
    name = "base_simple_cpu_sim_object",
    extra_param_deps = [
        "//src/cpu:base_cpu_sim_object",
        "//src/cpu/pred:branch_predictor_sim_object",
        "//src/sim:sim_object",
        "//src/cpu/pred:bpred_unit",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.cpu.simple.BaseSimpleCPU",
    sim_object_name = "BaseSimpleCPU",
    sim_object_py_lib = ":BaseSimpleCPU_py",
)

gem5_cc_library(
    name = "atomic",
    srcs = ["atomic.cc"],
    hdrs = ["atomic.hh"],
    deps = [
        ":base",
        ":base_atomic_simple_cpu_sim_object",
        ":debug_SimpleCPU",
        ":exec_context",
        "//src/arch/generic:decoder",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:statistics",
        "//src/cpu:base",
        "//src/cpu:debug_ExecFaulting",
        "//src/cpu:exetrace",
        "//src/cpu:exetrace_hh",
        "//src/cpu:inst_seq",
        "//src/cpu:reg_class",
        "//src/cpu:thread_context",
        "//src/cpu:utils",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/mem:physical",
        "//src/mem:request",
        "//src/sim:debug_Drain",
        "//src/sim:faults",
        "//src/sim:full_system",
        "//src/sim:sim_object",
        "//src/sim:system",
        "//src/sim/probe",
    ],
)

gem5_sim_object(
    name = "base_atomic_simple_cpu_sim_object",
    extra_param_deps = [
        ":base_simple_cpu_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:BaseSimpleCPU",
        "//src/python/m5/objects:SimPoint",
    ],
    sim_object_header = ":atomic",
    sim_object_module = "src.cpu.simple.BaseAtomicSimpleCPU",
    sim_object_name = "BaseAtomicSimpleCPU",
    sim_object_py_lib = ":BaseAtomicSimpleCPU_py",
)

gem5_cc_library(
    name = "timing",
    srcs = ["timing.cc"],
    hdrs = ["timing.hh"],
    deps = [
        ":base",
        ":base_timing_simple_cpu_sim_object",
        ":debug_SimpleCPU",
        ":exec_context",
        "//src/arch/generic:decoder",
        "//src/arch/generic:mmu",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/cpu:base",
        "//src/cpu:debug_ExecFaulting",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:debug_Mwait",
        "//src/cpu:exetrace",
        "//src/cpu:exetrace_hh",
        "//src/cpu:thread_context",
        "//src/cpu:translation",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Config",
        "//src/sim:debug_Drain",
        "//src/sim:faults",
        "//src/sim:full_system",
        "//src/sim:system",
    ],
)

gem5_sim_object(
    name = "base_timing_simple_cpu_sim_object",
    extra_param_deps = [
        ":base_simple_cpu_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:BaseSimpleCPU",
    ],
    sim_object_header = ":timing",
    sim_object_module = "src.cpu.simple.BaseTimingSimpleCPU",
    sim_object_name = "BaseTimingSimpleCPU",
    sim_object_py_lib = ":BaseTimingSimpleCPU_py",
)

gem5_cc_library(
    name = "noncaching",
    srcs = ["noncaching.cc"],
    hdrs = ["noncaching.hh"],
    deps = [
        ":atomic",
        ":base_non_caching_simple_cpu_sim_object",
        "//src/arch/generic:decoder",
        "//src/base:addr_range_map",
        "//src/mem:backdoor",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "base_non_caching_simple_cpu_sim_object",
    extra_param_deps = [
        ":base_atomic_simple_cpu_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:BaseAtomicSimpleCPU",
        "//src/python/m5/objects:BaseSimpleCPU",
    ],
    sim_object_header = ":noncaching",
    sim_object_module = "src.cpu.simple.BaseNonCachingSimpleCPU",
    sim_object_name = "BaseNonCachingSimpleCPU",
    sim_object_py_lib = ":BaseNonCachingSimpleCPU_py",
)
