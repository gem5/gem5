load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "minor_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseMinorCPU_py",
        ":MinorCPU_py",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "BaseMinorCPU_py",
    srcs = ["BaseMinorCPU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:citations",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
        "//src/python/m5/objects:DummyChecker",
        "//src/python/m5/objects:FUPool",
        "//src/python/m5/objects:IQUnit",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:SMT",
        "//src/python/m5/objects:TimingExpr",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_minor_cpu_sim_object",
    sim_object_header = "cpu.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "BaseMinorCPU",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":enum_ThreadPolicy_hh",
        ":minor_hdrs",
        "//src/cpu:base_cpu_sim_object",
        "//src/cpu/pred:branch_predictor_sim_object",
        "//src/sim:sim_object",
        ":minor_fu_pool_sim_object",
        ":minor_fu_sim_object",
        ":minor_fu_timing_sim_object",
        ":minor_op_class_set_sim_object",
        ":minor_op_class_sim_object",
        "//src/cpu/pred:bpred_unit",
    ],
)

gem5_sim_object(
    name = "minor_op_class_sim_object",
    sim_object_header = "func_unit_op_class.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "MinorOpClass",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":minor_hdrs",
        "//src/sim:sim_object",
        "//src/cpu:enum_OpClass_hh",
        "//src/cpu:func_unit",
    ],
)

gem5_sim_object(
    name = "minor_op_class_set_sim_object",
    sim_object_header = "func_unit_op_class_set.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "MinorOpClassSet",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":minor_hdrs",
        ":minor_op_class_sim_object",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "minor_fu_timing_sim_object",
    sim_object_header = "func_unit_timing.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "MinorFUTiming",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":minor_hdrs",
        ":minor_op_class_set_sim_object",
        "//src/sim:sim_object",
        "//src/cpu:timing_expr_sim_object",
        "//src/cpu:func_unit",
    ],
)

gem5_sim_object(
    name = "minor_fu_sim_object",
    sim_object_header = "func_unit_fu.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "MinorFU",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":minor_hdrs",
        ":minor_op_class_set_sim_object",
        ":minor_fu_timing_sim_object",
        "//src/sim:sim_object",
        "//src/cpu:func_unit",
    ],
)

gem5_sim_object(
    name = "minor_fu_pool_sim_object",
    sim_object_header = "func_unit_pool.hh",
    sim_object_module = "src.cpu.minor.BaseMinorCPU",
    sim_object_name = "MinorFUPool",
    sim_object_py_lib = ":BaseMinorCPU_py",
    extra_param_deps = [
        ":minor_hdrs",
        ":minor_fu_sim_object",
        "//src/sim:sim_object",
        "//src/cpu:func_unit",
    ],
)

gem5_enum_generator(
    name = "enum_ThreadPolicy",
    enum_module = "src.cpu.minor.BaseMinorCPU",
    enum_name = "ThreadPolicy",
    enum_py_lib = ":BaseMinorCPU_py",
)
gem5_debug_flag(
    name = "debug_MinorCPU",
    flag = "MinorCPU",
    desc = "Minor CPU-level events",
    included_by = [":debug_Minor"],
)

gem5_debug_flag(
    name = "debug_MinorExecute",
    flag = "MinorExecute",
    desc = "Minor Execute stage",
    included_by = [":debug_Minor"],
)

gem5_debug_flag(
    name = "debug_MinorInterrupt",
    flag = "MinorInterrupt",
    desc = "Minor interrupt handling",
    included_by = [":debug_Minor"],
)

gem5_debug_flag(
    name = "debug_MinorMem",
    flag = "MinorMem",
    desc = "Minor memory accesses",
    included_by = [":debug_Minor"],
)

gem5_debug_flag(
    name = "debug_MinorScoreboard",
    flag = "MinorScoreboard",
    desc = "Minor Execute register scoreboard",
    included_by = [":debug_Minor"],
)

gem5_debug_flag(
    name = "debug_MinorTrace",
    flag = "MinorTrace",
    desc = "MinorTrace cycle-by-cycle state trace",
)

gem5_debug_flag(
    name = "debug_MinorTiming",
    flag = "MinorTiming",
    desc = "Extra timing for instructions",
)

gem5_debug_flag(
    name = "debug_Minor",
    flag = "Minor",
    desc = "Minor compound flag",
    components = [
        ":debug_MinorCPU",
        ":debug_MinorExecute",
        ":debug_MinorInterrupt",
        ":debug_MinorMem",
        ":debug_MinorScoreboard",
    ],
)

py_library(
    name = "MinorCPU_py",
    srcs = ["MinorCPU.py"],
    deps = [
        ":BaseMinorCPU_py",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "minor_hdrs",
    hdrs = [
        "activity.hh",
        "buffers.hh",
        "cpu.hh",
        "decode.hh",
        "dyn_inst.hh",
        "exec_context.hh",
        "execute.hh",
        "fetch1.hh",
        "fetch2.hh",
        "func_unit.hh",
        "func_unit_fu.hh",
        "func_unit_op_class.hh",
        "func_unit_op_class_set.hh",
        "func_unit_pool.hh",
        "func_unit_timing.hh",
        "lsq.hh",
        "pipe_data.hh",
        "pipeline.hh",
        "scoreboard.hh",
        "stats.hh",
        "trace.hh",
    ],
    deps = [
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:types",
        "//src/cpu:base",
        "//src/cpu:func_unit",
        "//src/cpu:thread_context",
        "//src/cpu:activity",
        "//src/cpu:timebuf",
        "//src/cpu:inst_seq",
        "//src/cpu:static_inst",
        "//src/cpu:timing_expr",
        "//src/sim:ticked_object",
        ":debug_MinorTrace",
    ],
)

gem5_cc_library(
    name = "minor",
    srcs = [
        "activity.cc",
        "cpu.cc",
        "decode.cc",
        "dyn_inst.cc",
        "execute.cc",
        "fetch1.cc",
        "fetch2.cc",
        "func_unit.cc",
        "lsq.cc",
        "pipe_data.cc",
        "pipeline.cc",
        "scoreboard.cc",
        "stats.cc",
    ],
    deps = [
        ":minor_hdrs",
        ":base_minor_cpu_sim_object",
        ":minor_fu_pool_sim_object",
        ":minor_fu_sim_object",
        ":minor_fu_timing_sim_object",
        ":minor_op_class_set_sim_object",
        ":minor_op_class_sim_object",
    ],
)
