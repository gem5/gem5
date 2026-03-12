load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "cpu_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseCPU_py",
        ":CPUTracers_py",
        ":FuncUnit_py",
        ":StaticInstFlags_py",
        "//src/cpu/checker:checker_init",
    ] + select({
        "//bazel:macos": [],
        "//conditions:default": ["//src/cpu/kvm:kvm_init"],
    }) + [
        "//src/cpu/minor:minor_init",
        "//src/cpu/o3:o3_init",
        "//src/cpu/pred:pred_init",
        "//src/cpu/probes:probes_init",
        ":TimingExpr_py",
        "//src/cpu/simple:simple_init",
        "//src/cpu/testers:testers_init",
        "//src/cpu/trace:trace_init",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "smt",
    hdrs = ["smt.hh"],
)

gem5_cc_library(
    name = "inst_seq",
    hdrs = ["inst_seq.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_debug_flag(
    name = "debug_PCEvent",
    flag = "PCEvent",
)

gem5_debug_flag(
    name = "debug_InvalidReg",
    flag = "InvalidReg",
)

gem5_debug_flag(
    name = "debug_Quiesce",
    flag = "Quiesce",
)

gem5_debug_flag(
    name = "debug_Mwait",
    flag = "Mwait",
)

gem5_debug_flag(
    name = "debug_Context",
    flag = "Context",
)

gem5_debug_flag(
    name = "debug_Activity",
    flag = "Activity",
)

gem5_debug_flag(
    name = "debug_Commit",
    flag = "Commit",
)

gem5_debug_flag(
    name = "debug_Decode",
    flag = "Decode",
)

gem5_debug_flag(
    name = "debug_DynInst",
    flag = "DynInst",
)

gem5_debug_flag(
    name = "debug_ExecAsid",
    desc = "Include ASID in trace",
    flag = "ExecAsid",
    fmt = True,
    included_by = [":debug_ExecAll"],
)

gem5_debug_flag(
    name = "debug_ExecCPSeq",
    desc = "Instruction sequence number",
    flag = "ExecCPSeq",
    fmt = True,
    included_by = [":debug_ExecAll"],
)

gem5_debug_flag(
    name = "debug_ExecEffAddr",
    desc = "Include effective address",
    flag = "ExecEffAddr",
    fmt = True,
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecFaulting",
    desc = "Trace faulting instructions",
    flag = "ExecFaulting",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecFetchSeq",
    desc = "Fetch sequence number",
    flag = "ExecFetchSeq",
    fmt = True,
    included_by = [":debug_ExecAll"],
)

gem5_debug_flag(
    name = "debug_ExecFlags",
    desc = "Include instruction flags in trace",
    flag = "ExecFlags",
    fmt = True,
    included_by = [":debug_ExecAll"],
)

gem5_debug_flag(
    name = "debug_ExecKernel",
    desc = "Filter: Trace kernel mode instructions",
    flag = "ExecKernel",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecMacro",
    desc = "Filter: Include macroops",
    flag = "ExecMacro",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecMicro",
    desc = "Filter: Include microops",
    flag = "ExecMicro",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecOpClass",
    desc = "Include operand class",
    flag = "ExecOpClass",
    fmt = True,
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecRegDelta",
    flag = "ExecRegDelta",
    included_by = [":debug_ExecAll"],
)

gem5_debug_flag(
    name = "debug_ExecResult",
    desc = "Include results from execution",
    flag = "ExecResult",
    fmt = True,
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecSymbol",
    desc = "Try to include symbol names",
    flag = "ExecSymbol",
    fmt = True,
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecThread",
    desc = "Include thread ID in trace",
    flag = "ExecThread",
    fmt = True,
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_ExecUser",
    desc = "Filter: Trace user mode instructions",
    flag = "ExecUser",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_Fetch",
    flag = "Fetch",
)

gem5_debug_flag(
    name = "debug_HtmCpu",
    desc = "Hardware Transactional Memory (CPU side)",
    flag = "HtmCpu",
)

gem5_debug_flag(
    name = "debug_O3PipeView",
    flag = "O3PipeView",
)

gem5_debug_flag(
    name = "debug_ExecAll",
    components = [
        ":debug_ExecEnable",
        ":debug_ExecCPSeq",
        ":debug_ExecEffAddr",
        ":debug_ExecFaulting",
        ":debug_ExecFetchSeq",
        ":debug_ExecOpClass",
        ":debug_ExecRegDelta",
        ":debug_ExecResult",
        ":debug_ExecSymbol",
        ":debug_ExecThread",
        ":debug_ExecMicro",
        ":debug_ExecMacro",
        ":debug_ExecUser",
        ":debug_ExecKernel",
        ":debug_ExecAsid",
        ":debug_ExecFlags",
    ],
    flag = "ExecAll",
)

exec_debug_flag_components = [
    ":debug_ExecEnable",
    ":debug_ExecOpClass",
    ":debug_ExecThread",
    ":debug_ExecEffAddr",
    ":debug_ExecResult",
    ":debug_ExecSymbol",
    ":debug_ExecMicro",
    ":debug_ExecMacro",
    ":debug_ExecFaulting",
    ":debug_ExecUser",
    ":debug_ExecKernel",
]

gem5_debug_flag(
    name = "debug_Exec",
    components = exec_debug_flag_components,
    flag = "Exec",
)

gem5_debug_flag(
    name = "debug_ExecNoTicks",
    components = exec_debug_flag_components + [
        "//src/base:debug_FmtTicksOff",
    ],
    flag = "ExecNoTicks",
)

gem5_debug_flag(
    name = "debug_ExecEnable",
    flag = "ExecEnable",
    included_by = [
        ":debug_Exec",
        ":debug_ExecNoTicks",
        ":debug_ExecAll",
    ],
)

gem5_debug_flag(
    name = "debug_BAC",
    flag = "BAC",
)

gem5_debug_flag(
    name = "debug_Branch",
    flag = "Branch",
)

gem5_debug_flag(
    name = "debug_Checker",
    flag = "Checker",
)

gem5_debug_flag(
    name = "debug_CommitRate",
    flag = "CommitRate",
)

gem5_debug_flag(
    name = "debug_FreeList",
    flag = "FreeList",
)

gem5_debug_flag(
    name = "debug_FTQ",
    flag = "FTQ",
)

gem5_debug_flag(
    name = "debug_FUPool",
    flag = "FUPool",
)

gem5_debug_flag(
    name = "debug_IEW",
    flag = "IEW",
)

gem5_debug_flag(
    name = "debug_IQ",
    flag = "IQ",
)

gem5_debug_flag(
    name = "debug_LSQ",
    flag = "LSQ",
)

gem5_debug_flag(
    name = "debug_LSQUnit",
    flag = "LSQUnit",
)

gem5_debug_flag(
    name = "debug_MemDepUnit",
    flag = "MemDepUnit",
)

gem5_debug_flag(
    name = "debug_O3CPU",
    flag = "O3CPU",
)

gem5_debug_flag(
    name = "debug_RegFile",
    flag = "RegFile",
)

gem5_debug_flag(
    name = "debug_Rename",
    flag = "Rename",
)

gem5_debug_flag(
    name = "debug_ROB",
    flag = "ROB",
)

gem5_debug_flag(
    name = "debug_Scoreboard",
    flag = "Scoreboard",
)

gem5_debug_flag(
    name = "debug_StoreSet",
    flag = "StoreSet",
)

gem5_debug_flag(
    name = "debug_Writeback",
    flag = "Writeback",
)

gem5_cc_library(
    name = "activity",
    srcs = ["activity.cc"],
    hdrs = ["activity.hh"],
    deps = [
        ":debug_Activity",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:timebuf",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "pc_event_hh",
    hdrs = ["pc_event.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "pc_event",
    srcs = ["pc_event.cc"],
    hdrs = ["pc_event.hh"],
    deps = [
        ":debug_PCEvent",
        "//src/base:debug",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:cur_tick",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "reg_class",
    srcs = ["reg_class.cc"],
    hdrs = ["reg_class.hh"],
    deps = [
        ":debug_InvalidReg",
        "//src/arch/generic:vec_reg",
        "//src/base:cprintf",
        "//src/base:debug",
        "//src/base:intmath",
        "//src/base:types",
        "//src/sim:bufval",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "thread_context_hh",
    hdrs = ["thread_context.hh"],
    deps = [
        ":pc_event_hh",
        ":reg_class",
        "//src/arch/generic:htm",
        "//src/arch/generic:isa",
        "//src/arch/generic:pcstate",
        "//src/arch/generic:vec_pred_reg",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "thread_context",
    srcs = ["thread_context.cc"],
    hdrs = ["thread_context.hh"],
    deps = [
        ":base_cpu_sim_object",
        ":base_hh",
        ":debug_Context",
        ":debug_Quiesce",
        ":pc_event",
        ":reg_class",
        ":thread_context_hh",
        "//src/arch/generic:htm",
        "//src/arch/generic:isa",
        "//src/arch/generic:pcstate",
        "//src/arch/generic:vec_pred_reg",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:port",
        "//src/sim:full_system",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "inst_res",
    hdrs = ["inst_res.hh"],
    deps = [
        ":reg_class",
        "//src/base:logging",
        "//src/base:types",
    ],
)

py_library(
    name = "StaticInstFlags_py",
    srcs = ["StaticInstFlags.py"],
    deps = [
    ],
)

gem5_cc_library(
    name = "timing_expr",
    srcs = ["timing_expr.cc"],
    hdrs = [
        "timing_expr.hh",
        "timing_expr_bin.hh",
        "timing_expr_if.hh",
        "timing_expr_let.hh",
        "timing_expr_literal.hh",
        "timing_expr_ref.hh",
        "timing_expr_src_reg.hh",
        "timing_expr_un.hh",
    ],
    deps = [
        ":enum_TimingExprOp_hh",
        ":param_timing_expr_hh",
        ":param_timing_expr_bin_hh",
        ":param_timing_expr_if_hh",
        ":param_timing_expr_let_hh",
        ":param_timing_expr_literal_hh",
        ":param_timing_expr_ref_hh",
        ":param_timing_expr_src_reg_hh",
        ":param_timing_expr_un_hh",
        ":static_inst",
        ":thread_context",
        "//src/base:intmath",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "TimingExpr_py",
    srcs = ["TimingExpr.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_TimingExprOp",
    enum_module = "src.cpu.TimingExpr",
    enum_name = "TimingExprOp",
    enum_py_lib = ":TimingExpr_py",
)

gem5_sim_object(
    name = "timing_expr_sim_object",
    sim_object_header = "timing_expr.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExpr",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_literal_sim_object",
    sim_object_header = "timing_expr_literal.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprLiteral",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_src_reg_sim_object",
    sim_object_header = "timing_expr_src_reg.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprSrcReg",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_let_sim_object",
    sim_object_header = "timing_expr_let.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprLet",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_ref_sim_object",
    sim_object_header = "timing_expr_ref.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprRef",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_un_sim_object",
    sim_object_header = "timing_expr_un.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprUn",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":enum_TimingExprOp_hh",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_bin_sim_object",
    sim_object_header = "timing_expr_bin.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprBin",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":enum_TimingExprOp_hh",
        ":timing_expr_sim_object",
    ],
)

gem5_sim_object(
    name = "timing_expr_if_sim_object",
    sim_object_header = "timing_expr_if.hh",
    sim_object_module = "src.cpu.TimingExpr",
    sim_object_name = "TimingExprIf",
    sim_object_py_lib = ":TimingExpr_py",
    extra_deps = [
        ":timing_expr",
    ],
    extra_param_deps = [
        ":static_inst",
        ":timing_expr_sim_object",
    ],
)

gem5_enum_generator(
    name = "enum_StaticInstFlags",
    enum_module = "src.cpu.StaticInstFlags",
    enum_name = "StaticInstFlags",
    enum_py_lib = ":StaticInstFlags_py",
)

gem5_cc_library(
    name = "static_inst",
    srcs = ["static_inst.cc"],
    hdrs = ["static_inst.hh"],
    deps = [
        ":enum_StaticInstFlags_hh",
        ":op_class",
        ":reg_class",
        ":static_inst_fwd",
        ":thread_context_hh",
        "//src/arch/generic:pcstate",
        "//src/base:logging",
        "//src/base:refcnt",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "static_inst_fwd",
    hdrs = ["static_inst_fwd.hh"],
    deps = [
        "//src/base:refcnt",
    ],
)

gem5_cc_library(
    name = "null_static_inst",
    srcs = ["null_static_inst.cc"],
    hdrs = ["null_static_inst.hh"],
    deps = [
        ":static_inst",
        ":static_inst_fwd",
    ],
)

gem5_cc_library(
    name = "nop_static_inst",
    srcs = ["nop_static_inst.cc"],
    hdrs = ["nop_static_inst.hh"],
    deps = [
        ":static_inst",
        ":static_inst_fwd",
        "//src/arch/generic:pcstate",
    ],
)

py_library(
    name = "FuncUnit_py",
    srcs = ["FuncUnit.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_OpClass",
    enum_module = "src.cpu.FuncUnit",
    enum_name = "OpClass",
    enum_py_lib = ":FuncUnit_py",
)

gem5_sim_object(
    name = "fu_desc_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":op_desc_sim_object",
    ],
    extra_tool_deps = [],
    sim_object_header = ":func_unit",
    sim_object_module = "src.cpu.FuncUnit",
    sim_object_name = "FUDesc",
    sim_object_py_lib = ":FuncUnit_py",
)

gem5_sim_object(
    name = "op_desc_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":enum_OpClass_hh",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [],
    sim_object_header = ":func_unit",
    sim_object_module = "src.cpu.FuncUnit",
    sim_object_name = "OpDesc",
    sim_object_py_lib = ":FuncUnit_py",
)

gem5_cc_library(
    name = "func_unit",
    srcs = ["func_unit.cc"],
    hdrs = ["func_unit.hh"],
    deps = [
        ":enum_OpClass_hh",
        ":fu_desc_sim_object",
        ":op_class",
        ":op_desc_sim_object",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "op_class",
    hdrs = ["op_class.hh"],
    deps = [
        ":enum_OpClass_hh",
        "//src/base:types",
    ],
)

py_library(
    name = "CPUTracers_py",
    srcs = ["CPUTracers.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:InstTracer",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "exe_tracer_sim_object",
    extra_deps = [":exetrace"],
    extra_param_deps = [
        "//src/sim:inst_tracer_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:InstTracer",
    ],
    sim_object_header = ":exetrace_hh",
    sim_object_module = "src.cpu.CPUTracers",
    sim_object_name = "ExeTracer",
    sim_object_py_lib = ":CPUTracers_py",
)

gem5_cc_library(
    name = "exetrace_hh",
    hdrs = [
        "exetrace.hh",
    ],
    deps = [
        ":debug_ExecEnable",
        ":exe_tracer_sim_object",
        ":static_inst",
        ":thread_context_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:insttracer",
    ],
)

gem5_cc_library(
    name = "exetrace",
    srcs = ["exetrace.cc"],
    deps = [
        ":base_hh",
        ":debug_ExecAsid",
        ":debug_ExecCPSeq",
        ":debug_ExecEffAddr",
        ":debug_ExecEnable",
        ":debug_ExecFaulting",
        ":debug_ExecFetchSeq",
        ":debug_ExecFlags",
        ":debug_ExecKernel",
        ":debug_ExecMacro",
        ":debug_ExecMicro",
        ":debug_ExecOpClass",
        ":debug_ExecResult",
        ":debug_ExecSymbol",
        ":debug_ExecThread",
        ":debug_ExecUser",
        ":enum_OpClass_hh",
        ":exe_tracer_sim_object",
        ":exetrace_hh",
        ":static_inst",
        ":static_inst_fwd",
        ":thread_context_hh",
        "//src/arch/generic:mmu_hh",
        "//src/arch/generic:pcstate",
        "//src/base:cprintf",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/loader:symtab",
        "//src/sim:full_system",
        "//src/sim:insttracer",
    ],
)

gem5_sim_object(
    name = "intel_trace_sim_object",
    extra_param_deps = [
        "//src/sim:inst_tracer_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:InstTracer",
    ],
    sim_object_header = ":inteltrace",
    sim_object_module = "src.cpu.CPUTracers",
    sim_object_name = "IntelTrace",
    sim_object_py_lib = ":CPUTracers_py",
)

gem5_cc_library(
    name = "inteltrace",
    srcs = ["inteltrace.cc"],
    hdrs = ["inteltrace.hh"],
    deps = [
        ":debug_ExecEnable",
        ":exetrace",
        ":exetrace_hh",
        ":intel_trace_sim_object",
        ":static_inst",
        ":thread_context",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:insttracer",
    ],
)

gem5_sim_object(
    name = "native_trace_sim_object",
    extra_param_deps = [
        ":exe_tracer_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:InstTracer",
    ],
    sim_object_header = ":nativetrace",
    sim_object_module = "src.cpu.CPUTracers",
    sim_object_name = "NativeTrace",
    sim_object_py_lib = ":CPUTracers_py",
)

gem5_cc_library(
    name = "nativetrace",
    srcs = ["nativetrace.cc"],
    hdrs = ["nativetrace.hh"],
    deps = [
        ":exetrace",
        ":exetrace_hh",
        ":native_trace_sim_object",
        ":static_inst",
        "//src/base:debug_GDBMisc",
        "//src/base:socket",
        "//src/base:trace",
        "//src/base:types",
    ],
)

py_library(
    name = "BaseCPU_py",
    srcs = ["BaseCPU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5/objects:CPUTracers",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:InstTracer",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "base_cpu_sim_object",
    extra_deps = [
    ],
    extra_param_deps = [
        "//src/arch/generic:inst_decoder_sim_object",
        "//src/arch/generic:base_interrupts_sim_object",
        "//src/arch/generic:base_mmu_sim_object",
        "//src/arch/generic:base_isa_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:inst_tracer_sim_object",
        "//src/sim:process_sim_object",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
        "//src/sim:process_hh",
        "//src/sim:insttracer",
        "//src/arch/generic:decoder",
        "//src/arch/generic:interrupts",
        "//src/arch/generic:isa",
        "//src/arch/generic:mmu",
    ],
    extra_tool_deps = [
        "//src/python/m5:defines",
        "//src/python/m5/params",
        "//src/python/m5/objects:BaseISA",
        "//src/python/m5/objects:BaseInterrupts",
        "//src/python/m5/objects:BaseMMU",
        "//src/python/m5/objects:CPUTracers",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:InstDecoder",
        "//src/python/m5/objects:InstTracer",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/objects:Process",
        "//src/python/m5/util:fdthelper",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.cpu.BaseCPU",
    sim_object_name = "BaseCPU",
    sim_object_py_lib = ":BaseCPU_py",
)

gem5_cc_library(
    name = "base_hh",
    hdrs = ["base.hh"],
    deps = [
        ":debug_Mwait",
        "//src/arch/generic:interrupts",
        "//src/arch/generic:mmu",
        "//src/arch/generic:tlb",
        "//src/base:statistics_hh",
        "//src/dev:intpin",
        "//src/mem:htm_hh",
        "//src/mem:port_proxy",
        "//src/sim:clocked_object_hh",
        "//src/sim:eventq",
        "//src/sim:full_system",
        "//src/sim:insttracer",
        "//src/sim:signal",
        "//src/sim/probe",
        "//src/sim/probe:pmu",
    ],
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_cpu_sim_object",
        ":debug_Mwait",
        ":enum_OpClass_py",
        ":simple_thread",
        ":thread_context_hh",
        "//src/arch/generic:decoder",
        "//src/arch/generic:interrupts",
        "//src/arch/generic:isa",
        "//src/arch/generic:mmu",
        "//src/arch/generic:tlb",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base/loader:symtab",
        "//src/cpu/checker:cpu",
        "//src/dev:intpin",
        "//src/mem:htm_hh",
        "//src/mem:page_table",
        "//src/mem:port_proxy",
        "//src/sim:clocked_object_hh",
        "//src/sim:debug_SyscallVerbose",
        "//src/sim:debug_Thread",
        "//src/sim:eventq",
        "//src/sim:faults",
        "//src/sim:full_system",
        "//src/sim:futex_map",
        "//src/sim:insttracer",
        "//src/sim:process_hh",
        "//src/sim:root_hh",
        "//src/sim:signal",
        "//src/sim:sim_events",
        "//src/sim:sim_exit",
        "//src/sim:stat_control",
        "//src/sim:system_hh",
        "//src/sim/probe",
        "//src/sim/probe:pmu",
    ],
)

gem5_cc_library(
    name = "utils",
    hdrs = ["utils.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "simple_thread_hh",
    hdrs = ["simple_thread.hh"],
    deps = [
        ":reg_class",
        ":regfile",
        ":thread_context_hh",
        ":thread_state",
        "//src/arch:debug_CCRegs",
        "//src/arch:debug_FloatRegs",
        "//src/arch:debug_IntRegs",
        "//src/arch:debug_MatRegs",
        "//src/arch:debug_VecPredRegs",
        "//src/arch:debug_VecRegs",
        "//src/arch/generic:htm",
        "//src/arch/generic:mmu",
        "//src/arch/generic:pcstate",
        "//src/arch/generic:tlb",
        "//src/base:logging",
        "//src/base:types",
        "//src/mem:htm_hh",
        "//src/mem:page_table",
        "//src/mem:request",
        "//src/sim:byteswap",
        "//src/sim:eventq",
        "//src/sim:full_system",
        "//src/sim:process_hh",
        "//src/sim:serialize",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "simple_thread",
    srcs = ["simple_thread.cc"],
    hdrs = ["simple_thread.hh"],
    deps = [
        ":base_cpu_sim_object",
        ":base_hh",
        ":reg_class",
        ":regfile",
        ":simple_thread_hh",
        ":thread_context",
        ":thread_state",
        "//src/arch:debug_CCRegs",
        "//src/arch:debug_FloatRegs",
        "//src/arch:debug_IntRegs",
        "//src/arch:debug_MatRegs",
        "//src/arch:debug_VecPredRegs",
        "//src/arch:debug_VecRegs",
        "//src/arch/generic:decoder",
        "//src/arch/generic:htm",
        "//src/arch/generic:mmu",
        "//src/arch/generic:pcstate",
        "//src/arch/generic:tlb",
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:htm_hh",
        "//src/mem:page_table",
        "//src/mem:request",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:byteswap",
        "//src/sim:eventq",
        "//src/sim:faults",
        "//src/sim:full_system",
        "//src/sim:process",
        "//src/sim:process_hh",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "regfile",
    hdrs = ["regfile.hh"],
    deps = [
        ":reg_class",
    ],
)

gem5_cc_library(
    name = "thread_state_hh",
    hdrs = ["thread_state.hh"],
    deps = [
        ":base_hh",
        ":thread_context_hh",
        "//src/base:output",
        "//src/mem:port",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:full_system",
        "//src/sim:process_hh",
        "//src/sim:serialize",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "thread_state",
    srcs = ["thread_state.cc"],
    hdrs = ["thread_state.hh"],
    deps = [
        ":base_hh",
        ":thread_context_hh",
        "//src/base:output",
        "//src/mem:port",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:full_system",
        "//src/sim:process_hh",
        "//src/sim:serialize",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "exec_context",
    hdrs = ["exec_context.hh"],
    deps = [
        ":base_hh",
        ":reg_class",
        ":static_inst_fwd",
        ":thread_context_hh",
        ":translation",
        "//src/base:types",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "translation",
    hdrs = ["translation.hh"],
    deps = [
        "//src/arch/generic:mmu",
        "//src/arch/generic:tlb",
        "//src/sim:faults_hh",
    ],
)

gem5_cc_library(
    name = "decode_cache",
    hdrs = ["decode_cache.hh"],
    deps = [
        ":static_inst_fwd",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:types",
    ],
)


gem5_cc_library(
    name = "timebuf",
    hdrs = ["timebuf.hh"],
    deps = [
        "//src/base:types",
    ],
)

py_library(
    name = "DummyChecker_py",
    srcs = ["DummyChecker.py"],
    deps = [
        "//src/cpu/checker:CheckerCPU_py",
        "//src/python/m5/objects:CheckerCPU",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "dummy_checker_sim_object",
    extra_param_deps = [
        "//src/cpu/checker:checker_cpu_sim_object",
    ],
    sim_object_header = ":dummy_checker_hh",
    sim_object_module = "src.cpu.DummyChecker",
    sim_object_name = "DummyChecker",
    sim_object_py_lib = ":DummyChecker_py",
)

gem5_cc_library(
    name = "dummy_checker_hh",
    hdrs = ["dummy_checker.hh"],
    deps = [
        "//src/cpu/checker:cpu_hh",
    ],
)
