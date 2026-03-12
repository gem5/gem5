load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "o3_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseO3CPU_py",
        ":BaseO3Checker_py",
        ":FUPool_py",
        ":FuncUnitConfig_py",
        ":IQUnit_py",
        ":SMT_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

# Python Libraries
py_library(
    name = "BaseO3CPU_py",
    srcs = ["BaseO3CPU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:citations",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
        "//src/python/m5/objects:FUPool",
        "//src/python/m5/objects:IQUnit",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:SMT",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "SMT_py",
    srcs = ["SMT.py"],
    deps = [
        "//src/python/m5/params",
    ],
)

py_library(
    name = "IQUnit_py",
    srcs = ["IQUnit.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:FUPool",
        "//src/python/m5/objects:SMT",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "BaseO3Checker_py",
    srcs = ["BaseO3Checker.py"],
    deps = [
        "//src/python/m5/objects:CheckerCPU",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "FuncUnitConfig_py",
    srcs = ["FuncUnitConfig.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5/objects:FuncUnit",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "FUPool_py",
    srcs = ["FUPool.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:FuncUnit",
        "//src/python/m5/objects:FuncUnitConfig",
        "//src/python/m5/params",
    ],
)

# Enum Generators
gem5_enum_generator(
    name = "enum_SMTFetchPolicy",
    enum_module = "src.cpu.o3.SMT",
    enum_name = "SMTFetchPolicy",
    enum_py_lib = ":SMT_py",
    extra_tool_deps = [],
)

gem5_enum_generator(
    name = "enum_SMTQueuePolicy",
    enum_module = "src.cpu.o3.SMT",
    enum_name = "SMTQueuePolicy",
    enum_py_lib = ":SMT_py",
    extra_tool_deps = [],
)

gem5_enum_generator(
    name = "enum_CommitPolicy",
    enum_module = "src.cpu.o3.SMT",
    enum_name = "CommitPolicy",
    enum_py_lib = ":SMT_py",
    extra_tool_deps = [],
)

# C++ Libraries

# Header-only libraries are used to break circular dependencies
# between various O3 CPU components (e.g. DynInst <-> LSQUnit).
gem5_cc_library(
    name = "comm_hh",
    hdrs = ["comm.hh"],
    deps = [
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        "//src/arch/generic:pcstate",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/sim:faults",
    ],
)

gem5_cc_library(
    name = "limits_hh",
    hdrs = ["limits.hh"],
)

gem5_cc_library(
    name = "dyn_inst_ptr_hh",
    hdrs = ["dyn_inst_ptr.hh"],
    deps = ["//src/base:refcnt"],
)

gem5_cc_library(
    name = "dyn_inst_hh",
    hdrs = [
        "dyn_inst.hh",
    ],
    deps = [
        ":dyn_inst_ptr_hh",
        ":lsq_unit_hh",
        "//src/base:circular_queue",
        "//src/base:refcnt",
        "//src/base:trace",
        "//src/cpu:base_hh",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:exec_context",
        "//src/cpu:inst_res",
        "//src/cpu:inst_seq",
        "//src/cpu:op_class",
        "//src/cpu:reg_class",
        "//src/cpu:static_inst",
        "//src/cpu:translation",
    ],
)

gem5_cc_library(
    name = "thread_context_hh",
    hdrs = ["thread_context.hh"],
    deps = [
        ":cpu_hh",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "bac_hh",
    hdrs = ["bac.hh"],
    deps = [
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        "//src/base:statistics",
        "//src/cpu:timebuf",
        "//src/cpu/pred:bpred_unit",
        "//src/cpu/pred:branch_type",
    ],
)

gem5_cc_library(
    name = "checker_hh",
    hdrs = ["checker.hh"],
    deps = [
        ":dyn_inst_hh",
        "//src/cpu/checker:cpu",
    ],
)

gem5_cc_library(
    name = "checker",
    srcs = ["checker.cc"],
    deps = [
        ":checker_hh",
        ":dyn_inst_ptr_hh",
        ":thread_state_hh",
        "//src/cpu/checker:cpu_hh",
        "//src/cpu/checker:cpu_impl",
    ],
)

gem5_cc_library(
    name = "commit_hh",
    hdrs = ["commit.hh"],
    deps = [
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":enum_CommitPolicy_hh",
        ":iew_hh",
        ":limits_hh",
        ":rename_map_hh",
        ":rob_hh",
        "//src/base:statistics",
        "//src/cpu:exetrace_hh",
        "//src/cpu:inst_seq",
        "//src/cpu:timebuf",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "cpu_hh",
    hdrs = ["cpu.hh"],
    deps = [
        ":bac_hh",
        ":base_o3_cpu_sim_object",
        ":comm_hh",
        ":commit_hh",
        ":decode_hh",
        ":dyn_inst_ptr_hh",
        ":fetch_hh",
        ":free_list_hh",
        ":ftq_hh",
        ":iew_hh",
        ":limits_hh",
        ":rename_hh",
        ":rob_hh",
        ":scoreboard_hh",
        ":thread_state_hh",
        "//src/arch/generic:pcstate",
        "//src/base:statistics",
        "//src/cpu:activity",
        "//src/cpu:base_hh",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:timebuf",
        "//src/sim:process_hh",
    ],
)

gem5_cc_library(
    name = "decode_hh",
    hdrs = ["decode.hh"],
    deps = [
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        "//src/base:statistics",
        "//src/cpu:timebuf",
    ],
)

gem5_cc_library(
    name = "dep_graph_hh",
    hdrs = ["dep_graph.hh"],
    deps = [":comm_hh"],
)

gem5_cc_library(
    name = "fetch_hh",
    hdrs = ["fetch.hh"],
    deps = [
        ":bac_hh",
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":enum_SMTFetchPolicy_hh",
        ":ftq_hh",
        ":limits_hh",
        "//src/arch/generic:decoder",
        "//src/arch/generic:mmu",
        "//src/base:random",
        "//src/base:statistics",
        "//src/cpu:pc_event",
        "//src/cpu:timebuf",
        "//src/cpu:translation",
        "//src/cpu/pred:bpred_unit",
        "//src/mem:packet_hh",
        "//src/mem:port",
        "//src/sim:eventq",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "free_list_hh",
    hdrs = ["free_list.hh"],
    deps = [
        ":comm_hh",
        ":regfile_hh",
        "//src/base:logging",
        "//src/base:trace",
        "//src/cpu:debug_FreeList",
    ],
)

gem5_cc_library(
    name = "ftq_hh",
    hdrs = ["ftq.hh"],
    deps = [
        ":limits_hh",
        "//src/arch/generic:pcstate",
        "//src/base:statistics",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/cpu/pred:bpred_unit",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "fu_pool_hh",
    hdrs = ["fu_pool.hh"],
    deps = [
        ":fu_pool_sim_object",
        "//src/cpu:op_class",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "fu_pool",
    srcs = ["fu_pool.cc"],
    deps = [
        ":fu_pool_hh",
        "//src/cpu:func_unit",
    ],
)

gem5_cc_library(
    name = "iew_hh",
    hdrs = ["iew.hh"],
    deps = [
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":inst_queue_hh",
        ":limits_hh",
        ":lsq_hh",
        ":scoreboard_hh",
        "//src/base:statistics",
        "//src/cpu:debug_IEW",
        "//src/cpu:timebuf",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "inst_queue_hh",
    hdrs = ["inst_queue.hh"],
    deps = [
        ":comm_hh",
        ":dep_graph_hh",
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        ":limits_hh",
        ":mem_dep_unit_hh",
        ":store_set_hh",
        "//src/base:statistics",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/cpu:op_class",
        "//src/cpu:timebuf",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "lsq_hh",
    hdrs = ["lsq.hh"],
    deps = [
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        "//src/arch/generic:mmu",
        "//src/arch/generic:tlb",
        "//src/base:flags",
        "//src/base:statistics",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/cpu:utils",
        "//src/mem:port",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "lsq_unit_hh",
    hdrs = ["lsq_unit.hh"],
    deps = [
        ":comm_hh",
        ":dyn_inst_ptr_hh",
        ":lsq_hh",
        "//src/arch/generic:debugfaults",
        "//src/arch/generic:vec_reg",
        "//src/base:circular_queue",
        "//src/cpu:base_hh",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:debug_LSQUnit",
        "//src/cpu:inst_seq",
        "//src/cpu:timebuf",
        "//src/mem:packet_hh",
        "//src/mem:port",
    ],
)

gem5_cc_library(
    name = "mem_dep_unit_hh",
    hdrs = ["mem_dep_unit.hh"],
    deps = [
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        ":store_set_hh",
        "//src/base:statistics",
        "//src/cpu:debug_MemDepUnit",
        "//src/cpu:inst_seq",
    ],
)

gem5_cc_library(
    name = "regfile_hh",
    hdrs = ["regfile.hh"],
    deps = [
        ":comm_hh",
        "//src/arch/generic:isa",
        "//src/base:trace",
        "//src/cpu:debug_IEW",
        "//src/cpu:regfile",
    ],
)

gem5_cc_library(
    name = "rename_map_hh",
    hdrs = ["rename_map.hh"],
    deps = [
        ":dyn_inst_ptr_hh",
        ":free_list_hh",
        ":regfile_hh",
        "//src/arch/generic:isa",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "rename_hh",
    hdrs = ["rename.hh"],
    deps = [
        ":comm_hh",
        ":commit_hh",
        ":dyn_inst_ptr_hh",
        ":free_list_hh",
        ":iew_hh",
        ":limits_hh",
        "//src/base:statistics",
        "//src/cpu:timebuf",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "rob_hh",
    hdrs = ["rob.hh"],
    deps = [
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        ":limits_hh",
        "//src/base:statistics",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "scoreboard_hh",
    hdrs = ["scoreboard.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/cpu:debug_Scoreboard",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "store_set_hh",
    hdrs = ["store_set.hh"],
    deps = [
        "//src/base:named",
        "//src/base:types",
        "//src/base/cache:associative_cache",
        "//src/base/cache:cache_entry",
        "//src/cpu:inst_seq",
    ],
)

gem5_cc_library(
    name = "thread_state_hh",
    hdrs = ["thread_state.hh"],
    deps = [
        "//src/cpu:thread_context_hh",
        "//src/cpu:thread_state_hh",
    ],
)

gem5_cc_library(
    name = "bac",
    srcs = ["bac.cc"],
    deps = [
        ":bac_hh",
        ":base_o3_cpu_sim_object",
        ":comm_hh",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":ftq_hh",
        ":limits_hh",
        "//src/arch/generic:pcstate",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:units",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_BAC",
        "//src/cpu:debug_FTQ",
        "//src/cpu:debug_Fetch",
        "//src/cpu:inst_seq",
        "//src/cpu:static_inst_fwd",
        "//src/cpu:timebuf",
        "//src/cpu/pred:bpred_unit",
        "//src/cpu/pred:branch_type",
        "//src/cpu/pred:debug_Branch",
        "//src/cpu/pred:enum_BranchType_hh",
        "//src/sim:debug_Drain",
    ],
)

gem5_cc_library(
    name = "commit",
    srcs = ["commit.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":checker",
        ":commit_hh",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":limits_hh",
        ":thread_state_hh",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base/loader:symtab",
        "//src/cpu:base_hh",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_Commit",
        "//src/cpu:debug_CommitRate",
        "//src/cpu:debug_ExecFaulting",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:exetrace_hh",
        "//src/cpu:timebuf",
        "//src/cpu/checker:cpu_hh",
        "//src/sim:debug_Drain",
        "//src/sim:faults_hh",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "cpu",
    srcs = ["cpu.cc"],
    deps = [
        ":checker",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":limits_hh",
        ":thread_context_hh",
        "//src/cpu:activity",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_O3CPU",
        "//src/cpu:debug_Quiesce",
        "//src/cpu:simple_thread_hh",
        "//src/cpu:thread_context_hh",
        "//src/cpu/checker:cpu_hh",
        "//src/cpu/checker:thread_context_hh",
        "//src/sim:cur_tick",
        "//src/sim:debug_Drain",
        "//src/sim:enum_MemoryMode_hh",
        "//src/sim:full_system",
        "//src/sim:process_hh",
        "//src/sim:stat_control",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "decode",
    srcs = ["decode.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":comm_hh",
        ":cpu_hh",
        ":decode_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        "//src/arch/generic:pcstate",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:units",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_Decode",
        "//src/cpu:inst_seq",
        "//src/cpu:timebuf",
        "//src/sim:cur_tick",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "dyn_inst",
    srcs = ["dyn_inst.cc"],
    deps = [
        ":checker",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":inst_queue_hh",
        ":lsq_unit_hh",
        ":thread_state_hh",
        "//src/arch/generic:pcstate",
        "//src/base:amo",
        "//src/base:cprintf",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base_hh",
        "//src/cpu:debug_DynInst",
        "//src/cpu:debug_IQ",
        "//src/cpu:debug_O3PipeView",
        "//src/cpu:exetrace_hh",
        "//src/cpu:inst_seq",
        "//src/cpu:reg_class",
        "//src/cpu:static_inst",
        "//src/cpu:static_inst_fwd",
        "//src/cpu:thread_context_hh",
        "//src/cpu/checker:cpu_hh",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "fetch",
    srcs = ["fetch.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":fetch_hh",
        ":limits_hh",
        "//src/arch/generic:tlb",
        "//src/base:types",
        "//src/cpu:base_hh",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_Fetch",
        "//src/cpu:debug_O3CPU",
        "//src/cpu:exetrace_hh",
        "//src/cpu:nop_static_inst",
        "//src/mem:packet_hh",
        "//src/sim:byteswap",
        "//src/sim:core",
        "//src/sim:debug_Drain",
        "//src/sim:eventq",
        "//src/sim:full_system",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "free_list",
    srcs = ["free_list.cc"],
    deps = [
        ":free_list_hh",
        "//src/base:trace",
        "//src/cpu:debug_FreeList",
    ],
)

gem5_cc_library(
    name = "ftq",
    srcs = ["ftq.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":ftq_hh",
        "//src/arch/generic:pcstate",
        "//src/base:logging",
        "//src/cpu:debug_FTQ",
    ],
)

gem5_cc_library(
    name = "iew",
    srcs = ["iew.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":checker",
        ":comm_hh",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":fu_pool_hh",
        ":iew_hh",
        ":limits_hh",
        ":lsq_unit_hh",
        ":scoreboard_hh",
        "//src/arch/generic:pcstate",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:units",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_IEW",
        "//src/cpu:timebuf",
        "//src/cpu/checker:cpu_hh",
        "//src/sim:cur_tick",
        "//src/sim:debug_Drain",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "inst_queue",
    srcs = ["inst_queue.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":comm_hh",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        ":fu_pool_hh",
        ":iew_hh",
        ":inst_queue_hh",
        ":iq_unit_sim_object",
        ":limits_hh",
        "//src/arch/generic:isa",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:units",
        "//src/cpu:debug_IQ",
        "//src/cpu:enum_OpClass_hh",
        "//src/cpu:inst_seq",
        "//src/cpu:op_class",
        "//src/cpu:reg_class",
        "//src/cpu:timebuf",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "lsq",
    srcs = ["lsq.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        ":iew_hh",
        ":limits_hh",
        ":lsq_hh",
        ":lsq_unit_hh",
        "//src/arch/generic:mmu",
        "//src/base:amo",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:units",
        "//src/cpu:debug_Fetch",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:debug_LSQ",
        "//src/cpu:debug_Writeback",
        "//src/cpu:inst_seq",
        "//src/cpu:thread_context_hh",
        "//src/cpu:utils",
        "//src/mem:packet_hh",
        "//src/mem:port",
        "//src/mem:request",
        "//src/sim:cur_tick",
        "//src/sim:debug_Drain",
    ],
)

gem5_cc_library(
    name = "lsq_unit",
    srcs = ["lsq_unit.cc"],
    deps = [
        ":checker",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":limits_hh",
        ":lsq_hh",
        ":lsq_unit_hh",
        "//src/arch/generic:debugfaults",
        "//src/arch/generic:mmu",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:units",
        "//src/cpu:debug_HtmCpu",
        "//src/cpu:debug_LSQUnit",
        "//src/cpu:inst_seq",
        "//src/cpu/checker:cpu_hh",
        "//src/mem:htm_hh",
        "//src/mem:packet_hh",
        "//src/mem:port",
        "//src/mem:request",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:faults_hh",
    ],
)

gem5_cc_library(
    name = "mem_dep_unit",
    srcs = ["mem_dep_unit.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":inst_queue_hh",
        ":limits_hh",
        ":mem_dep_unit_hh",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:units",
        "//src/cpu:debug_MemDepUnit",
        "//src/cpu:inst_seq",
    ],
)

gem5_cc_library(
    name = "regfile",
    srcs = ["regfile.cc"],
    deps = [
        ":free_list_hh",
        ":regfile_hh",
    ],
)

gem5_cc_library(
    name = "rename",
    srcs = ["rename.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":limits_hh",
        ":rename_hh",
        "//src/cpu:debug_Activity",
        "//src/cpu:debug_Rename",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "rename_map",
    srcs = ["rename_map.cc"],
    deps = [
        ":dyn_inst_hh",
        ":rename_map_hh",
        "//src/cpu:debug_Rename",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "rob",
    srcs = ["rob.cc"],
    deps = [
        ":base_o3_cpu_sim_object",
        ":cpu_hh",
        ":dyn_inst_hh",
        ":dyn_inst_ptr_hh",
        ":enum_SMTQueuePolicy_hh",
        ":limits_hh",
        ":rob_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/base/stats:units",
        "//src/cpu:debug_Fetch",
        "//src/cpu:debug_ROB",
        "//src/cpu:inst_seq",
    ],
)

gem5_cc_library(
    name = "scoreboard",
    srcs = ["scoreboard.cc"],
    deps = [
        ":scoreboard_hh",
    ],
)

gem5_cc_library(
    name = "store_set",
    srcs = ["store_set.cc"],
    deps = [
        ":store_set_hh",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/cpu:debug_StoreSet",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)

gem5_cc_library(
    name = "thread_context",
    srcs = ["thread_context.cc"],
    deps = [
        ":thread_context_hh",
        "//src/cpu:debug_O3CPU",
    ],
)

gem5_cc_library(
    name = "thread_state",
    srcs = ["thread_state.cc"],
    deps = [
        ":cpu_hh",
        ":thread_state_hh",
    ],
)

gem5_cc_library(
    name = "o3_impl",
    deps = [
        ":bac",
        ":commit",
        ":cpu",
        ":decode",
        ":dyn_inst",
        ":fetch",
        ":free_list",
        ":ftq",
        ":iew",
        ":inst_queue",
        ":lsq",
        ":lsq_unit",
        ":mem_dep_unit",
        ":regfile",
        ":rename",
        ":rename_map",
        ":rob",
        ":scoreboard",
        ":store_set",
        ":thread_context",
        ":thread_state",
    ],
)

# SimObject Generators
gem5_sim_object(
    name = "base_o3_cpu_sim_object",
    extra_deps = [
        ":o3_impl",
        ":dyn_inst_hh",
        ":fu_pool_hh",
        ":inst_queue_hh",
        "//src/cpu:base",
        "//src/cpu/pred:bpred_unit",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":enum_CommitPolicy_hh",
        ":enum_SMTFetchPolicy_hh",
        ":enum_SMTQueuePolicy_hh",
        ":fu_pool_sim_object",
        ":iq_unit_sim_object",
        "//src/cpu:base_cpu_sim_object",
        "//src/cpu/pred:branch_predictor_sim_object",
        "//src/mem/cache/tags/indexing_policies:base_indexing_policy_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        ":FUPool_py",
        ":IQUnit_py",
        ":SMT_py",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BranchPredictor",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:FUPool",
        "//src/python/m5/objects:FuncUnit",
        "//src/python/m5/objects:FuncUnitConfig",
        "//src/python/m5/objects:IQUnit",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:SMT",
        "//src/python/m5/params",
    ],
    sim_object_header = ":cpu_hh",
    sim_object_module = "src.cpu.o3.BaseO3CPU",
    sim_object_name = "BaseO3CPU",
    sim_object_py_lib = ":BaseO3CPU_py",
)

gem5_sim_object(
    name = "base_o3_checker_sim_object",
    extra_deps = [
        ":checker",
        "//src/cpu/checker:cpu",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/cpu/checker:checker_cpu_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:CheckerCPU",
        "//src/python/m5/objects:BaseCPU",
    ],
    sim_object_header = ":checker_hh",
    sim_object_module = "src.cpu.o3.BaseO3Checker",
    sim_object_name = "BaseO3Checker",
    sim_object_py_lib = ":BaseO3Checker_py",
)

gem5_sim_object(
    name = "fu_pool_sim_object",
    extra_deps = [
        ":fu_pool",
        "//src/cpu:func_unit",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/cpu:fu_desc_sim_object",
        "//src/cpu:op_desc_sim_object",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        ":FuncUnitConfig_py",
        "//src/cpu:FuncUnit_py",
        "//src/python/m5:defines",
        "//src/python/m5/params",
        "//src/python/m5/objects:FuncUnit",
        "//src/python/m5/objects:FuncUnitConfig",
    ],
    sim_object_header = ":fu_pool_hh",
    sim_object_module = "src.cpu.o3.FUPool",
    sim_object_name = "FUPool",
    sim_object_py_lib = ":FUPool_py",
)

gem5_sim_object(
    name = "iq_unit_sim_object",
    extra_deps = [
        ":fu_pool_hh",
        ":o3_impl",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":enum_SMTQueuePolicy_hh",
        ":fu_pool_sim_object",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        ":FUPool_py",
        ":SMT_py",
        "//src/python/m5/objects:FUPool",
        "//src/python/m5/objects:FuncUnit",
        "//src/python/m5/objects:FuncUnitConfig",
        "//src/python/m5/objects:SMT",
    ],
    sim_object_header = ":inst_queue_hh",
    sim_object_module = "src.cpu.o3.IQUnit",
    sim_object_name = "IQUnit",
    sim_object_py_lib = ":IQUnit_py",
)
