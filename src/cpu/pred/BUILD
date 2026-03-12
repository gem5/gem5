load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "pred_init",
    srcs = ["__init__.py"],
    deps = [
        ":BranchPredictor_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_Indirect",
    flag = "Indirect",
)

gem5_debug_flag(
    name = "debug_BTB",
    flag = "BTB",
)

gem5_debug_flag(
    name = "debug_RAS",
    flag = "RAS",
)

gem5_debug_flag(
    name = "debug_FreeList",
    flag = "FreeList",
)

gem5_debug_flag(
    name = "debug_Branch",
    flag = "Branch",
)

gem5_debug_flag(
    name = "debug_Tage",
    flag = "Tage",
)

gem5_debug_flag(
    name = "debug_LTage",
    flag = "LTage",
)

gem5_debug_flag(
    name = "debug_TageSCL",
    flag = "TageSCL",
)

py_library(
    name = "BranchPredictor_py",
    srcs = ["BranchPredictor.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_BranchType",
    enum_module = "src.cpu.pred.BranchPredictor",
    enum_name = "BranchType",
    enum_py_lib = ":BranchPredictor_py",
)

gem5_enum_generator(
    name = "enum_TargetProvider",
    enum_module = "src.cpu.pred.BranchPredictor",
    enum_name = "TargetProvider",
    enum_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "branch_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":branch_target_buffer_sim_object",
        ":conditional_predictor_sim_object",
        ":indirect_predictor_sim_object",
        ":return_addr_stack_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        ":btb",
        ":conditional",
        ":indirect",
        ":ras",
    ],
    extra_tool_deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
    sim_object_header = ":bpred_unit",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "BranchPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "branch_target_buffer_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
    sim_object_header = ":btb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "BranchTargetBuffer",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "return_addr_stack_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
    sim_object_header = ":ras",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "ReturnAddrStack",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "conditional_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
    sim_object_header = ":conditional",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "ConditionalPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "indirect_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
    sim_object_header = ":indirect",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "IndirectPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "bpred_unit",
    srcs = ["bpred_unit.cc"],
    hdrs = ["bpred_unit.hh"],
    deps = [
        ":branch_predictor_sim_object",
        ":branch_type",
        ":btb",
        ":conditional",
        ":debug_Branch",
        ":enum_BranchType_hh",
        ":enum_TargetProvider_hh",
        ":indirect",
        ":ras",
        "//src/arch/generic:pcstate",
        "//src/base:compiler",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/cpu:static_inst",
        "//src/sim:sim_object",
        "//src/sim/probe:pmu",
    ],
)

gem5_cc_library(
    name = "btb",
    srcs = ["btb.cc"],
    hdrs = ["btb.hh"],
    deps = [
        ":branch_target_buffer_sim_object",
        ":branch_type",
        ":debug_BTB",
        "//src/arch/generic:pcstate",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/cpu:static_inst",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "branch_type",
    hdrs = ["branch_type.hh"],
    deps = [
        ":enum_BranchType_hh",
        "//src/cpu:static_inst",
    ],
)

gem5_cc_library(
    name = "conditional",
    srcs = ["conditional.cc"],
    hdrs = ["conditional.hh"],
    deps = [
        ":branch_type",
        ":conditional_predictor_sim_object",
        "//src/arch/generic:pcstate",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/sim:clocked_object_hh",
    ],
)

gem5_cc_library(
    name = "indirect",
    srcs = ["indirect.cc"],
    hdrs = ["indirect.hh"],
    deps = [
        ":branch_type",
        ":indirect_predictor_sim_object",
        "//src/arch/generic:pcstate",
        "//src/cpu:inst_seq",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "ras",
    srcs = ["ras.cc"],
    hdrs = ["ras.hh"],
    deps = [
        ":branch_type",
        ":debug_RAS",
        ":return_addr_stack_sim_object",
        "//src/arch/generic:pcstate",
        "//src/base:statistics",
        "//src/base:types",
        "//src/cpu:static_inst",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "btb_entry_hh",
    hdrs = ["btb_entry.hh"],
    deps = [
        ":btb_indexing_policy_sim_object",
        ":btb_set_associative_sim_object",
        "//src/arch/generic:pcstate",
        "//src/base:intmath",
        "//src/base:types",
        "//src/cpu:static_inst",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)

gem5_sim_object(
    name = "btb_indexing_policy_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":btb_entry_hh",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "BTBIndexingPolicy",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "btb_set_associative_sim_object",
    extra_param_deps = [
        ":btb_indexing_policy_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":btb_entry_hh",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "BTBSetAssociative",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "simple_btb",
    srcs = ["simple_btb.cc"],
    hdrs = ["simple_btb.hh"],
    deps = [
        ":btb",
        ":btb_entry_hh",
        ":debug_BTB",
        ":simple_btb_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/cache:associative_cache",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)

gem5_sim_object(
    name = "simple_btb_sim_object",
    extra_deps = [
        ":btb_entry_hh",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_param_deps = [
        ":btb_indexing_policy_sim_object",
        ":branch_target_buffer_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":simple_btb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "SimpleBTB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "local_bp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":conditional_predictor_sim_object",
    ],
    sim_object_header = ":2bit_local",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "LocalBP",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "2bit_local",
    srcs = ["2bit_local.cc"],
    hdrs = ["2bit_local.hh"],
    deps = [
        ":branch_type",
        ":conditional",
        ":local_bp_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:sat_counter",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:debug_Fetch",
    ],
)

gem5_sim_object(
    name = "tournament_bp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":conditional_predictor_sim_object",
    ],
    sim_object_header = ":tournament",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TournamentBP",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tournament",
    srcs = ["tournament.cc"],
    hdrs = ["tournament.hh"],
    deps = [
        ":conditional",
        ":tournament_bp_sim_object",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:sat_counter",
        "//src/base:trace",
        "//src/base:types",
    ],
)

gem5_sim_object(
    name = "bi_mode_bp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":conditional_predictor_sim_object",
    ],
    sim_object_header = ":bi_mode",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "BiModeBP",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "bi_mode",
    srcs = ["bi_mode.cc"],
    hdrs = ["bi_mode.hh"],
    deps = [
        ":bi_mode_bp_sim_object",
        ":branch_type",
        ":conditional",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:sat_counter",
    ],
)

gem5_sim_object(
    name = "gshare_bp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":branch_predictor_sim_object",
    ],
    sim_object_header = ":gshare",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "GshareBP",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "gshare",
    srcs = ["gshare.cc"],
    hdrs = ["gshare.hh"],
    deps = [
        ":bpred_unit",
        ":gshare_bp_sim_object",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:sat_counter",
        "//src/base:trace",
    ],
)

gem5_sim_object(
    name = "simple_indirect_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":indirect_predictor_sim_object",
    ],
    sim_object_header = ":simple_indirect",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "SimpleIndirectPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "simple_indirect",
    srcs = ["simple_indirect.cc"],
    hdrs = ["simple_indirect.hh"],
    deps = [
        ":debug_Indirect",
        ":indirect",
        ":simple_indirect_predictor_sim_object",
        "//src/base:intmath",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/cpu:inst_seq",
    ],
)

gem5_sim_object(
    name = "tage_base_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":tage_base",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGEBase",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tage_base",
    srcs = ["tage_base.cc"],
    hdrs = ["tage_base.hh"],
    deps = [
        ":branch_type",
        ":debug_Tage",
        ":tage_base_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/cpu:debug_Fetch",
        "//src/cpu:null_static_inst",
        "//src/cpu:static_inst",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "tage_sim_object",
    extra_deps = [
        ":tage_base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":conditional_predictor_sim_object",
        ":tage_base_sim_object",
    ],
    sim_object_header = ":tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tage",
    srcs = ["tage.cc"],
    hdrs = ["tage.hh"],
    deps = [
        ":conditional",
        ":debug_Tage",
        ":tage_base",
        ":tage_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:debug_Fetch",
    ],
)

gem5_sim_object(
    name = "loop_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":loop_predictor",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "LoopPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "loop_predictor",
    srcs = ["loop_predictor.cc"],
    hdrs = ["loop_predictor.hh"],
    deps = [
        ":debug_LTage",
        ":loop_predictor_sim_object",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:static_inst",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "ltage_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":loop_predictor",
    ],
    extra_param_deps = [
        ":loop_predictor_sim_object",
        ":tage_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":ltage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "LTAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "ltage",
    srcs = ["ltage.cc"],
    hdrs = ["ltage.hh"],
    deps = [
        ":debug_LTage",
        ":loop_predictor",
        ":ltage_sim_object",
        ":tage",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:debug_Fetch",
    ],
)

gem5_sim_object(
    name = "statistical_corrector_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":statistical_corrector",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "StatisticalCorrector",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "statistical_corrector",
    srcs = ["statistical_corrector.cc"],
    hdrs = ["statistical_corrector.hh"],
    deps = [
        ":statistical_corrector_sim_object",
        "//src/base:intmath",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:static_inst",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "tage_sc_l_tage_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_base_sim_object",
    ],
    sim_object_header = ":tage_sc_l",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_TAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_loop_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":loop_predictor_sim_object",
    ],
    sim_object_header = ":tage_sc_l",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_LoopPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":statistical_corrector",
    ],
    extra_param_deps = [
        ":ltage_sim_object",
        ":statistical_corrector_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":tage_sc_l",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tage_sc_l",
    srcs = ["tage_sc_l.cc"],
    hdrs = ["tage_sc_l.hh"],
    deps = [
        ":debug_TageSCL",
        ":loop_predictor",
        ":ltage",
        ":statistical_corrector",
        ":tage_base",
        ":tage_sc_l_loop_predictor_sim_object",
        ":tage_sc_l_sim_object",
        ":tage_sc_l_tage_sim_object",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:trace",
    ],
)

gem5_sim_object(
    name = "tage_sc_l_tage_64_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_sc_l_tage_sim_object",
    ],
    sim_object_header = ":tage_sc_l_64kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_TAGE_64KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_64_kb_statistical_corrector_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":statistical_corrector_sim_object",
    ],
    sim_object_header = ":tage_sc_l_64kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_64KB_StatisticalCorrector",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_64_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_sc_l_sim_object",
    ],
    sim_object_header = ":tage_sc_l_64kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_64KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tage_sc_l_64kb",
    srcs = ["tage_sc_l_64KB.cc"],
    hdrs = ["tage_sc_l_64KB.hh"],
    deps = [
        ":tage_sc_l",
        ":tage_sc_l_64_kb_sim_object",
        ":tage_sc_l_64_kb_statistical_corrector_sim_object",
        ":tage_sc_l_tage_64_kb_sim_object",
    ],
)

gem5_sim_object(
    name = "tage_sc_l_tage_8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_sc_l_tage_sim_object",
    ],
    sim_object_header = ":tage_sc_l_8kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_TAGE_8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_8_kb_statistical_corrector_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":statistical_corrector_sim_object",
    ],
    sim_object_header = ":tage_sc_l_8kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_8KB_StatisticalCorrector",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "tage_sc_l_8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_sc_l_sim_object",
    ],
    sim_object_header = ":tage_sc_l_8kb",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "TAGE_SC_L_8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "tage_sc_l_8kb",
    srcs = ["tage_sc_l_8KB.cc"],
    hdrs = ["tage_sc_l_8KB.hh"],
    deps = [
        ":debug_TageSCL",
        ":tage_sc_l",
        ":tage_sc_l_8_kb_sim_object",
        ":tage_sc_l_8_kb_statistical_corrector_sim_object",
        ":tage_sc_l_tage_8_kb_sim_object",
        "//src/base:random",
    ],
)

gem5_sim_object(
    name = "multiperspective_perceptron_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":conditional_predictor_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptron",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron",
    srcs = ["multiperspective_perceptron.cc"],
    hdrs = ["multiperspective_perceptron.hh"],
    deps = [
        ":conditional",
        ":debug_Branch",
        ":multiperspective_perceptron_sim_object",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:trace",
    ],
)

gem5_sim_object(
    name = "multiperspective_perceptron8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":multiperspective_perceptron_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_8KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptron8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron_8KB",
    srcs = ["multiperspective_perceptron_8KB.cc"],
    hdrs = ["multiperspective_perceptron_8KB.hh"],
    deps = [
        ":multiperspective_perceptron",
        ":multiperspective_perceptron8_kb_sim_object",
    ],
)

gem5_sim_object(
    name = "multiperspective_perceptron64_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":multiperspective_perceptron_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_64KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptron64KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron_64KB",
    srcs = ["multiperspective_perceptron_64KB.cc"],
    hdrs = ["multiperspective_perceptron_64KB.hh"],
    deps = [
        ":multiperspective_perceptron",
        ":multiperspective_perceptron64_kb_sim_object",
    ],
)

gem5_sim_object(
    name = "mpp_tage_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_base_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_TAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "mpp_loop_predictor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":loop_predictor_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_LoopPredictor",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "mpp_statistical_corrector_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":statistical_corrector_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_StatisticalCorrector",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "multiperspective_perceptron_tage_sim_object",
    extra_deps = [
        ":loop_predictor",
        ":statistical_corrector",
        ":tage_base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":loop_predictor_sim_object",
        ":multiperspective_perceptron_sim_object",
        ":statistical_corrector_sim_object",
        ":tage_base_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":multiperspective_perceptron_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptronTAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron_tage",
    srcs = ["multiperspective_perceptron_tage.cc"],
    hdrs = ["multiperspective_perceptron_tage.hh"],
    deps = [
        ":loop_predictor",
        ":mpp_loop_predictor_sim_object",
        ":mpp_statistical_corrector_sim_object",
        ":mpp_tage_sim_object",
        ":multiperspective_perceptron",
        ":multiperspective_perceptron_tage_sim_object",
        ":statistical_corrector",
        ":tage_base",
        "//src/base:random",
    ],
)

gem5_sim_object(
    name = "mpp_statistical_corrector_64_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":mpp_statistical_corrector_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_64KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_StatisticalCorrector_64KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "multiperspective_perceptron_tage64_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":multiperspective_perceptron_tage_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_64KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptronTAGE64KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron_tage_64KB",
    srcs = ["multiperspective_perceptron_tage_64KB.cc"],
    hdrs = ["multiperspective_perceptron_tage_64KB.hh"],
    deps = [
        ":mpp_statistical_corrector_64_kb_sim_object",
        ":multiperspective_perceptron_tage",
        ":multiperspective_perceptron_tage64_kb_sim_object",
    ],
)

gem5_sim_object(
    name = "mpp_tage_8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":mpp_tage_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_8KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_TAGE_8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "mpp_loop_predictor_8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":mpp_loop_predictor_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_8KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_LoopPredictor_8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "mpp_statistical_corrector_8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":mpp_statistical_corrector_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_8KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MPP_StatisticalCorrector_8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "multiperspective_perceptron_tage8_kb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":multiperspective_perceptron_tage_sim_object",
    ],
    sim_object_header = ":multiperspective_perceptron_tage_8KB",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "MultiperspectivePerceptronTAGE8KB",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "multiperspective_perceptron_tage_8KB",
    srcs = ["multiperspective_perceptron_tage_8KB.cc"],
    hdrs = ["multiperspective_perceptron_tage_8KB.hh"],
    deps = [
        ":mpp_loop_predictor_8_kb_sim_object",
        ":mpp_statistical_corrector_8_kb_sim_object",
        ":mpp_tage_8_kb_sim_object",
        ":multiperspective_perceptron_tage",
        ":multiperspective_perceptron_tage8_kb_sim_object",
    ],
)

gem5_sim_object(
    name = "ittage_tage_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tage_base_sim_object",
        ":indirect_predictor_sim_object",
    ],
    sim_object_header = ":it_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "ITTAGE_TAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_sim_object(
    name = "ittage_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":indirect_predictor_sim_object",
        ":ittage_tage_sim_object",
    ],
    sim_object_header = ":it_tage",
    sim_object_module = "src.cpu.pred.BranchPredictor",
    sim_object_name = "ITTAGE",
    sim_object_py_lib = ":BranchPredictor_py",
)

gem5_cc_library(
    name = "it_tage",
    srcs = ["it_tage.cc"],
    hdrs = ["it_tage.hh"],
    deps = [
        ":bpred_unit",
        ":branch_type",
        ":debug_Indirect",
        ":debug_Tage",
        ":enum_BranchType_hh",
        ":indirect",
        ":ittage_sim_object",
        ":ittage_tage_sim_object",
        ":tage_base",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:static_inst",
    ],
)
