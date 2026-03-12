load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "ReplacementPolicies_py",
    srcs = ["ReplacementPolicies.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "replacement_policies_init",
    srcs = ["__init__.py"],
    deps = [
        ":ReplacementPolicies_py",
    ],
)

gem5_cc_library(
    name = "base",
    hdrs = ["base.hh"],
    deps = [
        ":base_replacement_policy_sim_object",
        ":replaceable_entry",
        "//src/base:compiler",
        "//src/mem:packet_hh",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "base_replacement_policy_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "BaseReplacementPolicy",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "replaceable_entry",
    hdrs = ["replaceable_entry.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:cprintf",
    ],
)

gem5_cc_test(
    name = "replaceable_entry_test",
    srcs = [
        "replaceable_entry.test.cc",
    ],
    deps = [
        ":replaceable_entry",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "lru_rp",
    srcs = ["lru_rp.cc"],
    hdrs = ["lru_rp.hh"],
    deps = [
        ":base",
        ":lrurp_sim_object",
        "//src/sim:cur_tick",
    ],
)

gem5_sim_object(
    name = "lrurp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":lru_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "LRURP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "random_rp",
    srcs = ["random_rp.cc"],
    hdrs = ["random_rp.hh"],
    deps = [
        ":base",
        ":random_rp_sim_object",
        "//src/base:random",
    ],
)

gem5_sim_object(
    name = "random_rp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":random_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "RandomRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "dueling_rp",
    srcs = ["dueling_rp.cc"],
    hdrs = ["dueling_rp.hh"],
    deps = [
        ":base",
        ":dueling_rp_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/mem/cache/tags:dueling",
    ],
)

gem5_sim_object(
    name = "dueling_rp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":base",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":dueling_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "DuelingRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "fifo_rp",
    srcs = ["fifo_rp.cc"],
    hdrs = ["fifo_rp.hh"],
    deps = [
        ":base",
        ":fiforp_sim_object",
        "//src/base:types",
        "//src/sim:cur_tick",
    ],
)

gem5_sim_object(
    name = "fiforp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":fifo_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "FIFORP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "lfu_rp",
    srcs = ["lfu_rp.cc"],
    hdrs = ["lfu_rp.hh"],
    deps = [
        ":base",
        ":lfurp_sim_object",
    ],
)

gem5_sim_object(
    name = "lfurp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":lfu_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "LFURP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "second_chance_rp",
    srcs = ["second_chance_rp.cc"],
    hdrs = ["second_chance_rp.hh"],
    deps = [
        ":base",
        ":fifo_rp",
        ":second_chance_rp_sim_object",
    ],
)

gem5_sim_object(
    name = "second_chance_rp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":fiforp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":second_chance_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "SecondChanceRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "bip_rp",
    srcs = ["bip_rp.cc"],
    hdrs = ["bip_rp.hh"],
    deps = [
        ":biprp_sim_object",
        ":lru_rp",
        "//src/base:random",
        "//src/sim:cur_tick",
    ],
)

gem5_sim_object(
    name = "biprp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":lrurp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":bip_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "BIPRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "mru_rp",
    srcs = ["mru_rp.cc"],
    hdrs = ["mru_rp.hh"],
    deps = [
        ":base",
        ":mrurp_sim_object",
        "//src/base:types",
        "//src/sim:cur_tick",
    ],
)

gem5_sim_object(
    name = "mrurp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":lrurp_sim_object",
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":mru_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "MRURP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "brrip_rp",
    srcs = ["brrip_rp.cc"],
    hdrs = ["brrip_rp.hh"],
    deps = [
        ":base",
        ":brriprp_sim_object",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:sat_counter",
    ],
)

gem5_sim_object(
    name = "brriprp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":brrip_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "BRRIPRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "ship_rp",
    srcs = ["ship_rp.cc"],
    hdrs = ["ship_rp.hh"],
    deps = [
        ":brrip_rp",
        ":s_hi_p_mem_rp_sim_object",
        ":s_hi_ppcrp_sim_object",
        ":s_hi_prp_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:sat_counter",
        "//src/mem:packet_hh",
    ],
)

gem5_sim_object(
    name = "s_hi_prp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
        ":brriprp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":ship_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "SHiPRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_sim_object(
    name = "s_hi_p_mem_rp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":brriprp_sim_object",
        ":s_hi_prp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":ship_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "SHiPMemRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_sim_object(
    name = "s_hi_ppcrp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":brriprp_sim_object",
        ":s_hi_p_mem_rp_sim_object",
        ":s_hi_prp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":ship_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "SHiPPCRP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "tree_plru_rp",
    srcs = ["tree_plru_rp.cc"],
    hdrs = ["tree_plru_rp.hh"],
    deps = [
        ":base",
        ":tree_plrurp_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
    ],
)

gem5_sim_object(
    name = "tree_plrurp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":tree_plru_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "TreePLRURP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

gem5_cc_library(
    name = "weighted_lru_rp",
    srcs = ["weighted_lru_rp.cc"],
    hdrs = ["weighted_lru_rp.hh"],
    deps = [
        ":lru_rp",
        ":weighted_lrurp_sim_object",
        "//src/base:types",
        "//src/sim:cur_tick",
    ],
)

gem5_sim_object(
    name = "weighted_lrurp_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":lrurp_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":weighted_lru_rp",
    sim_object_module = "src.mem.cache.replacement_policies.ReplacementPolicies",
    sim_object_name = "WeightedLRURP",
    sim_object_py_lib = ":ReplacementPolicies_py",
)

# TODO(jlowepower): Depends on throw/exceptions.
# gem5_cc_test(
#     name = "tree_plru_rp_test",
#     srcs = ["tree_plru_rp.test.cc"],
#     deps = [
#         ":tree_plrurp_sim_object",
#         ":tree_plru_rp",
#         "//ext/googletest:gtest_main",
#         "//src/sim:sim_events",
#         "//src/sim:sim_object",
#     ],
# )

gem5_cc_test(
    name = "lfu_rp_test",
    srcs = ["lfu_rp.test.cc"],
    deps = [
        ":lfu_rp",
        ":lfurp_sim_object",
        "//ext/googletest:gtest_main",
        "//src/base:statistics",
        "//src/sim:root",
    ],
)

gem5_cc_test(
    name = "lru_rp_test",
    srcs = ["lru_rp.test.cc"],
    deps = [
        ":lru_rp",
        ":lrurp_sim_object",
        "//ext/googletest:gtest_main",
        "//src/base:statistics",
        "//src/sim:root",
    ],
)
