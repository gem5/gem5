load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "IndexingPolicies_py",
    srcs = ["IndexingPolicies.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "base",
    hdrs = ["base.hh"],
    deps = [
        ":base_indexing_policy_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "base_indexing_policy_sim_object",
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
    sim_object_module = "src.mem.cache.tags.indexing_policies.IndexingPolicies",
    sim_object_name = "BaseIndexingPolicy",
    sim_object_py_lib = ":IndexingPolicies_py",
)

gem5_cc_library(
    name = "set_associative",
    srcs = ["set_associative.cc"],
    hdrs = ["set_associative.hh"],
    deps = [
        ":base",
        ":set_associative_sim_object",
        "//src/mem/cache/replacement_policies:replaceable_entry",
    ],
)

gem5_sim_object(
    name = "set_associative_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_indexing_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":set_associative",
    sim_object_module = "src.mem.cache.tags.indexing_policies.IndexingPolicies",
    sim_object_name = "SetAssociative",
    sim_object_py_lib = ":IndexingPolicies_py",
)

gem5_cc_library(
    name = "skewed_associative",
    srcs = ["skewed_associative.cc"],
    hdrs = ["skewed_associative.hh"],
    deps = [
        ":base",
        ":skewed_associative_sim_object",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/mem/cache/replacement_policies:replaceable_entry",
    ],
)

gem5_sim_object(
    name = "skewed_associative_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_indexing_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":skewed_associative",
    sim_object_module = "src.mem.cache.tags.indexing_policies.IndexingPolicies",
    sim_object_name = "SkewedAssociative",
    sim_object_py_lib = ":IndexingPolicies_py",
)
