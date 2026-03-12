load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "PartitioningPolicies_py",
    srcs = ["PartitioningPolicies.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "base_pp",
    srcs = ["base_pp.cc"],
    hdrs = ["base_pp.hh"],
    deps = [
        ":base_partitioning_policy_sim_object",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "base_partitioning_policy_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":base_pp",
    sim_object_module = "src.mem.cache.tags.partitioning_policies.PartitioningPolicies",
    sim_object_name = "BasePartitioningPolicy",
    sim_object_py_lib = ":PartitioningPolicies_py",
)

gem5_cc_library(
    name = "way_allocation",
    srcs = ["way_allocation.cc"],
    hdrs = ["way_allocation.hh"],
    deps = [
        ":way_policy_allocation_sim_object",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "way_policy_allocation_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":way_allocation",
    sim_object_module = "src.mem.cache.tags.partitioning_policies.PartitioningPolicies",
    sim_object_name = "WayPolicyAllocation",
    sim_object_py_lib = ":PartitioningPolicies_py",
)

gem5_cc_library(
    name = "way_pp",
    srcs = ["way_pp.cc"],
    hdrs = ["way_pp.hh"],
    deps = [
        ":base_pp",
        ":way_allocation",
        ":way_partitioning_policy_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/mem/cache:debug_PartitionPolicy",
        "//src/mem/cache/replacement_policies:replaceable_entry",
    ],
)

gem5_sim_object(
    name = "way_partitioning_policy_sim_object",
    extra_deps = [
        ":base_pp",
        ":way_allocation",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_partitioning_policy_sim_object",
        ":way_policy_allocation_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":way_pp",
    sim_object_module = "src.mem.cache.tags.partitioning_policies.PartitioningPolicies",
    sim_object_name = "WayPartitioningPolicy",
    sim_object_py_lib = ":PartitioningPolicies_py",
)

gem5_cc_library(
    name = "max_capacity_pp",
    srcs = ["max_capacity_pp.cc"],
    hdrs = ["max_capacity_pp.hh"],
    deps = [
        ":base_partitioning_policy_sim_object",
        ":base_pp",
        ":max_capacity_partitioning_policy_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/mem/cache:cache_blk",
        "//src/mem/cache:debug_PartitionPolicy",
    ],
)

gem5_sim_object(
    name = "max_capacity_partitioning_policy_sim_object",
    extra_deps = [
        ":base_pp",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_partitioning_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":max_capacity_pp",
    sim_object_module = "src.mem.cache.tags.partitioning_policies.PartitioningPolicies",
    sim_object_name = "MaxCapacityPartitioningPolicy",
    sim_object_py_lib = ":PartitioningPolicies_py",
)

gem5_cc_library(
    name = "partition_manager",
    srcs = ["partition_manager.cc"],
    hdrs = ["partition_manager.hh"],
    deps = [
        ":base_pp",
        ":partition_manager_sim_object",
        "//src/mem:packet",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "partition_manager_sim_object",
    extra_deps = [
        ":base_pp",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_partitioning_policy_sim_object",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":partition_manager",
    sim_object_module = "src.mem.cache.tags.partitioning_policies.PartitioningPolicies",
    sim_object_name = "PartitionManager",
    sim_object_py_lib = ":PartitioningPolicies_py",
)
