load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "dueling",
    srcs = ["dueling.cc"],
    hdrs = ["dueling.hh"],
    deps = [
        "//src/base:bitfield",
        "//src/base:logging",
        "//src/base:sat_counter",
    ],
)

gem5_cc_test(
    name = "dueling_test",
    srcs = ["dueling.test.cc"],
    deps = [
        ":dueling",
        "//ext/googletest:gtest_main",
        "//src/base:sat_counter",
    ],
)

py_library(
    name = "Tags_py",
    srcs = ["Tags.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:PartitioningPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "tags_init",
    srcs = ["__init__.py"],
    deps = [
        ":Tags_py",
    ],
)

gem5_cc_library(
    name = "tagged_entry",
    hdrs = ["tagged_entry.hh"],
    deps = [
        ":tagged_indexing_policy_sim_object",
        ":tagged_set_associative_sim_object",
        "//src/base:cprintf",
        "//src/base:types",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)

gem5_sim_object(
    name = "tagged_indexing_policy_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":tagged_entry",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "TaggedIndexingPolicy",
    sim_object_py_lib = ":Tags_py",
)

gem5_sim_object(
    name = "tagged_set_associative_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":tagged_indexing_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/mem/cache/tags/indexing_policies:IndexingPolicies_py",
    ],
    sim_object_header = ":tagged_entry",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "TaggedSetAssociative",
    sim_object_py_lib = ":Tags_py",
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_tags_sim_object",
        "//src/base:callback",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/mem/cache:cache_blk",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
        "//src/sim:clocked_object_hh",
        "//src/sim:core",
        "//src/sim:sim_exit",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "base_set_assoc",
    srcs = ["base_set_assoc.cc"],
    hdrs = ["base_set_assoc.hh"],
    deps = [
        ":base",
        ":base_set_assoc_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem/cache:cache_blk",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
    ],
)

gem5_cc_library(
    name = "compressed_tags",
    srcs = ["compressed_tags.cc"],
    hdrs = ["compressed_tags.hh"],
    deps = [
        ":compressed_tags_sim_object",
        ":sector_tags",
        ":super_blk",
        "//src/base:trace",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_CacheComp",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
    ],
)

gem5_cc_library(
    name = "super_blk",
    srcs = ["super_blk.cc"],
    hdrs = ["super_blk.hh"],
    deps = [
        ":sector_blk",
        "//src/base:bitfield",
    ],
)

gem5_cc_library(
    name = "fa_lru",
    srcs = ["fa_lru.cc"],
    hdrs = ["fa_lru.hh"],
    deps = [
        ":base",
        ":falru_sim_object",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem/cache:cache_blk",
        "//src/mem/cache/replacement_policies:replaceable_entry",
    ],
)

gem5_cc_library(
    name = "sector_blk",
    srcs = ["sector_blk.cc"],
    hdrs = ["sector_blk.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/mem/cache:cache_blk",
        "//src/mem/cache/replacement_policies:replaceable_entry",
    ],
)

gem5_cc_library(
    name = "sector_tags",
    srcs = ["sector_tags.cc"],
    hdrs = ["sector_tags.hh"],
    deps = [
        ":base",
        ":sector_blk",
        ":sector_tags_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
    ],
)

gem5_sim_object(
    name = "base_tags_sim_object",
    extra_deps = [
        "//src/sim:clocked_object_hh",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
        "//src/mem/cache/tags:tagged_entry",
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        ":tagged_indexing_policy_sim_object",
        "//src/mem/cache/tags/partitioning_policies:partition_manager_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":base",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "BaseTags",
    sim_object_py_lib = ":Tags_py",
)

gem5_sim_object(
    name = "base_set_assoc_sim_object",
    extra_deps = [
        ":base",
        "//src/mem/cache/replacement_policies:base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_tags_sim_object",
        "//src/mem/cache/tags/partitioning_policies:partition_manager_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_set_assoc",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "BaseSetAssoc",
    sim_object_py_lib = ":Tags_py",
)

gem5_sim_object(
    name = "sector_tags_sim_object",
    extra_deps = [
        ":base",
        ":sector_blk",
        "//src/mem/cache/replacement_policies:base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_tags_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":sector_tags",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "SectorTags",
    sim_object_py_lib = ":Tags_py",
)

gem5_sim_object(
    name = "compressed_tags_sim_object",
    extra_deps = [
        ":sector_tags",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":sector_tags_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":compressed_tags",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "CompressedTags",
    sim_object_py_lib = ":Tags_py",
)

gem5_sim_object(
    name = "falru_sim_object",
    extra_deps = [
        ":base",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_tags_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":fa_lru",
    sim_object_module = "src.mem.cache.tags.Tags",
    sim_object_name = "FALRU",
    sim_object_py_lib = ":Tags_py",
)
