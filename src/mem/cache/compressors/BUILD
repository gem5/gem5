load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_CacheComp",
    flag = "CacheComp",
)

py_library(
    name = "Compressors_py",
    srcs = ["Compressors.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "compressors_init",
    srcs = ["__init__.py"],
    deps = [
        ":Compressors_py",
    ],
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_cache_compressor_sim_object",
        ":debug_CacheComp",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem/cache:base_hh",
        "//src/mem/cache/tags:super_blk",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "dictionary_compressor",
    hdrs = [
        "dictionary_compressor.hh",
        "dictionary_compressor_impl.hh",
    ],
    deps = [
        ":base",
        ":base_dictionary_compressor_sim_object",
        ":debug_CacheComp",
        "//src/base:bitfield",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "base_delta",
    srcs = ["base_delta.cc"],
    hdrs = [
        "base_delta.hh",
        "base_delta_impl.hh",
    ],
    deps = [
        ":base16_delta8_sim_object",
        ":base32_delta16_sim_object",
        ":base32_delta8_sim_object",
        ":base64_delta16_sim_object",
        ":base64_delta32_sim_object",
        ":base64_delta8_sim_object",
        ":debug_CacheComp",
        ":dictionary_compressor",
        "//src/base:bitfield",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "cpack",
    srcs = ["cpack.cc"],
    hdrs = ["cpack.hh"],
    deps = [
        ":c_pack_sim_object",
        ":dictionary_compressor",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "fpc",
    srcs = ["fpc.cc"],
    hdrs = ["fpc.hh"],
    deps = [
        ":dictionary_compressor",
        ":fpc_sim_object",
        "//src/base:bitfield",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "fpcd",
    srcs = ["fpcd.cc"],
    hdrs = ["fpcd.hh"],
    deps = [
        ":dictionary_compressor",
        ":fpcd_sim_object",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "frequent_values",
    srcs = ["frequent_values.cc"],
    hdrs = ["frequent_values.hh"],
    deps = [
        ":base",
        ":debug_CacheComp",
        ":frequent_values_compressor_sim_object",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:sat_counter",
        "//src/base:types",
        "//src/base/cache:associative_cache",
        "//src/mem/cache:base",
        "//src/mem/cache:cache_probe_arg",
        "//src/mem/cache/compressors/encoders:huffman_encoder",
        "//src/sim:eventq",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "multi",
    srcs = ["multi.cc"],
    hdrs = ["multi.hh"],
    deps = [
        ":base",
        ":debug_CacheComp",
        ":multi_compressor_sim_object",
        "//src/base:bitfield",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "perfect",
    srcs = ["perfect.cc"],
    hdrs = ["perfect.hh"],
    deps = [
        ":base",
        ":debug_CacheComp",
        ":perfect_compressor_sim_object",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "repeated_qwords",
    srcs = ["repeated_qwords.cc"],
    hdrs = ["repeated_qwords.hh"],
    deps = [
        ":debug_CacheComp",
        ":dictionary_compressor",
        ":repeated_qwords_compressor_sim_object",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "zero",
    srcs = ["zero.cc"],
    hdrs = ["zero.hh"],
    deps = [
        ":base_dictionary_compressor",
        ":debug_CacheComp",
        ":dictionary_compressor",
        ":zero_compressor_sim_object",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "base_dictionary_compressor",
    srcs = ["base_dictionary_compressor.cc"],
    hdrs = ["dictionary_compressor.hh"],
    deps = [
        ":base",
        ":base_dictionary_compressor_sim_object",
        ":dictionary_compressor",
        "//src/base:bitfield",
        "//src/base:statistics",
        "//src/base:types",
    ],
)

gem5_sim_object(
    name = "base64_delta8_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base64Delta8",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base64_delta16_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base64Delta16",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base64_delta32_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base64Delta32",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base32_delta8_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base32Delta8",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base32_delta16_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base32Delta16",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base16_delta8_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base_delta",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "Base16Delta8",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "c_pack_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":cpack",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "CPack",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "fpc_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":fpc",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "FPC",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "fpcd_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":fpcd",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "FPCD",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "frequent_values_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":base",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/mem/cache/tags/indexing_policies:base_indexing_policy_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        ":base_cache_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":frequent_values",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "FrequentValuesCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "multi_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":base",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_cache_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":multi",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "MultiCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "perfect_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":base",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_cache_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":perfect",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "PerfectCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "repeated_qwords_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":repeated_qwords",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "RepeatedQwordsCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "zero_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":dictionary_compressor",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_dictionary_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":zero",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "ZeroCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base_cache_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "BaseCacheCompressor",
    sim_object_py_lib = ":Compressors_py",
)

gem5_sim_object(
    name = "base_dictionary_compressor_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":base",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_cache_compressor_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":dictionary_compressor",
    sim_object_module = "src.mem.cache.compressors.Compressors",
    sim_object_name = "BaseDictionaryCompressor",
    sim_object_py_lib = ":Compressors_py",
)
