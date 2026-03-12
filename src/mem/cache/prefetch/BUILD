load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "Prefetcher_py",
    srcs = ["Prefetcher.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:Tags",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "prefetch_init",
    srcs = ["__init__.py"],
    deps = [
        ":Prefetcher_py",
    ],
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_prefetcher_sim_object",
        "//src/arch/generic:tlb",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/base:statistics",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/mem/cache:cache_probe_arg",
        "//src/sim:byteswap",
        "//src/sim:clocked_object",
        "//src/sim:system_hh",
        "//src/sim/probe",
    ],
)

gem5_sim_object(
    name = "base_prefetcher_sim_object",
    extra_param_deps = [
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "BasePrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "multi",
    srcs = ["multi.cc"],
    hdrs = ["multi.hh"],
    deps = [
        ":base",
        ":multi_prefetcher_sim_object",
    ],
)

gem5_sim_object(
    name = "multi_prefetcher_sim_object",
    extra_param_deps = [
        ":base_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
        ":base",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":multi",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "MultiPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "queued",
    srcs = ["queued.cc"],
    hdrs = ["queued.hh"],
    deps = [
        ":base",
        ":queued_prefetcher_sim_object",
        "//src/arch/generic:mmu_hh",
        "//src/arch/generic:tlb",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache:debug_HWPrefetchQueue",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "queued_prefetcher_sim_object",
    extra_param_deps = [
        ":base_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":queued",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "QueuedPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "stride",
    srcs = ["stride.cc"],
    hdrs = ["stride.hh"],
    deps = [
        ":queued",
        ":stride_prefetcher_hashed_set_associative_sim_object",
        ":stride_prefetcher_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:sat_counter",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/cache:associative_cache",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/tags/indexing_policies:set_associative",
    ],
)

gem5_sim_object(
    name = "stride_prefetcher_hashed_set_associative_sim_object",
    extra_param_deps = [
        "//src/mem/cache/tags:tagged_set_associative_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Tags",
    ],
    sim_object_header = ":stride",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "StridePrefetcherHashedSetAssociative",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_sim_object(
    name = "stride_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        ":stride_prefetcher_hashed_set_associative_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":stride",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "StridePrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "tagged",
    srcs = ["tagged.cc"],
    hdrs = ["tagged.hh"],
    deps = [
        ":queued",
        ":tagged_prefetcher_sim_object",
        "//src/mem:packet_hh",
    ],
)

gem5_sim_object(
    name = "tagged_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":tagged",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "TaggedPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "indirect_memory",
    srcs = ["indirect_memory.cc"],
    hdrs = ["indirect_memory.hh"],
    deps = [
        ":indirect_memory_prefetcher_sim_object",
        ":queued",
        "//src/base:sat_counter",
        "//src/base/cache:associative_cache",
        "//src/mem/cache/tags:tagged_entry",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "indirect_memory_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":indirect_memory",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "IndirectMemoryPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "signature_path",
    srcs = ["signature_path.cc"],
    hdrs = ["signature_path.hh"],
    deps = [
        ":queued",
        ":signature_path_prefetcher_sim_object",
        "//src/base:sat_counter",
        "//src/base/cache:associative_cache",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/tags:tagged_entry",
    ],
)

gem5_sim_object(
    name = "signature_path_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":signature_path",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "SignaturePathPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "signature_path_v2",
    srcs = ["signature_path_v2.cc"],
    hdrs = ["signature_path_v2.hh"],
    deps = [
        ":signature_path",
        ":signature_path_prefetcher_v2_sim_object",
        "//src/base/cache:associative_cache",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
    ],
)

gem5_sim_object(
    name = "signature_path_prefetcher_v2_sim_object",
    extra_param_deps = [
        ":signature_path_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":signature_path_v2",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "SignaturePathPrefetcherV2",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "access_map_pattern_matching",
    srcs = ["access_map_pattern_matching.cc"],
    hdrs = ["access_map_pattern_matching.hh"],
    deps = [
        ":access_map_pattern_matching_sim_object",
        ":ampm_prefetcher_sim_object",
        ":queued",
        "//src/base/cache:associative_cache",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/tags:tagged_entry",
        "//src/sim:clocked_object_hh",
    ],
)

gem5_sim_object(
    name = "access_map_pattern_matching_sim_object",
    extra_param_deps = [
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":access_map_pattern_matching",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "AccessMapPatternMatching",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_sim_object(
    name = "ampm_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        ":access_map_pattern_matching_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":access_map_pattern_matching",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "AMPMPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "delta_correlating_prediction_tables",
    srcs = ["delta_correlating_prediction_tables.cc"],
    hdrs = ["delta_correlating_prediction_tables.hh"],
    deps = [
        ":dcpt_prefetcher_sim_object",
        ":delta_correlating_prediction_tables_sim_object",
        ":queued",
        "//src/base:circular_queue",
        "//src/base/cache:associative_cache",
        "//src/mem/cache:debug_HWPrefetch",
    ],
)

gem5_sim_object(
    name = "delta_correlating_prediction_tables_sim_object",
    extra_param_deps = [
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags/indexing_policies:base_indexing_policy_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":delta_correlating_prediction_tables",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "DeltaCorrelatingPredictionTables",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_sim_object(
    name = "dcpt_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        ":delta_correlating_prediction_tables_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":delta_correlating_prediction_tables",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "DCPTPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "irregular_stream_buffer",
    srcs = ["irregular_stream_buffer.cc"],
    hdrs = ["irregular_stream_buffer.hh"],
    deps = [
        ":irregular_stream_buffer_prefetcher_sim_object",
        ":queued",
        "//src/base:callback",
        "//src/base:sat_counter",
        "//src/base/cache:associative_cache",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/tags:tagged_entry",
    ],
)

gem5_sim_object(
    name = "irregular_stream_buffer_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":irregular_stream_buffer",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "IrregularStreamBufferPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "slim_ampm",
    srcs = ["slim_ampm.cc"],
    hdrs = ["slim_ampm.hh"],
    deps = [
        ":access_map_pattern_matching",
        ":delta_correlating_prediction_tables",
        ":queued",
        ":slim_ampm_prefetcher_sim_object",
    ],
)

gem5_sim_object(
    name = "slim_ampm_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        ":access_map_pattern_matching_sim_object",
        ":delta_correlating_prediction_tables_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
        ":access_map_pattern_matching",
        ":delta_correlating_prediction_tables",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":slim_ampm",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "SlimAMPMPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "bop",
    srcs = ["bop.cc"],
    hdrs = ["bop.hh"],
    deps = [
        ":bop_prefetcher_sim_object",
        ":queued",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
    ],
)

gem5_sim_object(
    name = "bop_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":bop",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "BOPPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "sbooe",
    srcs = ["sbooe.cc"],
    hdrs = ["sbooe.hh"],
    deps = [
        ":queued",
        ":sbooe_prefetcher_sim_object",
        "//src/base:circular_queue",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
    ],
)

gem5_sim_object(
    name = "sbooe_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":sbooe",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "SBOOEPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "sms",
    srcs = ["sms.cc"],
    hdrs = ["sms.hh"],
    deps = [
        ":queued",
        ":sms_prefetcher_sim_object",
        "//src/mem:packet_hh",
        "//src/mem/cache:debug_HWPrefetch",
    ],
)

gem5_sim_object(
    name = "sms_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/sim:sim_object_hh",
    ],
    sim_object_header = ":sms",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "SmsPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "spatio_temporal_memory_streaming",
    srcs = ["spatio_temporal_memory_streaming.cc"],
    hdrs = ["spatio_temporal_memory_streaming.hh"],
    deps = [
        ":queued",
        ":s_te_ms_prefetcher_sim_object",
        "//src/base:circular_queue",
        "//src/base:sat_counter",
        "//src/base/cache:associative_cache",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/tags:tagged_entry",
    ],
)

gem5_sim_object(
    name = "s_te_ms_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":spatio_temporal_memory_streaming",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "STeMSPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)

gem5_cc_library(
    name = "pif",
    srcs = ["pif.cc"],
    hdrs = ["pif.hh"],
    deps = [
        ":pif_prefetcher_sim_object",
        ":queued",
        "//src/base:circular_queue",
        "//src/base/cache:associative_cache",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/tags:tagged_entry",
    ],
)

gem5_sim_object(
    name = "pif_prefetcher_sim_object",
    extra_param_deps = [
        ":queued_prefetcher_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/tags:tagged_indexing_policy_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem/cache/tags:tagged_entry",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:IndexingPolicies",
        "//src/python/m5/objects:ReplacementPolicies",
    ],
    sim_object_header = ":pif",
    sim_object_module = "src.mem.cache.prefetch.Prefetcher",
    sim_object_name = "PIFPrefetcher",
    sim_object_py_lib = ":Prefetcher_py",
)
