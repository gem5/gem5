load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "Cache_py",
    srcs = ["Cache.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:Compressors",
        "//src/python/m5/objects:Prefetcher",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/objects:Tags",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "cache_init",
    srcs = ["__init__.py"],
    deps = [
        ":Cache_py",
        "//src/mem/cache/compressors:compressors_init",
        "//src/mem/cache/prefetch:prefetch_init",
        "//src/mem/cache/replacement_policies:replacement_policies_init",
        "//src/mem/cache/tags:tags_init",
    ],
)

gem5_enum_generator(
    name = "enum_Clusivity",
    enum_module = "src.mem.cache.Cache",
    enum_name = "Clusivity",
    enum_py_lib = ":Cache_py",
)

gem5_sim_object(
    name = "write_allocator_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":cache",
    sim_object_module = "src.mem.cache.Cache",
    sim_object_name = "WriteAllocator",
    sim_object_py_lib = ":Cache_py",
)

gem5_sim_object(
    name = "base_cache_sim_object",
    extra_deps = [
        ":cache_blk",
        ":cache_probe_arg",
        ":mshr",
        ":cache",
        "//src/mem/cache/prefetch:base",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/tags:base",
        "//src/mem/cache/compressors:base",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
    extra_param_deps = [
        ":write_allocator_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/mem/cache/prefetch:base_prefetcher_sim_object",
        "//src/mem/cache/tags:base_tags_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/compressors:base_cache_compressor_sim_object",
        "//src/mem/cache/tags/partitioning_policies:partition_manager_sim_object",
        ":enum_Clusivity_hh",
        "//src/base:addr_range",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
        "//src/python/m5/objects:PartitioningPolicies",
    ],
    sim_object_header = ":base_hh",
    sim_object_module = "src.mem.cache.Cache",
    sim_object_name = "BaseCache",
    sim_object_py_lib = ":Cache_py",
)

gem5_sim_object(
    name = "cache_sim_object",
    extra_deps = [
        ":base_hh",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_cache_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":cache",
    sim_object_module = "src.mem.cache.Cache",
    sim_object_name = "Cache",
    sim_object_py_lib = ":Cache_py",
)

gem5_sim_object(
    name = "noncoherent_cache_sim_object",
    extra_deps = [
        ":base_hh",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":base_cache_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":noncoherent_cache",
    sim_object_module = "src.mem.cache.Cache",
    sim_object_name = "NoncoherentCache",
    sim_object_py_lib = ":Cache_py",
)

gem5_debug_flag(
    name = "debug_Cache",
    flag = "Cache",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_CacheComp",
    flag = "CacheComp",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_CachePort",
    flag = "CachePort",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_CacheRepl",
    flag = "CacheRepl",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_CacheTags",
    flag = "CacheTags",
)

gem5_debug_flag(
    name = "debug_CacheVerbose",
    flag = "CacheVerbose",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_HWPrefetch",
    flag = "HWPrefetch",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_MSHR",
    flag = "MSHR",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_HWPrefetchQueue",
    flag = "HWPrefetchQueue",
)

gem5_debug_flag(
    name = "debug_PartitionPolicy",
    flag = "PartitionPolicy",
    included_by = [":debug_CacheAll"],
)

gem5_debug_flag(
    name = "debug_CacheAll",
    components = [
        ":debug_Cache",
        ":debug_CacheComp",
        ":debug_CachePort",
        ":debug_CacheRepl",
        ":debug_CacheVerbose",
        ":debug_HWPrefetch",
        ":debug_MSHR",
        ":debug_PartitionPolicy",
    ],
    flag = "CacheAll",
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_cache_sim_object",
        ":cache_blk",
        ":cache_probe_arg",
        ":debug_Cache",
        ":debug_CacheComp",
        ":debug_CachePort",
        ":debug_CacheRepl",
        ":debug_CacheVerbose",
        ":debug_HWPrefetch",
        ":enum_Clusivity_hh",
        ":mshr",
        ":mshr_queue",
        ":queue_entry",
        ":write_allocator_sim_object",
        ":write_queue",
        "//src/base:addr_range",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:packet_queue",
        "//src/mem:qport",
        "//src/mem:request",
        "//src/mem/cache/compressors:base",
        "//src/mem/cache/prefetch:base",
        "//src/mem/cache/tags:base",
        "//src/mem/cache/tags:compressed_tags",
        "//src/mem/cache/tags:super_blk",
        "//src/mem/cache/tags/partitioning_policies:partition_manager",
        "//src/sim:clocked_object_hh",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
        "//src/sim:system_hh",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "base_hh",
    hdrs = ["base.hh"],
    deps = [
        ":cache_blk",
        ":cache_probe_arg",
        ":debug_Cache",
        ":debug_CachePort",
        ":enum_Clusivity_hh",
        ":mshr_queue_hh",
        ":write_allocator_sim_object",
        ":write_queue_hh",
        "//src/base:addr_range",
        "//src/base:compiler",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:packet_queue",
        "//src/mem:qport",
        "//src/mem:request",
        "//src/mem/cache/tags:base",
        "//src/sim:clocked_object_hh",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
        "//src/sim:system_hh",
        "//src/sim/probe",
    ],
)

gem5_cc_library(
    name = "cache",
    srcs = ["cache.cc"],
    hdrs = ["cache.hh"],
    deps = [
        ":base",
        ":cache_blk",
        ":cache_sim_object",
        ":debug_Cache",
        ":debug_CacheTags",
        ":debug_CacheVerbose",
        ":enum_Clusivity_hh",
        ":mshr",
        ":write_queue_entry_hh",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/mem/cache/compressors:base",
        "//src/mem/cache/tags:base",
    ],
)

gem5_cc_library(
    name = "cache_blk",
    srcs = ["cache_blk.cc"],
    hdrs = ["cache_blk.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:printable",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/mem/cache/tags:tagged_entry",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "cache_probe_arg",
    hdrs = ["cache_probe_arg.hh"],
    deps = [
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "mshr",
    srcs = ["mshr.cc"],
    hdrs = ["mshr.hh"],
    deps = [
        ":base_hh",
        ":debug_MSHR",
        ":queue_entry",
        "//src/base:logging",
        "//src/base:printable",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "mshr_hh",
    hdrs = ["mshr.hh"],
    deps = [
        ":debug_MSHR",
        ":queue_entry",
        "//src/base:printable",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "mshr_queue",
    srcs = ["mshr_queue.cc"],
    hdrs = ["mshr_queue.hh"],
    deps = [
        ":debug_MSHR",
        ":mshr",
        ":queue",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "mshr_queue_hh",
    hdrs = ["mshr_queue.hh"],
    deps = [
        ":mshr_hh",
        ":queue",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "noncoherent_cache",
    srcs = ["noncoherent_cache.cc"],
    hdrs = ["noncoherent_cache.hh"],
    deps = [
        ":base",
        ":cache_blk",
        ":debug_Cache",
        ":mshr_hh",
        ":noncoherent_cache_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "write_queue",
    srcs = ["write_queue.cc"],
    hdrs = ["write_queue.hh"],
    deps = [
        ":queue",
        ":write_queue_entry",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "write_queue_hh",
    hdrs = ["write_queue.hh"],
    deps = [
        ":queue",
        ":write_queue_entry_hh",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "queue_entry",
    hdrs = ["queue_entry.hh"],
    deps = [
        "//src/base:named",
        "//src/base:types",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "queue",
    hdrs = ["queue.hh"],
    deps = [
        ":queue_entry",
        "//src/base:logging",
        "//src/base:named",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/sim:cur_tick",
        "//src/sim:debug_Drain",
        "//src/sim:drain",
    ],
)

gem5_cc_library(
    name = "write_queue_entry",
    srcs = ["write_queue_entry.cc"],
    hdrs = ["write_queue_entry.hh"],
    deps = [
        ":base_hh",
        ":queue_entry",
        "//src/base:logging",
        "//src/base:printable",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "write_queue_entry_hh",
    hdrs = ["write_queue_entry.hh"],
    deps = [
        ":queue_entry",
        "//src/base:printable",
        "//src/base:types",
        "//src/mem:packet",
    ],
)
