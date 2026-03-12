load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "structures_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

py_library(
    name = "RubyCache_py",
    srcs = ["RubyCache.py"],
    deps = [
        ":structures_init",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_cache_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        "//src/mem/cache/replacement_policies:base",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":CacheMemory",
    sim_object_module = "src.mem.ruby.structures.RubyCache",
    sim_object_name = "RubyCache",
    sim_object_py_lib = ":RubyCache_py",
)

py_library(
    name = "DirectoryMemory_py",
    srcs = ["DirectoryMemory.py"],
    deps = [
        ":structures_init",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_directory_memory_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/base:addr_range",
        "//src/sim:sim_object",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
    sim_object_header = ":DirectoryMemory",
    sim_object_module = "src.mem.ruby.structures.DirectoryMemory",
    sim_object_name = "RubyDirectoryMemory",
    sim_object_py_lib = ":DirectoryMemory_py",
)

py_library(
    name = "RubyPrefetcher_py",
    srcs = ["RubyPrefetcher.py"],
    deps = [
        ":structures_init",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_prefetcher_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":RubyPrefetcher",
    sim_object_module = "src.mem.ruby.structures.RubyPrefetcher",
    sim_object_name = "RubyPrefetcher",
    sim_object_py_lib = ":RubyPrefetcher_py",
)

py_library(
    name = "WireBuffer_py",
    srcs = ["WireBuffer.py"],
    deps = [
        ":structures_init",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_wire_buffer_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":WireBuffer",
    sim_object_module = "src.mem.ruby.structures.WireBuffer",
    sim_object_name = "RubyWireBuffer",
    sim_object_py_lib = ":WireBuffer_py",
)

gem5_cc_library(
    name = "CacheMemory",
    srcs = ["CacheMemory.cc"],
    hdrs = ["CacheMemory.hh"],
    deps = [
        ":ALUFreeListArray",
        ":BankedArray",
        ":ruby_cache_sim_object",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:statistics",
        "//src/mem:debug_HtmMem",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/replacement_policies:weighted_lru_rp",
        "//src/mem/ruby:debug_RubyCache",
        "//src/mem/ruby:debug_RubyCacheTrace",
        "//src/mem/ruby:debug_RubyResourceStalls",
        "//src/mem/ruby:debug_RubyStats",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:AbstractCacheEntry",
        "//src/mem/ruby/system:CacheRecorder_hh",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "CacheMemory_hh",
    hdrs = ["CacheMemory.hh"],
    deps = [
        ":ALUFreeListArray",
        ":BankedArray",
        ":ruby_cache_sim_object",
        "//src/base:random",
        "//src/base:statistics",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/ruby:debug_RubyCache",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/protocol:CacheRequestType_hh",
        "//src/mem/ruby/protocol:CacheResourceType_hh",
        "//src/mem/ruby/protocol:RubyRequest_hh",
        "//src/mem/ruby/slicc_interface:AbstractCacheEntry",
        "//src/mem/ruby/slicc_interface:RubySlicc_ComponentMapping",
        "//src/mem/ruby/system:CacheRecorder_hh",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "DirectoryMemory",
    srcs = ["DirectoryMemory.cc"],
    hdrs = ["DirectoryMemory.hh"],
    deps = [
        ":ruby_directory_memory_sim_object",
        "//src/base:addr_range",
        "//src/base:intmath",
        "//src/mem/ruby:debug_RubyCache",
        "//src/mem/ruby:debug_RubyStats",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:AbstractCacheEntry",
        "//src/mem/ruby/slicc_interface:RubySlicc_Util",
        "//src/mem/ruby/system:RubySystem",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "DirectoryMemory_hh",
    hdrs = ["DirectoryMemory.hh"],
    deps = [
        ":ruby_directory_memory_sim_object",
        "//src/base:addr_range",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/protocol:DirectoryRequestType_hh",
        "//src/mem/ruby/slicc_interface:AbstractCacheEntry",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "RubyPrefetcher",
    srcs = ["RubyPrefetcher.cc"],
    hdrs = ["RubyPrefetcher.hh"],
    deps = [
        ":ruby_prefetcher_sim_object",
        "//src/base:bitfield",
        "//src/base:circular_queue",
        "//src/base:statistics",
        "//src/mem/ruby:debug_RubyPrefetcher",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/slicc_interface:AbstractController",
        "//src/mem/ruby/slicc_interface:RubyRequest",
        "//src/mem/ruby/system:RubySystem",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "RubyPrefetcher_hh",
    hdrs = ["RubyPrefetcher.hh"],
    deps = [
        ":ruby_prefetcher_sim_object",
        "//src/base:circular_queue",
        "//src/base:statistics",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/network:MessageBuffer_hh",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/slicc_interface:RubyRequest_hh",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "WireBuffer",
    srcs = ["WireBuffer.cc"],
    hdrs = ["WireBuffer.hh"],
    deps = [
        ":ruby_wire_buffer_sim_object",
        "//src/base:cprintf",
        "//src/base:stl_helpers",
        "//src/mem/ruby/common",
        "//src/mem/ruby/slicc_interface:Message",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "WireBuffer_hh",
    hdrs = ["WireBuffer.hh"],
    deps = [
        ":ruby_wire_buffer_sim_object",
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/slicc_interface:Message_hh",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "ALUFreeListArray",
    srcs = ["ALUFreeListArray.cc"],
    hdrs = ["ALUFreeListArray.hh"],
    deps = [
        "//src/base:intmath",
        "//src/mem/ruby/common",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "BankedArray",
    srcs = ["BankedArray.cc"],
    hdrs = ["BankedArray.hh"],
    deps = [
        "//src/base:intmath",
        "//src/mem/ruby/common",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "MN_TBEStorage_hh",
    hdrs = ["MN_TBEStorage.hh"],
    deps = [
        ":TBEStorage",
        "//src/base:statistics",
        "//src/mem/ruby/common",
    ],
)

gem5_cc_library(
    name = "MN_TBETable",
    srcs = ["MN_TBETable.cc"],
    hdrs = ["MN_TBETable.hh"],
    deps = [
        ":TBETable",
        "//src/mem/ruby/protocol:MiscNode_TBE_hh",
    ],
)

gem5_cc_library(
    name = "MN_TBETable_hh",
    hdrs = ["MN_TBETable.hh"],
    deps = [
        ":TBETable_hh",
        "//src/mem/ruby/protocol:MiscNode_TBE_hh",
    ],
)

gem5_cc_library(
    name = "PerfectCacheMemory",
    hdrs = ["PerfectCacheMemory.hh"],
    deps = [
        "//src/base:compiler",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:AccessPermission_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "PerfectCacheMemory_hh",
    hdrs = ["PerfectCacheMemory.hh"],
    deps = [
        "//src/base:compiler",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/protocol:AccessPermission_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "PersistentTable",
    srcs = ["PersistentTable.cc"],
    hdrs = ["PersistentTable.hh"],
    deps = [
        "//src/base:intmath",
        "//src/base:logging",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:AccessType_hh",
    ],
)

gem5_cc_library(
    name = "PersistentTable_hh",
    hdrs = ["PersistentTable.hh"],
    deps = [
        "//src/base:intmath",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:MachineID_hh",
        "//src/mem/ruby/common:NetDest_hh",
        "//src/mem/ruby/protocol:AccessType_hh",
    ],
)

gem5_cc_library(
    name = "RubyPrefetcherProxy",
    srcs = ["RubyPrefetcherProxy.cc"],
    hdrs = ["RubyPrefetcherProxy.hh"],
    deps = [
        "//src/mem/cache:cache_probe_arg",
        "//src/mem/cache:debug_HWPrefetch",
        "//src/mem/cache/prefetch:base",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/slicc_interface:RubyRequest_hh",
        "//src/mem/ruby/slicc_interface:ruby_controller_sim_object",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "RubyPrefetcherProxy_hh",
    hdrs = ["RubyPrefetcherProxy.hh"],
    deps = [
        "//src/mem/cache:cache_probe_arg",
        "//src/mem/cache/prefetch:base",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/slicc_interface:RubyRequest_hh",
    ],
)

gem5_cc_library(
    name = "TBEStorage",
    srcs = ["TBEStorage.cc"],
    hdrs = ["TBEStorage.hh"],
    deps = [
        "//src/base:statistics",
        "//src/base/stats:group",
    ],
)

gem5_cc_library(
    name = "TBEStorage_hh",
    hdrs = ["TBEStorage.hh"],
    deps = [
        "//src/base:statistics",
        "//src/base/stats:group",
    ],
)

gem5_cc_library(
    name = "TBETable_hh",
    hdrs = ["TBETable.hh"],
    deps = [
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "TBETable",
    hdrs = ["TBETable.hh"],
    deps = [
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "TimerTable",
    srcs = ["TimerTable.cc"],
    hdrs = ["TimerTable.hh"],
    deps = [
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "TimerTable_hh",
    hdrs = ["TimerTable.hh"],
    deps = [
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:Consumer_hh",
    ],
)
