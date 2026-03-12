load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "system_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

py_library(
    name = "RubySystem_py",
    srcs = ["RubySystem.py"],
    deps = [
        ":system_init",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "Sequencer_py",
    srcs = ["Sequencer.py"],
    deps = [
        ":system_init",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_system_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
        "//src/mem:simple_mem",
    ],
    sim_object_header = ":RubySystem",
    sim_object_module = "src.mem.ruby.system.RubySystem",
    sim_object_name = "RubySystem",
    sim_object_py_lib = ":RubySystem_py",
)

gem5_sim_object(
    name = "ruby_port_sim_object",
    extra_param_deps = [
        ":ruby_system_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
    sim_object_header = ":RubyPort",
    sim_object_module = "src.mem.ruby.system.Sequencer",
    sim_object_name = "RubyPort",
    sim_object_py_lib = ":Sequencer_py",
)

gem5_sim_object(
    name = "ruby_sequencer_sim_object",
    extra_param_deps = [
        ":ruby_system_sim_object",
        ":ruby_port_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
        "//src/mem/ruby/structures:CacheMemory_hh",
    ],
    sim_object_header = ":Sequencer",
    sim_object_module = "src.mem.ruby.system.Sequencer",
    sim_object_name = "RubySequencer",
    sim_object_py_lib = ":Sequencer_py",
)

gem5_sim_object(
    name = "dma_sequencer_sim_object",
    extra_param_deps = [
        ":ruby_system_sim_object",
        ":ruby_port_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
    ],
    sim_object_header = ":DMASequencer",
    sim_object_module = "src.mem.ruby.system.Sequencer",
    sim_object_name = "DMASequencer",
    sim_object_py_lib = ":Sequencer_py",
)

gem5_sim_object(
    name = "ruby_htm_sequencer_sim_object",
    extra_param_deps = [
        ":ruby_system_sim_object",
        ":ruby_port_sim_object",
        ":ruby_sequencer_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":HTMSequencer",
    sim_object_module = "src.mem.ruby.system.Sequencer",
    sim_object_name = "RubyHTMSequencer",
    sim_object_py_lib = ":Sequencer_py",
)

gem5_sim_object(
    name = "ruby_port_proxy_sim_object",
    extra_param_deps = [
        ":ruby_port_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":RubyPortProxy",
    sim_object_module = "src.mem.ruby.system.Sequencer",
    sim_object_name = "RubyPortProxy",
    sim_object_py_lib = ":Sequencer_py",
)

gem5_cc_library(
    name = "RubySystem_hh",
    hdrs = [
        "RubySystem.hh",
    ],
    deps = [
        ":CacheRecorder_hh",
        ":ruby_system_sim_object",
        "//src/base:callback",
        "//src/base:output",
        "//src/mem:packet",
        "//src/mem/ruby/profiler:Profiler_hh",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/slicc_interface:ProtocolInfo",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "RubySystem",
    srcs = [
        "RubySystem.cc",
    ],
    hdrs = [
        "RubySystem.hh",
    ],
    deps = [
        ":CacheRecorder",
        ":DMASequencer",
        ":Sequencer",
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/base:output",
        "//src/base:statistics",
        "//src/mem:packet",
        "//src/mem:simple_mem",
        "//src/mem/ruby:debug_RubyCacheTrace",
        "//src/mem/ruby:debug_RubySystem",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/network:Topology",
        "//src/mem/ruby/profiler:Profiler",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/slicc_interface:ProtocolInfo",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/sim:clocked_object",
        "//src/sim:eventq",
        "//src/sim:simulate",
        "//src/sim:system",
        "@zlib//:zlib",
    ],
)

gem5_cc_library(
    name = "CacheRecorder_hh",
    hdrs = [
        "CacheRecorder.hh",
    ],
    deps = [
        "//src/base:types",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/common:TypeDefines_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
    ],
)

gem5_cc_library(
    name = "CacheRecorder",
    srcs = [
        "CacheRecorder.cc",
    ],
    hdrs = [
        "CacheRecorder.hh",
    ],
    deps = [
        ":RubySystem_hh",
        ":Sequencer",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem/ruby:debug_RubyCacheTrace",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
        "//src/sim:sim_exit",
    ],
)

gem5_cc_library(
    name = "Sequencer_hh",
    hdrs = [
        "Sequencer.hh",
    ],
    deps = [
        ":RubyPort_hh",
        ":ruby_sequencer_sim_object",
        "//src/cpu/testers/rubytest:RubyTester",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/protocol:MachineType_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
        "//src/mem/ruby/protocol:SequencerRequestType_hh",
        "//src/mem/ruby/structures:CacheMemory_hh",
    ],
)

gem5_cc_library(
    name = "Sequencer",
    srcs = [
        "Sequencer.cc",
    ],
    hdrs = [
        "Sequencer.hh",
    ],
    deps = [
        ":RubyPort_hh",
        ":RubySystem_hh",
        ":ruby_sequencer_sim_object",
        "//src/base:logging",
        "//src/cpu/testers/rubytest:RubyTester",
        "//src/mem:debug_LLSC",
        "//src/mem:packet",
        "//src/mem/ruby:debug_ProtocolTrace",
        "//src/mem/ruby:debug_RubyHitMiss",
        "//src/mem/ruby:debug_RubySequencer",
        "//src/mem/ruby:debug_RubyStats",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/profiler:Profiler_hh",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:RubyRequest",
        "//src/mem/ruby/slicc_interface:RubySlicc_Util",
        "//src/mem/ruby/structures:CacheMemory",
    ],
)

gem5_cc_library(
    name = "DMASequencer_hh",
    hdrs = [
        "DMASequencer.hh",
    ],
    deps = [
        ":RubyPort_hh",
        ":dma_sequencer_sim_object",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/protocol:DMASequencerRequestType_hh",
    ],
)

gem5_cc_library(
    name = "DMASequencer",
    srcs = [
        "DMASequencer.cc",
    ],
    hdrs = [
        "DMASequencer.hh",
    ],
    deps = [
        ":RubyPort_hh",
        ":RubySystem_hh",
        ":dma_sequencer_sim_object",
        "//src/mem/ruby:debug_RubyDma",
        "//src/mem/ruby:debug_RubyStats",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:DMASequencerRequestType_hh",
        "//src/mem/ruby/protocol:SequencerMsg_hh",
        "//src/mem/ruby/protocol:SequencerRequestType_hh",
    ],
)

gem5_cc_library(
    name = "RubyPort_hh",
    hdrs = [
        "RubyPort.hh",
    ],
    deps = [
        ":RubySystem_hh",
        ":ruby_port_sim_object",
        "//src/mem:tport",
        "//src/mem/ruby/common:MachineID_hh",
        "//src/mem/ruby/network:MessageBuffer_hh",
        "//src/mem/ruby/protocol:RequestStatus_hh",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "RubyPort",
    srcs = [
        "RubyPort.cc",
    ],
    hdrs = [
        "RubyPort.hh",
    ],
    deps = [
        ":RubySystem",
        ":ruby_port_sim_object",
        "//src/mem:simple_mem",
        "//src/mem:tport",
        "//src/mem/ruby:debug_RubyPort",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:AbstractController",
        "//src/sim:clocked_object",
        "//src/sim:debug_Config",
        "//src/sim:debug_Drain",
        "//src/sim:full_system",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "HTMSequencer_hh",
    hdrs = [
        "HTMSequencer.hh",
    ],
    deps = [
        ":RubyPort_hh",
        ":Sequencer_hh",
        ":ruby_htm_sequencer_sim_object",
        ":ruby_sequencer_sim_object",
        "//src/mem:htm",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/protocol:HtmCallbackMode_hh",
        "//src/mem/ruby/protocol:HtmFailedInCacheReason_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "HTMSequencer",
    srcs = [
        "HTMSequencer.cc",
    ],
    hdrs = [
        "HTMSequencer.hh",
    ],
    deps = [
        ":RubyPort",
        ":Sequencer",
        ":ruby_htm_sequencer_sim_object",
        ":ruby_sequencer_sim_object",
        "//src/mem:debug_HtmMem",
        "//src/mem:htm",
        "//src/mem/ruby:debug_RubyPort",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:RubySlicc_Util",
        "//src/mem/ruby/system:RubySystem",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "RubyPortProxy",
    srcs = [
        "RubyPortProxy.cc",
    ],
    hdrs = [
        "RubyPortProxy.hh",
    ],
    deps = [
        ":RubyPort",
        ":ruby_port_proxy_sim_object",
        "//src/base:logging",
    ],
)
