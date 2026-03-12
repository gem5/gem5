load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "slicc_interface_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

py_library(
    name = "Controller_py",
    srcs = ["Controller.py"],
    deps = [
        ":slicc_interface_init",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_controller_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/base:addr_range",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
    sim_object_header = ":AbstractController",
    sim_object_module = "src.mem.ruby.slicc_interface.Controller",
    sim_object_name = "RubyController",
    sim_object_py_lib = ":Controller_py",
)

gem5_cc_library(
    name = "AbstractCacheEntry",
    srcs = ["AbstractCacheEntry.cc"],
    hdrs = ["AbstractCacheEntry.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:trace",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/ruby:debug_RubyCache",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:AccessPermission_hh",
    ],
)

gem5_cc_library(
    name = "AbstractCacheEntry_hh",
    hdrs = ["AbstractCacheEntry.hh"],
    deps = [
        "//src/base:logging",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/protocol:AccessPermission_hh",
    ],
)

gem5_cc_library(
    name = "AbstractController_hh",
    hdrs = ["AbstractController.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/base:addr_range_map",
        "//src/base:callback",
        "//src/mem:packet",
        "//src/mem:qport",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/common:Histogram_hh",
        "//src/mem/ruby/common:MachineID_hh",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/protocol:AccessPermission_hh",
        "//src/mem/ruby/system:CacheRecorder_hh",
        "//src/sim:clocked_object",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "AbstractController",
    srcs = ["AbstractController.cc"],
    hdrs = ["AbstractController.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/base:addr_range_map",
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:qport",
        "//src/mem/ruby:debug_RubyQueue",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:ruby_controller_sim_object",
        "//src/mem/ruby/system:CacheRecorder",
        "//src/mem/ruby/system:RubySystem",
        "//src/mem/ruby/system:Sequencer",
        "//src/sim:clocked_object",
        "//src/sim:eventq",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "Message",
    hdrs = ["Message.hh"],
    deps = [
        "//src/mem:packet",
        "//src/mem/ruby/common:NetDest_hh",
        "//src/mem/ruby/common:WriteMask_hh",
        "//src/mem/ruby/protocol:MessageSizeType_hh",
    ],
)

gem5_cc_library(
    name = "Message_hh",
    hdrs = ["Message.hh"],
    deps = [
        "//src/mem:packet",
        "//src/mem/ruby/common:NetDest_hh",
        "//src/mem/ruby/common:WriteMask_hh",
        "//src/mem/ruby/protocol:MessageSizeType_hh",
    ],
)

gem5_cc_library(
    name = "ProtocolInfo",
    hdrs = ["ProtocolInfo.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "RubyRequest",
    srcs = ["RubyRequest.cc"],
    hdrs = ["RubyRequest.hh"],
    deps = [
        ":Message",
        ":RubySlicc_Util",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:PrefetchBit_hh",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
    ],
)

gem5_cc_library(
    name = "RubyRequest_hh",
    hdrs = ["RubyRequest.hh"],
    deps = [
        ":Message_hh",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:DataBlock_hh",
        "//src/mem/ruby/common:WriteMask_hh",
        "//src/mem/ruby/protocol:PrefetchBit_hh",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
    ],
)

gem5_cc_library(
    name = "RubySlicc_ComponentMapping",
    hdrs = ["RubySlicc_ComponentMapping.hh"],
    deps = [
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:MachineType_hh",
    ],
)

gem5_cc_library(
    name = "RubySlicc_Util",
    hdrs = ["RubySlicc_Util.hh"],
    deps = [
        "//src/mem:packet",
        "//src/mem/ruby:debug_RubyProtocol",
        "//src/mem/ruby:debug_RubySlicc",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
    ],
)

gem5_cc_library(
    name = "RubySlicc_includes",
    hdrs = ["RubySlicc_includes.hh"],
    deps = [
        ":RubySlicc_ComponentMapping",
        ":RubySlicc_Util",
    ],
)
