load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "CHIGeneric_py",
    srcs = ["CHIGeneric.py"],
    deps = [
        "//src/python/m5/objects:Controller",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/params",
    ],
)

gem5_debug_flag(
    name = "debug_RubyCHIGeneric",
    flag = "RubyCHIGeneric",
)

gem5_debug_flag(
    name = "debug_RubyCHIGenericVerbose",
    flag = "RubyCHIGenericVerbose",
)

gem5_cc_library(
    name = "CHIGenericController",
    srcs = ["CHIGenericController.cc"],
    hdrs = ["CHIGenericController.hh"],
    deps = [
        ":chi_generic_controller_sim_object",
        ":debug_RubyCHIGeneric",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/protocol:AccessPermission_hh",
        "//src/mem/ruby/protocol:MemoryMsg_hh",
        "//src/mem/ruby/protocol:CHIDataMsg_hh",
        "//src/mem/ruby/protocol:CHIProtocolInfo_hh",
        "//src/mem/ruby/protocol:CHIRequestMsg_hh",
        "//src/mem/ruby/protocol:CHIResponseMsg_hh",
        "//src/mem/ruby/slicc_interface:AbstractController",
        "//src/mem/ruby/system:CacheRecorder",
        "//src/mem/ruby/system:RubySystem",
        "//src/mem/ruby/system:Sequencer",
    ],
)

gem5_sim_object(
    name = "chi_generic_controller_sim_object",
    extra_param_deps = [
        "//src/mem/ruby/slicc_interface:ruby_controller_sim_object",
        "//src/mem/ruby/network:message_buffer_sim_object",
        "//src/sim:sim_object",
        "//src/mem/ruby/network:MessageBuffer",
    ],
    extra_tool_deps = [
        "//src/mem/ruby/network:MessageBuffer_py",
    ],
    sim_object_header = ":CHIGenericController",
    sim_object_module = "src.mem.ruby.protocol.CHI.generic.CHIGeneric",
    sim_object_name = "CHIGenericController",
    sim_object_py_lib = ":CHIGeneric_py",
)
