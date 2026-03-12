load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "profiler_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

gem5_cc_library(
    name = "AccessTraceForAddress",
    srcs = ["AccessTraceForAddress.cc"],
    hdrs = ["AccessTraceForAddress.hh"],
    deps = [
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
    ],
)

gem5_cc_library(
    name = "AddressProfiler",
    srcs = ["AddressProfiler.cc"],
    hdrs = ["AddressProfiler.hh"],
    deps = [
        ":AccessTraceForAddress",
        ":Profiler_hh",
        "//src/base:bitfield",
        "//src/base:stl_helpers",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:AccessType_hh",
        "//src/mem/ruby/slicc_interface:RubyRequest",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "Profiler_hh",
    hdrs = ["Profiler.hh"],
    deps = [
        "//src/base:callback",
        "//src/base:statistics",
        "//src/mem/ruby/common:MachineID_hh",
        "//src/mem/ruby/protocol:AccessType_hh",
        "//src/mem/ruby/protocol:PrefetchBit_hh",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
        "//src/mem/ruby/system:ruby_system_sim_object",
    ],
)

gem5_cc_library(
    name = "Profiler",
    srcs = ["Profiler.cc"],
    hdrs = ["Profiler.hh"],
    deps = [
        ":AddressProfiler",
        "//bazel/generated/config",
        "//src/base:callback",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:stl_helpers",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/protocol:AccessType_hh",
        "//src/mem/ruby/protocol:MachineType_hh",
        "//src/mem/ruby/protocol:PrefetchBit_hh",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:RubyRequestType_hh",
        "//src/mem/ruby/slicc_interface:RubyRequest",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/mem/ruby/system:Sequencer",
        "//src/mem/ruby/system:ruby_system_sim_object",
    ],
)

gem5_cc_library(
    name = "StoreTrace",
    srcs = ["StoreTrace.cc"],
    hdrs = ["StoreTrace.hh"],
    deps = [
        "//src/base:types",
        "//src/mem/ruby/common",
        "//src/sim:cur_tick",
    ],
)
