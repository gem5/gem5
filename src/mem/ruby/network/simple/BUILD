load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "SimpleLink_py",
    srcs = ["SimpleLink.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BasicLink",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "SimpleNetwork_py",
    srcs = ["SimpleNetwork.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BasicRouter",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/objects:Network",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

gem5_sim_object(
    name = "simple_ext_link_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        "//src/mem/ruby/network:basic_ext_link_sim_object",
    ],
    sim_object_header = ":SimpleLink",
    sim_object_module = "src.mem.ruby.network.simple.SimpleLink",
    sim_object_name = "SimpleExtLink",
    sim_object_py_lib = ":SimpleLink_py",
)

gem5_sim_object(
    name = "simple_int_link_sim_object",
    extra_param_deps = [
        "//src/mem/ruby/network:basic_int_link_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        "//src/mem/ruby/network:message_buffer_sim_object",
        "//src/mem/ruby/network:MessageBuffer",
    ],
    sim_object_header = ":SimpleLink",
    sim_object_module = "src.mem.ruby.network.simple.SimpleLink",
    sim_object_name = "SimpleIntLink",
    sim_object_py_lib = ":SimpleLink_py",
)

gem5_sim_object(
    name = "simple_network_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        "//src/mem/ruby/network:ruby_network_sim_object",
    ],
    sim_object_header = ":SimpleNetwork",
    sim_object_module = "src.mem.ruby.network.simple.SimpleNetwork",
    sim_object_name = "SimpleNetwork",
    sim_object_py_lib = ":SimpleNetwork_py",
)

gem5_sim_object(
    name = "base_routing_unit_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = "//src/mem/ruby/network/simple/routing:BaseRoutingUnit",
    sim_object_module = "src.mem.ruby.network.simple.SimpleNetwork",
    sim_object_name = "BaseRoutingUnit",
    sim_object_py_lib = ":SimpleNetwork_py",
)

gem5_sim_object(
    name = "weight_based_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        ":base_routing_unit_sim_object",
    ],
    sim_object_header = "//src/mem/ruby/network/simple/routing:WeightBased",
    sim_object_module = "src.mem.ruby.network.simple.SimpleNetwork",
    sim_object_name = "WeightBased",
    sim_object_py_lib = ":SimpleNetwork_py",
)

gem5_sim_object(
    name = "switch_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        ":base_routing_unit_sim_object",
        "//src/mem/ruby/network:message_buffer_sim_object",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network:basic_router_sim_object",
        "//src/mem/ruby/network/simple/routing:BaseRoutingUnit",
    ],
    sim_object_header = ":Switch",
    sim_object_module = "src.mem.ruby.network.simple.SimpleNetwork",
    sim_object_name = "Switch",
    sim_object_py_lib = ":SimpleNetwork_py",
)

gem5_cc_library(
    name = "PerfectSwitch",
    srcs = ["PerfectSwitch.cc"],
    hdrs = ["PerfectSwitch.hh"],
    deps = [
        ":SimpleNetwork_hh",
        ":Switch",
        "//src/base:cast",
        "//src/base:cprintf",
        "//src/base:random",
        "//src/mem/ruby:debug_RubyNetwork",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/slicc_interface:Message",
    ],
)

gem5_cc_library(
    name = "PerfectSwitch_hh",
    hdrs = ["PerfectSwitch.hh"],
    deps = [
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/common:TypeDefines_hh",
    ],
)

gem5_cc_library(
    name = "SimpleLink",
    srcs = ["SimpleLink.cc"],
    hdrs = ["SimpleLink.hh"],
    deps = [
        ":simple_ext_link_sim_object",
        ":simple_int_link_sim_object",
        "//src/mem/ruby/network:BasicLink",
    ],
)

gem5_cc_library(
    name = "SimpleNetwork",
    srcs = ["SimpleNetwork.cc"],
    hdrs = ["SimpleNetwork.hh"],
    deps = [
        ":PerfectSwitch",
        ":SimpleLink",
        ":Switch",
        ":Throttle",
        ":simple_network_sim_object",
        "//src/base:cast",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/profiler:Profiler",
    ],
)

gem5_cc_library(
    name = "SimpleNetwork_hh",
    hdrs = ["SimpleNetwork.hh"],
    deps = [
        ":simple_network_sim_object",
        "//src/mem/ruby/network:Network",
    ],
)

gem5_cc_library(
    name = "Switch",
    srcs = ["Switch.cc"],
    hdrs = ["Switch.hh"],
    deps = [
        ":PerfectSwitch_hh",
        ":SimpleNetwork_hh",
        ":Throttle_hh",
        ":switch_sim_object",
        "//src/base:cprintf",
        "//src/mem:packet",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:BasicRouter",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network/simple/routing:BaseRoutingUnit",
        "//src/mem/ruby/protocol:MessageSizeType_hh",
    ],
)

gem5_cc_library(
    name = "Throttle",
    srcs = ["Throttle.cc"],
    hdrs = ["Throttle.hh"],
    deps = [
        ":Switch",
        "//src/base:cprintf",
        "//src/mem/ruby:debug_RubyNetwork",
        "//src/mem/ruby/common",
        "//src/mem/ruby/network:MessageBuffer",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/slicc_interface:Message",
        "//src/mem/ruby/system:RubySystem",
        "//src/sim:stats",
    ],
)

gem5_cc_library(
    name = "Throttle_hh",
    hdrs = ["Throttle.hh"],
    deps = [
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)
