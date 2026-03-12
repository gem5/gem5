load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "network_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

py_library(
    name = "MessageBuffer_py",
    srcs = ["MessageBuffer.py"],
    deps = [
        ":network_init",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "message_buffer_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":enum_MessageRandomization_hh",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":MessageBuffer",
    sim_object_module = "src.mem.ruby.network.MessageBuffer",
    sim_object_name = "MessageBuffer",
    sim_object_py_lib = ":MessageBuffer_py",
)

gem5_enum_generator(
    name = "enum_MessageRandomization",
    enum_module = "src.mem.ruby.network.MessageBuffer",
    enum_name = "MessageRandomization",
    enum_py_lib = ":MessageBuffer_py",
)

py_library(
    name = "BasicLink_py",
    srcs = ["BasicLink.py"],
    deps = [
        ":network_init",
        ":MessageBuffer_py",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BasicRouter",
        "//src/python/m5/objects:Controller",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "basic_link_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        ":message_buffer_sim_object",
    ],
    sim_object_header = ":BasicLink",
    sim_object_module = "src.mem.ruby.network.BasicLink",
    sim_object_name = "BasicLink",
    sim_object_py_lib = ":BasicLink_py",
)

gem5_sim_object(
    name = "basic_ext_link_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        "//src/mem/ruby/slicc_interface:ruby_controller_sim_object",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        ":message_buffer_sim_object",
        ":basic_router_sim_object",
        ":basic_link_sim_object",
        ":BasicRouter_hh",
    ],
    sim_object_header = ":BasicLink",
    sim_object_module = "src.mem.ruby.network.BasicLink",
    sim_object_name = "BasicExtLink",
    sim_object_py_lib = ":BasicLink_py",
)

gem5_sim_object(
    name = "basic_int_link_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        ":BasicRouter_hh",
        ":basic_router_sim_object",
        ":basic_link_sim_object",
        ":message_buffer_sim_object",
    ],
    sim_object_header = ":BasicLink",
    sim_object_module = "src.mem.ruby.network.BasicLink",
    sim_object_name = "BasicIntLink",
    sim_object_py_lib = ":BasicLink_py",
)

py_library(
    name = "BasicRouter_py",
    srcs = ["BasicRouter.py"],
    deps = [
        ":network_init",
        ":MessageBuffer_py",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "basic_router_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
        ":message_buffer_sim_object",
    ],
    sim_object_header = ":BasicRouter",
    sim_object_module = "src.mem.ruby.network.BasicRouter",
    sim_object_name = "BasicRouter",
    sim_object_py_lib = ":BasicRouter_py",
)

py_library(
    name = "Network_py",
    srcs = ["Network.py"],
    deps = [
        ":network_init",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BasicLink",
        "//src/python/m5/objects:BasicRouter",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_network_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        ":basic_link_sim_object",
        ":basic_ext_link_sim_object",
        ":basic_int_link_sim_object",
        ":basic_router_sim_object",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:clocked_object",
        "//src/sim:sim_object",
        ":BasicLink",
        ":BasicRouter",
    ],
    sim_object_header = ":Network",
    sim_object_module = "src.mem.ruby.network.Network",
    sim_object_name = "RubyNetwork",
    sim_object_py_lib = ":Network_py",
)

gem5_cc_library(
    name = "MessageBuffer",
    srcs = ["MessageBuffer.cc"],
    hdrs = ["MessageBuffer.hh"],
    deps = [
        ":dummy_port",
        ":message_buffer_sim_object",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:stl_helpers",
        "//src/base:trace",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem/ruby:debug_RubyQueue",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/slicc_interface:Message",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "MessageBuffer_hh",
    hdrs = ["MessageBuffer.hh"],
    deps = [
        ":dummy_port",
        ":message_buffer_sim_object",
        "//src/base:trace",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem/ruby:debug_RubyQueue",
        "//src/mem/ruby/common:Address_hh",
        "//src/mem/ruby/common:Consumer_hh",
        "//src/mem/ruby/slicc_interface:Message_hh",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "dummy_port",
    hdrs = ["dummy_port.hh"],
    deps = [
        "//src/mem:port",
    ],
)

gem5_cc_library(
    name = "BasicLink",
    srcs = ["BasicLink.cc"],
    hdrs = ["BasicLink.hh"],
    deps = [
        ":basic_ext_link_sim_object",
        ":basic_int_link_sim_object",
        ":basic_link_sim_object",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "BasicRouter",
    srcs = ["BasicRouter.cc"],
    hdrs = ["BasicRouter.hh"],
    deps = [
        ":MessageBuffer",
        ":Topology_hh",
        ":basic_router_sim_object",
        "//src/mem/ruby:debug_RubyNetwork",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "BasicRouter_hh",
    hdrs = ["BasicRouter.hh"],
    deps = [
        ":basic_router_sim_object",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "Topology",
    srcs = ["Topology.cc"],
    hdrs = ["Topology.hh"],
    deps = [
        ":BasicLink",
        ":BasicRouter",
        ":Network",
        "//src/base:trace",
        "//src/mem/ruby:debug_RubyNetwork",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:shared",
        "//src/mem/ruby/slicc_interface:AbstractController_hh",
        "//src/mem/ruby/system:RubySystem_hh",
    ],
)

gem5_cc_library(
    name = "Topology_hh",
    hdrs = ["Topology.hh"],
    deps = [
        ":BasicLink",
        "//src/mem/ruby/common:TypeDefines_hh",
        "//src/mem/ruby/protocol:LinkDirection_hh",
    ],
)

gem5_cc_library(
    name = "Network",
    srcs = ["Network.cc"],
    hdrs = ["Network.hh"],
    deps = [
        ":BasicLink",
        ":Topology_hh",
        ":dummy_port",
        ":ruby_network_sim_object",
        "//src/base:addr_range",
        "//src/base:logging",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:LinkDirection_hh",
        "//src/mem/ruby/protocol:MessageSizeType_hh",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:clocked_object",
    ],
)
