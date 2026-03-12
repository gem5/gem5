load("@rules_python//python:defs.bzl", "py_library")
load(
    "//bazel:rules.bzl",
    "gem5_cc_library",
    "gem5_debug_flag",
    "gem5_enum_generator",
    "gem5_pybind_library",
    "gem5_sim_object",
)

py_library(
    name = "traffic_gen_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseTrafficGen_py",
        ":PyTrafficGen_py",
        ":TrafficGen_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "PyTrafficGen_py",
    srcs = ["PyTrafficGen.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseTrafficGen",
    ],
)

gem5_pybind_library(
    name = "pygen",
    srcs = ["pygen.cc"],
    hdrs = ["pygen.hh"],
    deps = [
        ":base",
        ":base_gen",
        ":debug_TrafficGen",
        ":py_traffic_gen_sim_object",
        "//bazel/generated/config",
        "//src/base:compiler",
        "//src/base:trace",
        "//src/sim:init",
    ],
)

gem5_sim_object(
    name = "py_traffic_gen_sim_object",
    extra_deps = [
        "//src/sim:clocked_object",
        "//src/sim:sim_object",
    ],
    extra_param_deps = [":base_traffic_gen_sim_object"],
    extra_tool_deps = [
        "//src/python/m5:defines",
        "//src/python/m5/objects:BaseTrafficGen",
    ],
    sim_object_header = ":pygen",
    sim_object_module = "src.cpu.testers.traffic_gen.PyTrafficGen",
    sim_object_name = "PyTrafficGen",
    sim_object_py_lib = ":PyTrafficGen_py",
)

gem5_debug_flag(
    name = "debug_TrafficGen",
    desc = "None",
    flag = "TrafficGen",
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":base_gen",
        ":base_hh",
        ":base_traffic_gen_sim_object",
        ":debug_TrafficGen",
        ":dram_gen",
        ":dram_rot_gen",
        ":exit_gen",
        ":hybrid_gen",
        ":idle_gen",
        ":linear_gen",
        ":nvm_gen",
        ":random_gen",
        ":stream_gen",
        ":strided_gen",
        "//bazel/generated/config",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:statistics",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:qport",
        "//src/sim:clocked_object",
        "//src/sim:debug_Checkpoint",
        "//src/sim:sim_exit",
        "//src/sim:stats",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "base_hh",
    hdrs = ["base.hh"],
    deps = [
        ":base_traffic_gen_sim_object",
        "//src/base:statistics",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:qport",
        "//src/sim:clocked_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "base_gen",
    srcs = ["base_gen.cc"],
    hdrs = ["base_gen.hh"],
    deps = [
        ":base_hh",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
    ],
)

py_library(
    name = "BaseTrafficGen_py",
    srcs = ["BaseTrafficGen.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:ClockedObject",
    ],
)

gem5_sim_object(
    name = "base_traffic_gen_sim_object",
    extra_deps = [
        "//src/sim:clocked_object",
        "//src/sim:sim_object",
        "//src/sim:system_hh",
        ":traffic_gen",
    ],
    extra_param_deps = [
        ":enum_StreamGenType_hh",
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.cpu.testers.traffic_gen.BaseTrafficGen",
    sim_object_name = "BaseTrafficGen",
    sim_object_py_lib = ":BaseTrafficGen_py",
)

gem5_enum_generator(
    name = "enum_StreamGenType",
    enum_module = "src.cpu.testers.traffic_gen.BaseTrafficGen",
    enum_name = "StreamGenType",
    enum_py_lib = ":BaseTrafficGen_py",
)

gem5_cc_library(
    name = "dram_gen",
    srcs = ["dram_gen.cc"],
    hdrs = ["dram_gen.hh"],
    deps = [
        ":debug_TrafficGen",
        ":random_gen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "random_gen",
    srcs = ["random_gen.cc"],
    hdrs = ["random_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "dram_rot_gen",
    srcs = ["dram_rot_gen.cc"],
    hdrs = ["dram_rot_gen.hh"],
    deps = [
        ":debug_TrafficGen",
        ":dram_gen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "exit_gen",
    srcs = ["exit_gen.cc"],
    hdrs = ["exit_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:sim_exit",
    ],
)

gem5_cc_library(
    name = "hybrid_gen",
    srcs = ["hybrid_gen.cc"],
    hdrs = ["hybrid_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "idle_gen",
    srcs = ["idle_gen.cc"],
    hdrs = ["idle_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "linear_gen",
    srcs = ["linear_gen.cc"],
    hdrs = ["linear_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "nvm_gen",
    srcs = ["nvm_gen.cc"],
    hdrs = ["nvm_gen.hh"],
    deps = [
        ":debug_TrafficGen",
        ":random_gen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:enum_AddrMap_hh",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "stream_gen",
    srcs = ["stream_gen.cc"],
    hdrs = ["stream_gen.hh"],
    deps = [
        ":base_traffic_gen_sim_object",
        "//src/base:random",
    ],
)

gem5_cc_library(
    name = "strided_gen",
    srcs = ["strided_gen.cc"],
    hdrs = ["strided_gen.hh"],
    deps = [
        ":base_gen",
        ":debug_TrafficGen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/mem:packet",
    ],
)

py_library(
    name = "TrafficGen_py",
    srcs = ["TrafficGen.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:BaseTrafficGen",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "traffic_gen_sim_object",
    extra_deps = [
        "//src/sim:sim_object",
    ],
    extra_param_deps = [":base_traffic_gen_sim_object"],
    extra_tool_deps = [
        "//src/python/m5/params",
        "//src/python/m5/objects:BaseTrafficGen",
    ],
    sim_object_header = ":traffic_gen",
    sim_object_module = "src.cpu.testers.traffic_gen.TrafficGen",
    sim_object_name = "TrafficGen",
    sim_object_py_lib = ":TrafficGen_py",
)

gem5_cc_library(
    name = "traffic_gen",
    srcs = ["traffic_gen.cc"],
    hdrs = ["traffic_gen.hh"],
    deps = [
        ":base",
        ":debug_TrafficGen",
        ":traffic_gen_sim_object",
        "//src/base:intmath",
        "//src/base:random",
        "//src/sim:stats",
        "//src/sim:system",
    ],
)

# TODO(jlowepower): Re-enable this once protobuf is available in gem5.
# gem5_cc_library(
#     name = "trace_gen",
#     srcs = ["trace_gen.cc"],
#     hdrs = ["trace_gen.hh"],
#     copts = ["-Ithird_party.gem5.develop.src.],
#     deps = [
#         ":debug_TrafficGen",
#         ":base_gen",
#         "//src/base:bitfield",
#         "//src/base:intmath",
#         "//src/base:trace",
#         "//src/mem:packet",
#         "//src/proto:packet_proto",
#         "//src/proto:protoio",
#         "//src/sim:core",
#         "//src/sim:cur_tick",
#     ],
# )
