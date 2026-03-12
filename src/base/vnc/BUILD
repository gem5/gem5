load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_sim_object")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "vnc_init",
    srcs = ["__init__.py"],
    deps = [
        ":Vnc_py",
    ],
)

py_library(
    name = "Vnc_py",
    srcs = ["Vnc.py"],
    deps = [
        "//src/base:Graphics_py",
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "vnc_input_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:enum_ImageFormat_hh",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Graphics",
    ],
    sim_object_header = ":vnc",
    sim_object_module = "src.base.vnc.Vnc",
    sim_object_name = "VncInput",
    sim_object_py_lib = ":Vnc_py",
)

gem5_sim_object(
    name = "vnc_server_sim_object",
    extra_param_deps = [
        ":vnc_input_sim_object",
        "//src/base:socket",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Graphics",
    ],
    sim_object_header = ":vnc",
    sim_object_module = "src.base.vnc.Vnc",
    sim_object_name = "VncServer",
    sim_object_py_lib = ":Vnc_py",
)

gem5_cc_library(
    name = "vnc",
    srcs = [
        "vncinput.cc",
        "vncserver.cc",
    ],
    hdrs = [
        "vncinput.hh",
        "vncserver.hh",
    ],
    deps = [
        ":vnc_input_sim_object",
        ":vnc_server_sim_object",
        "//src/base:atomicio",
        "//src/base:debug_VNC",
        "//src/base:framebuffer",
        "//src/base:imgwriter",
        "//src/base:imgwriter_hh",
        "//src/base:logging",
        "//src/base:pollevent",
        "//src/base:socket",
        "//src/base:trace",
        "//src/base:types",
        "//src/base:circlebuf",
        "//src/base:compiler",
        "//src/base:output",
        "//src/sim:byteswap",
    ],
)
