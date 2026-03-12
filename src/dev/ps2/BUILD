load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_PS2",
    flag = "PS2",
)

gem5_cc_library(
    name = "types",
    srcs = ["types.cc"],
    hdrs = ["types.hh"],
    deps = [
        "//ext/x11keysym",
        "//src/base:bitunion",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "device",
    srcs = ["device.cc"],
    hdrs = ["device.hh"],
    deps = [
        ":debug_PS2",
        ":ps2_device_sim_object",
        ":types",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:serialize",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "keyboard",
    srcs = ["keyboard.cc"],
    hdrs = ["keyboard.hh"],
    deps = [
        ":debug_PS2",
        ":device",
        ":ps2_keyboard_sim_object",
        ":types",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base/vnc:vnc",
    ],
)

gem5_cc_library(
    name = "mouse",
    srcs = ["mouse.cc"],
    hdrs = ["mouse.hh"],
    deps = [
        ":debug_PS2",
        ":device",
        ":ps2_mouse_sim_object",
        ":types",
        "//src/base:bitunion",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "touchkit",
    srcs = ["touchkit.cc"],
    hdrs = ["touchkit.hh"],
    deps = [
        ":debug_PS2",
        ":device",
        ":mouse",
        ":ps2_touch_kit_sim_object",
        ":types",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base/vnc:vnc",
    ],
)

py_library(
    name = "PS2_py",
    srcs = ["PS2.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ps2_device_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.ps2.PS2",
    sim_object_name = "PS2Device",
    sim_object_py_lib = ":PS2_py",
)

gem5_sim_object(
    name = "ps2_keyboard_sim_object",
    extra_deps = [
        "//src/base/vnc:vnc",
    ],
    extra_param_deps = [
        ":ps2_device_sim_object",
        "//src/base/vnc:vnc_input_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Graphics",
        "//src/python/m5/objects:Vnc",
        "//src/python/m5/objects:VncInput",
    ],
    sim_object_header = ":keyboard",
    sim_object_module = "src.dev.ps2.PS2",
    sim_object_name = "PS2Keyboard",
    sim_object_py_lib = ":PS2_py",
)

gem5_sim_object(
    name = "ps2_mouse_sim_object",
    extra_param_deps = [
        ":ps2_device_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":mouse",
    sim_object_module = "src.dev.ps2.PS2",
    sim_object_name = "PS2Mouse",
    sim_object_py_lib = ":PS2_py",
)

gem5_sim_object(
    name = "ps2_touch_kit_sim_object",
    extra_deps = [
        "//src/base/vnc:vnc",
    ],
    extra_param_deps = [
        ":ps2_device_sim_object",
        "//src/base/vnc:vnc_input_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Graphics",
        "//src/python/m5/objects:Vnc",
        "//src/python/m5/objects:VncInput",
    ],
    sim_object_header = ":touchkit",
    sim_object_module = "src.dev.ps2.PS2",
    sim_object_name = "PS2TouchKit",
    sim_object_py_lib = ":PS2_py",
)
