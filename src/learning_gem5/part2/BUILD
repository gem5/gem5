load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "part2_init",
    srcs = ["__init__.py"],
    deps = [
        ":HelloObject_py",
        ":SimpleMemobj_py",
        ":SimpleObject_py",
        ":SimpleCache_py",
    ],
)

gem5_sim_object(
    name = "simple_object_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_hh",
        "//src/sim:sim_object_sim_object",
    ],
    sim_object_header = ":simple_object",
    sim_object_module = "src.learning_gem5.part2.SimpleObject",
    sim_object_name = "SimpleObject",
    sim_object_py_lib = ":SimpleObject_py",
)

py_library(
    name = "SimpleObject_py",
    srcs = ["SimpleObject.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "SimpleMemobj_py",
    srcs = ["SimpleMemobj.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "SimpleCache_py",
    srcs = ["SimpleCache.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "simple_object",
    srcs = ["simple_object.cc"],
    hdrs = ["simple_object.hh"],
    deps = [
        ":simple_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
)

gem5_debug_flag(
    name = "debug_HelloExample",
    desc = "This is an example debug flag",
    flag = "HelloExample",
)

gem5_sim_object(
    name = "hello_object_sim_object",
    extra_param_deps = [
        ":goodbye_object",
        ":goodbye_object_sim_object",
        "//src/sim:sim_object_hh",
        "//src/sim:sim_object_sim_object",
    ],
    sim_object_header = ":hello_object",
    sim_object_module = "src.learning_gem5.part2.HelloObject",
    sim_object_name = "HelloObject",
    sim_object_py_lib = ":HelloObject_py",
)

py_library(
    name = "HelloObject_py",
    srcs = ["HelloObject.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "hello_object",
    srcs = ["hello_object.cc"],
    hdrs = ["hello_object.hh"],
    deps = [
        ":debug_HelloExample",
        ":goodbye_object",
        ":goodbye_object_sim_object",
        ":hello_object_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "goodbye_object_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_hh",
        "//src/sim:sim_object_sim_object",
    ],
    sim_object_header = ":goodbye_object",
    sim_object_module = "src.learning_gem5.part2.HelloObject",
    sim_object_name = "GoodbyeObject",
    sim_object_py_lib = ":HelloObject_py",
)

gem5_cc_library(
    name = "goodbye_object",
    srcs = ["goodbye_object.cc"],
    hdrs = ["goodbye_object.hh"],
    deps = [
        ":debug_HelloExample",
        ":goodbye_object_sim_object",
        "//src/base:trace",
        "//src/sim:sim_exit",
        "//src/sim:sim_object_hh",
    ],
)
