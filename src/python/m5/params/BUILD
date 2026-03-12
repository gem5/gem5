load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_enum_generator")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "params",
    srcs = glob(["*.py"]),
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5:ticks",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "base_params",
    srcs = ["base_params.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "enum_params",
    srcs = ["enum_params.py"],
    deps = [
        ":base_params",
    ],
)

gem5_enum_generator(
    name = "enum_ByteOrder",
    enum_module = "src.python.m5.params",
    enum_name = "ByteOrder",
    enum_py_lib = ":enum_params",
)
