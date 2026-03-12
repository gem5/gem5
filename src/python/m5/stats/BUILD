load("@rules_python//python:defs.bzl", "py_library")


py_library(
    name = "stats",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":gem5stats",
        "//src/python/m5/objects:Root",
        "//src/python/m5/params",
        "//src/python/m5/util",
        "//src/python/pybind11:gem5_pybind11_core",
        "//src/python/pybind11:stats",
    ],
)

py_library(
    name = "gem5stats",
    srcs = ["gem5stats.py"],
    visibility = ["//visibility:private"],
    deps = [
        "//src/python/m5/ext/pystats",
        "//src/python/m5/objects:Root",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
        "//src/python/pybind11:stats",
    ],
)
