load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "m5_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)


py_library(
    name = "proxy",
    srcs = ["proxy.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "ticks",
    srcs = ["ticks.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/m5/util",
    ],
)

py_library(
    name = "citations",
    srcs = ["citations.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":options",
    ],
)

py_library(
    name = "options",
    srcs = ["options.py"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

py_library(
    name = "SimObject",
    srcs = ["SimObject.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":citations",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "defines",
    srcs = ["defines.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "event",
    srcs = ["event.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/pybind11:event",
        "//src/python/pybind11:gem5_pybind11_core",
    ],
)

# This is only included to make the __init__.py file happy. You should not use any functions from
# this file.
py_library(
    name = "main",
    srcs = ["main.py"],
    visibility = ["//visibility:private"],
    deps = [
    ],
)

py_library(
    name = "m5",
    srcs = ["__init__.py"],
    imports = [".."],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":core",
        ":defines",
        ":event",
        ":main",
        ":simulate",
        "//src/python/m5/objects",
        "//src/python/m5/params",
        "//src/python/m5/stats",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "core",
    srcs = ["core.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":options",
        "//src/python/pybind11:gem5_pybind11_core",
    ],
)

py_library(
    name = "simulate",
    srcs = ["simulate.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":SimObject",
        ":citations",
        ":ticks",
        "//src/python:_m5",
        "//src/python/m5/objects:Root",
        "//src/python/m5/params",
        "//src/python/m5/stats",
        "//src/python/m5/util",
        "//src/python/m5/util:dot_writer",
        "//src/python/m5/util:dot_writer_ruby",
    ],
)

py_library(
    name = "debug",
    srcs = ["debug.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/m5/util",
        "//src/python/pybind11:debug",
    ],
)

py_library(
    name = "trace",
    srcs = ["trace.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/pybind11:debug",
    ],
)
