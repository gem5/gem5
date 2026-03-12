load("@rules_python//python:defs.bzl", "py_library")


py_library(
    name = "util",
    srcs = [
        "attrdict.py",
        "convert.py",
        "multidict.py",
        "pybind.py",
        "terminal.py",
        "terminal_formatter.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util_base",
    ],
)

py_library(
    name = "util_base",
    srcs = [
        "__init__.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

py_library(
    name = "dot_writer",
    srcs = [
        "dot_writer.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util_base",
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
        # pydot is an external library and should be provided by the environment
    ],
)

py_library(
    name = "dot_writer_ruby",
    srcs = [
        "dot_writer_ruby.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util_base",
        # pydot is an external library and should be provided by the environment
    ],
)

py_library(
    name = "fdthelper",
    srcs = [
        "fdthelper.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util_base",
        "//src/python/m5:SimObject",
        "//src/python/m5/ext/pyfdt",
    ],
)
