load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "utils",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "filelock",
    srcs = ["filelock.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "override",
    srcs = ["override.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "progress_bar",
    srcs = ["progress_bar.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "requires",
    srcs = ["requires.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5:isas",
        "//src/python/gem5:runtime",
    ],
)

py_library(
    name = "simpoint",
    srcs = ["simpoint.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/gem5/resources:resource",
        "//src/python/m5/util:util_base",
    ],
)

py_library(
    name = "socks_ssl_context",
    srcs = ["socks_ssl_context.py"],
    visibility = ["//visibility:public"],
)
