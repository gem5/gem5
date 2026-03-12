load("@rules_python//python:defs.bzl", "py_library")


py_library(
    name = "gem5",
    srcs = ["__init__.py"],
    visibility = ["//visibility:private"],
)

py_library(
    name = "coherence_protocol",
    srcs = ["coherence_protocol.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "isas",
    srcs = ["isas.py"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "runtime",
    srcs = ["runtime.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/m5:defines",
        "//src/python/m5/util:util_base",
    ],
)

py_library(
    name = "gem5_default_config",
    srcs = ["gem5_default_config.py"],
    visibility = ["//visibility:public"],
)
