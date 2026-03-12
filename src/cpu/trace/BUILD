load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "trace_init",
    srcs = ["__init__.py"],
    deps = [
        ":TraceCPU_py",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "TraceCPU_py",
    srcs = ["TraceCPU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)
