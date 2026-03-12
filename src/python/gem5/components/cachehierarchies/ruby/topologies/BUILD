load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "ruby",
    srcs = ["__init__.py"],
    deps = [
        ":simple_pt2pt",
    ],
)

py_library(
    name = "simple_pt2pt",
    srcs = ["simple_pt2pt.py"],
    deps = [
        "//src/python/m5/objects:SimpleLink",
        "//src/python/m5/objects:SimpleNetwork",
    ],
)
