load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "components",
    srcs = ["__init__.py"],
    deps = [
        "//src/python/gem5/components/boards",
        "//src/python/gem5/components/cachehierarchies",
    ],
)
