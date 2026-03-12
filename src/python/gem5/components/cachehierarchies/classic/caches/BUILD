load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "caches",
    srcs = glob(["*.py"]),
    deps = [
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:Cache",
        "//src/python/m5/objects:Prefetcher",
    ],
)
