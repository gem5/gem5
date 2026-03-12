load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "spatter_gen",
    srcs = ["__init__.py"],
    deps = [
        ":spatter_generator",
        ":spatter_generator_core",
        ":spatter_kernel",
    ],
)

py_library(
    name = "spatter_generator",
    srcs = ["spatter_generator.py"],
    deps = [
        ":spatter_generator_core",
        ":spatter_kernel",
        "//src/python/gem5/components/processors:abstract_generator",
        "//src/python/gem5/utils",
        "//src/python/m5/objects:SpatterGen",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "spatter_generator_core",
    srcs = ["spatter_generator_core.py"],
    deps = [
        ":spatter_kernel",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/gem5/components/processors:abstract_generator_core",
        "//src/python/m5/objects:SpatterGen",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "spatter_kernel",
    srcs = ["spatter_kernel.py"],
    deps = [
        "//src/python/gem5/components/processors:abstract_generator",
        "//src/python/gem5/utils",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:SpatterGen",
        "//src/python/m5/objects:VoltageDomain",
        "//src/python/m5/stats",
        "//src/python/m5/util",
    ],
)
