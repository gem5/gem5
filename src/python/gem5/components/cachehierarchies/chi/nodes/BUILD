load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "nodes",
    srcs = ["__init__.py"],
)

py_library(
    name = "abstract_node",
    srcs = ["abstract_node.py"],
    deps = [
        "//src/python/gem5:isas",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/gem5/components/processors:cpu_types",
        "//src/python/m5/objects:CHI_Cache_Controller",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/objects:Network",
    ],
)

py_library(
    name = "directory",
    srcs = ["directory.py"],
    deps = [
        ":abstract_node",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Network",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "dma_requestor",
    srcs = ["dma_requestor.py"],
    deps = [
        ":abstract_node",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "l1_cache",
    srcs = ["l1_cache.py"],
    deps = [
        ":abstract_node",
        "//src/python/gem5:isas",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Network",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "l2_cache",
    srcs = ["l2_cache.py"],
    deps = [
        ":abstract_node",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Network",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "memory_controller",
    srcs = ["memory_controller.py"],
    deps = [
        ":abstract_node",
        "//src/python/m5/objects:CHI_Memory_Controller",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/objects:Network",
        "//src/python/m5/params",
    ],
)
