load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "caches",
    srcs = ["__init__.py"],
)

py_library(
    name = "directory",
    srcs = ["directory.py"],
    deps = [
        "//src/python/m5/objects:DirectoryMemory",
        "//src/python/m5/objects:MESI_Two_Level_Directory_Controller",
        "//src/python/m5/objects:MessageBuffer",
    ],
)

py_library(
    name = "l1_cache",
    srcs = ["l1_cache.py"],
    deps = [
        "//src/python/gem5:isas",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:MESI_Two_Level_L1Cache_Controller",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/objects:RubyCache",
        "//src/python/m5/objects:RubyPrefetcher",
    ],
)

py_library(
    name = "l2_cache",
    srcs = ["l2_cache.py"],
    deps = [
        "//src/python/m5/objects:MESI_Two_Level_L2Cache_Controller",
        "//src/python/m5/objects:MessageBuffer",
        "//src/python/m5/objects:RubyCache",
    ],
)

py_library(
    name = "dma_controller",
    srcs = ["dma_controller.py"],
    deps = [
        "//src/python/m5/objects:MESI_Two_Level_DMA_Controller",
        "//src/python/m5/objects:MessageBuffer",
    ],
)
