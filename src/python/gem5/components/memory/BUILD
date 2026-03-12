load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "__init__",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_memory_system",
        ":hbm",
        ":multi_channel",
        ":single_channel",
    ],
)

py_library(
    name = "abstract_memory_system",
    srcs = ["abstract_memory_system.py"],
    deps = [
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/objects:Root",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "simple",
    srcs = ["simple.py"],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":abstract_memory_system",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "memory_py",
    srcs = ["memory.py"],
    deps = [
        ":abstract_memory_system",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "multi_channel",
    srcs = ["multi_channel.py"],
    deps = [
        ":abstract_memory_system",
        ":memory_py",
        "//src/python/gem5/components/memory/dram_interfaces:ddr3",
        "//src/python/gem5/components/memory/dram_interfaces:ddr4",
        "//src/python/gem5/components/memory/dram_interfaces:hbm",
        "//src/python/gem5/components/memory/dram_interfaces:lpddr3",
    ],
)

py_library(
    name = "single_channel",
    srcs = ["single_channel.py"],
    deps = [
        ":abstract_memory_system",
        ":memory_py",
        "//src/python/gem5/components/memory/dram_interfaces:ddr3",
        "//src/python/gem5/components/memory/dram_interfaces:ddr4",
        "//src/python/gem5/components/memory/dram_interfaces:ddr5",
        "//src/python/gem5/components/memory/dram_interfaces:hbm",
        "//src/python/gem5/components/memory/dram_interfaces:lpddr3",
    ],
)

py_library(
    name = "hbm",
    srcs = ["hbm.py"],
    deps = [
        ":abstract_memory_system",
        ":memory_py",
        "//src/python/gem5/components/memory/dram_interfaces:hbm",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:HBMCtrl",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/params",
    ],
)
