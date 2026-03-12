load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "abstract_board",
    srcs = ["abstract_board.py"],
    imports = ["../../.."],
    deps = [
        "//src/python/gem5/resources:resource",
        "//src/python/m5",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:PciUpstream",
        "//src/python/m5/objects:Root",
        "//src/python/m5/objects:System",
        "//src/python/m5/objects:VoltageDomain",
        "//src/python/m5/objects:XBar",
    ],
)

py_library(
    name = "se_binary_workload",
    srcs = ["se_binary_workload.py"],
    deps = [
        ":abstract_board",
        "//src/python/gem5/components/processors:switchable_processor",
        "//src/python/gem5/resources:elfie",
        "//src/python/gem5/resources:looppoint",
        "//src/python/gem5/resources:resource",
        "//src/python/m5/objects:Process",
        "//src/python/m5/objects:Workload",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "simple_board",
    srcs = ["simple_board.py"],
    deps = [
        ":abstract_system_board",
        ":se_binary_workload",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/memory:abstract_memory_system",
        "//src/python/gem5/components/processors:abstract_processor",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:XBar",
    ],
)

py_library(
    name = "test_board",
    srcs = ["test_board.py"],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":abstract_board",
        ":abstract_system_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/memory:abstract_memory_system",
        "//src/python/gem5/components/processors:abstract_generator",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:XBar",
    ],
)

py_library(
    name = "abstract_system_board",
    srcs = ["abstract_system_board.py"],
    deps = [
        ":abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:System",
    ],
)

py_library(
    name = "boards",
    srcs = ["__init__.py"],
    deps = [
    ],
)

py_library(
    name = "mem_mode",
    srcs = ["mem_mode.py"],
    deps = [
    ],
)

py_library(
    name = "kernel_disk_workload",
    srcs = ["kernel_disk_workload.py"],
    deps = [
        ":abstract_board",
        "//src/python/gem5/resources:resource",
        "//src/python/m5",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "arm_board",
    srcs = ["arm_board.py"],
    deps = [
        ":abstract_board",
        ":kernel_disk_workload",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/classic:no_cache",
        "//src/python/gem5/components/memory:abstract_memory_system",
        "//src/python/gem5/components/processors:abstract_processor",
        "//src/python/gem5/resources:resource",
        "//src/python/gem5/utils:override",
        "//src/python/gem5/utils:requires",
        "//src/python/m5",
        "//src/python/m5/objects:ArmFsWorkload",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:Bridge",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:DiskImage",
        "//src/python/m5/objects:GenericTimer",
        "//src/python/m5/objects:PciUpstream",
        "//src/python/m5/objects:RealView",
        "//src/python/m5/objects:Root",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:Terminal",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/objects:VirtIOBlock",
        "//src/python/m5/objects:Vnc",
        "//src/python/m5/objects:VoltageDomain",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
    ],
)
