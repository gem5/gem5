load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "simulate",
    srcs = ["__init__.py"],
    deps = [
        ":exit_event",
        ":exit_event_generators",
        ":exit_handler",
        ":simulator",
    ],
)

py_library(
    name = "exit_event",
    srcs = ["exit_event.py"],
)

py_library(
    name = "exit_event_generators",
    srcs = ["exit_event_generators.py"],
    deps = [
        "//src/python/gem5/components/processors:abstract_processor",
        "//src/python/gem5/components/processors:switchable_processor",
        "//src/python/gem5/components/processors/spatter_gen",
        "//src/python/gem5/resources:looppoint",
        "//src/python/gem5/resources:resource",
        "//src/python/m5/stats",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "exit_handler",
    srcs = ["exit_handler.py"],
    deps = [
        ":exit_event",
        ":exit_event_generators",
        "//src/python/gem5/utils:override",
        "//src/python/m5:options",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "simulator",
    srcs = ["simulator.py"],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":exit_event",
        ":exit_handler",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/resources:resource",
        "//src/python/m5:options",
        "//src/python/m5:ticks",
        "//src/python/m5/ext/pystats",
        "//src/python/m5/stats",
        "//src/python/m5/util",
    ],
)
