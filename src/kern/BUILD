load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag")

py_library(
    name = "kern_init",
    srcs = ["__init__.py"],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_DebugPrintf",
    desc = "Flag for DebugPrintf calls",
    flag = "DebugPrintf",
)

gem5_debug_flag(
    name = "debug_Printf",
    desc = "Flag for Printf calls",
    flag = "Printf",
)

gem5_cc_library(
    name = "operatingsystem",
    srcs = ["operatingsystem.cc"],
    hdrs = ["operatingsystem.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "system_events",
    srcs = ["system_events.cc"],
    hdrs = ["system_events.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:trace",
        "//src/cpu:debug_PCEvent",
        "//src/cpu:pc_event",
        "//src/cpu:thread_context",
    ],
)
