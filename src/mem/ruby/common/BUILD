load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "common_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

gem5_cc_library(
    name = "common",
    srcs = [
        "Address.cc",
        "BoolVec.cc",
        "Consumer.cc",
        "DataBlock.cc",
        "Histogram.cc",
        "IntVec.cc",
        "NetDest.cc",
        "SubBlock.cc",
        "WriteMask.cc",
    ],
    hdrs = [
        "Address.hh",
        "BoolVec.hh",
        "Consumer.hh",
        "DataBlock.hh",
        "ExpectedMap.hh",
        "Histogram.hh",
        "IntVec.hh",
        "MachineID.hh",
        "NetDest.hh",
        "Set.hh",
        "SubBlock.hh",
        "TriggerQueue.hh",
        "TypeDefines.hh",
        "WriteMask.hh",
    ],
    deps = [
        "//src/base:amo",
        "//src/base:bitfield",
        "//src/base:cprintf",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:stl_helpers",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem/ruby/protocol:MachineType_hh",
        "//src/mem/ruby/system:RubySystem_hh",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "Address_hh",
    hdrs = ["Address.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "BoolVec_hh",
    hdrs = ["BoolVec.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "Consumer_hh",
    hdrs = ["Consumer.hh"],
    deps = [
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "DataBlock_hh",
    hdrs = ["DataBlock.hh"],
    deps = [
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "ExpectedMap_hh",
    hdrs = ["ExpectedMap.hh"],
)

gem5_cc_library(
    name = "Histogram_hh",
    hdrs = ["Histogram.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "IntVec_hh",
    hdrs = ["IntVec.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "MachineID_hh",
    hdrs = ["MachineID.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/mem/ruby/protocol:MachineType_hh",
    ],
)

gem5_cc_library(
    name = "NetDest_hh",
    hdrs = ["NetDest.hh"],
    deps = [
        ":MachineID_hh",
        ":Set_hh",
    ],
)

gem5_cc_library(
    name = "Set_hh",
    hdrs = ["Set.hh"],
    deps = [
        ":TypeDefines_hh",
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "SubBlock_hh",
    hdrs = ["SubBlock.hh"],
    deps = [
        ":Address_hh",
        ":DataBlock_hh",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "TriggerQueue_hh",
    hdrs = ["TriggerQueue.hh"],
    deps = [
        ":Address_hh",
        ":DataBlock_hh",
    ],
)

gem5_cc_library(
    name = "TypeDefines_hh",
    hdrs = ["TypeDefines.hh"],
)

gem5_cc_library(
    name = "WriteMask_hh",
    hdrs = ["WriteMask.hh"],
    deps = [
        ":DataBlock_hh",
        ":TypeDefines_hh",
        "//src/base:amo",
        "//src/base:intmath",
    ],
)
