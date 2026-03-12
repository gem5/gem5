load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_Stats",
    desc = "Statistics management",
    flag = "Stats",
)

gem5_cc_library(
    name = "group",
    srcs = [
        "group.cc",
    ],
    hdrs = [
        "group.hh",
    ],
    deps = [
        ":debug_Stats",
        ":info",
        ":units",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:named",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "units",
    hdrs = [
        "units.hh",
    ],
    deps = [
        "//src/base:compiler",
        "//src/base:cprintf",
    ],
)

gem5_cc_library(
    name = "info",
    srcs = ["info.cc"],
    hdrs = ["info.hh"],
    deps = [
        ":debug_Stats",
        ":storage",
        ":types",
        ":units",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:debug",
        "//src/base:flags",
        "//src/base:logging",
    ],
)

gem5_cc_test(
    name = "info_test",
    srcs = [
        "info.test.cc",
    ],
    deps = [
        ":info",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "types",
    hdrs = [
        "types.hh",
    ],
    deps = [
        "//src/base:compiler",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "storage",
    srcs = [
        "storage.cc",
    ],
    hdrs = [
        "storage.hh",
    ],
    deps = [
        ":types",
        "//src/base:cast",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_test(
    name = "storage_test",
    srcs = [
        "storage.test.cc",
    ],
    deps = [
        ":storage",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_test(
    name = "group_test",
    srcs = [
        "group.test.cc",
    ],
    deps = [
        ":group",
        ":info",
        ":output",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_test(
    name = "units_test",
    srcs = [
        "units.test.cc",
    ],
    deps = [
        ":units",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "output",
    hdrs = [
        "output.hh",
    ],
    deps = [
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "text",
    srcs = [
        "text.cc",
    ],
    hdrs = [
        "text.hh",
    ],
    deps = [
        ":info",
        ":output",
        ":types",
        "//src/base:cast",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:output",
    ],
)
