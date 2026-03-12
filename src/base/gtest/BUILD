load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "gtest",
    testonly = 1,
    srcs = [
        "logging.cc",
        "logging_mock.cc",
    ],
    hdrs = [
        "cur_tick_fake.hh",
        "logging.hh",
        "serialization_fixture.hh",
    ],
    deps = [
        "//ext/googletest:gtest",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:cur_tick",
        "//src/sim:serialize",
    ],
)
