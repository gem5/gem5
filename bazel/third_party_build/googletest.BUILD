load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "googletest",
    srcs = glob(
        ["googletest/src/*.cc"],
        exclude = [
            "googletest/src/gtest_main.cc",
            "googletest/src/gtest-all.cc",
        ],
    ),
    hdrs = glob(["googletest/include/**/*.h", "googletest/src/*.h"]),
    includes = ["googletest/include", "googletest"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtest_main",
    srcs = ["googletest/src/gtest_main.cc"],
    deps = [":googletest"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "googlemock",
    srcs = glob(
        ["googlemock/src/*.cc"],
        exclude = [
            "googlemock/src/gmock_main.cc",
            "googlemock/src/gmock-all.cc",
        ],
    ),
    hdrs = glob(["googlemock/include/**/*.h"]),
    includes = ["googlemock/include"],
    deps = [":googletest"],
    visibility = ["//visibility:public"],
)
