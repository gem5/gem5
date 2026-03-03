load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "dramsim3",
    srcs = glob(["src/*.cc"]),
    hdrs = glob(["src/*.h"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = glob(["src/*.h"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
)
