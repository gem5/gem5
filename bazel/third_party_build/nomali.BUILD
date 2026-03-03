load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "nomali",
    srcs = glob(["lib/*.cc"]),
    hdrs = glob(["lib/*.hh", "lib/*.h", "include/**/*.h"]),
    includes = ["include", "lib"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
