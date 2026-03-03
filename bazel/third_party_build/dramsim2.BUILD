load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "dramsim2",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = glob(["*.h"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
