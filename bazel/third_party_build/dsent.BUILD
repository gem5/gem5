load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "dsent",
    srcs = glob(["**/*.cc"], exclude = ["tests/**"]),
    hdrs = glob(["**/*.h"], exclude = ["tests/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
