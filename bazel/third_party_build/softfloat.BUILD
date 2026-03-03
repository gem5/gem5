load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "softfloat",
    srcs = glob(
        ["*.c"],
        exclude = ["*_test.c"],
    ),
    hdrs = glob(["*.h"]),
    copts = [
        "-Wno-implicit-fallthrough",
        "-Wno-unused-variable",
    ],
    visibility = ["//visibility:public"],
)
