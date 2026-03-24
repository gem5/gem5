load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fputils",
    srcs = ["fp64.c", "fp80.c"],
    hdrs = ["fpbits.h"] + glob(["include/fputils/*.h"]),
    includes = ["include"],
    copts = ["-std=c99"],
    visibility = ["//visibility:public"],
)
