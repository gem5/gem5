load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "pybind11",
    hdrs = glob(["include/pybind11/**/*.h"]),
    includes = ["include"],
    deps = ["@system_python//:headers"],
    visibility = ["//visibility:public"],
)
