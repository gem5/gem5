load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "systemc",
    srcs = glob(["src/**/*.cc", "src/**/*.c", "src/**/*.cpp"],
                exclude = ["src/**/qt/**", "src/**/pthreads/**"]),
    hdrs = glob(["src/**/*.h", "src/**/*.hh"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = glob(["src/**/*.h", "src/**/*.hh"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
)
