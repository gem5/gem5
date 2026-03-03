load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "iostream3",
    srcs = ["zfstream.cc"],
    hdrs = ["zfstream.h"],
    includes = ["."],
    linkopts = ["-lz"],
    visibility = ["//visibility:public"],
)
