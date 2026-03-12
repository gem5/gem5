load("@rules_cc//cc:defs.bzl", "cc_library")


cc_library(
    name = "zfstream",
    srcs = ["zfstream.cc"],
    hdrs = ["zfstream.h"],
    visibility = ["//visibility:public"],
    includes = ["."],
    deps = [
        "@zlib//:zlib",
    ],
)
