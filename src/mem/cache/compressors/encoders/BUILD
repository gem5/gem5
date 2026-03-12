load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "base_encoder",
    hdrs = ["base.hh"],
    deps = [
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "huffman_encoder",
    srcs = ["huffman.cc"],
    hdrs = ["huffman.hh"],
    deps = [
        ":base_encoder",
        "//src/base:compiler",
        "//src/base:logging",
    ],
)
