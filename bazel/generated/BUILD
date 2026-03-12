load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

genrule(
    name = "tags_gen",
    srcs = ["//util:cpt_upgrader.py"],
    outs = ["tags.cc"],
    cmd = "python3 $(location //util:cpt_upgrader.py) --get-cc-file > $@",
)

gem5_cc_library(
    name = "tags",
    srcs = [":tags_gen"],
    deps = [
    ],
)
