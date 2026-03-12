load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library")

py_library(
    name = "guest_abi_init",
    srcs = ["__init__.py"],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "guest_abi",
    hdrs = [
        "definition.hh",
        "dispatch.hh",
        "layout.hh",
        "varargs.hh",
    ],
    deps = [
        "//src/base:compiler",
    ],
)
