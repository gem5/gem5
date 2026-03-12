load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "threadinfo",
    hdrs = ["threadinfo.hh"],
    deps = [
        "//src/base:types",
        "//src/cpu:thread_context",
        "//src/mem:translating_port_proxy",
        "//src/sim:system",
    ],
)
