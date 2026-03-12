load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "functional",
    srcs = ["functional.cc"],
    hdrs = ["functional.hh"],
    deps = [
        "//src/mem:backdoor",
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "timing",
    srcs = ["timing.cc"],
    hdrs = ["timing.hh"],
    deps = [
        "//src/mem:packet",
    ],
)

gem5_cc_library(
    name = "atomic",
    srcs = ["atomic.cc"],
    hdrs = ["atomic.hh"],
    deps = [
        "//src/base:trace",
        "//src/mem:backdoor",
        "//src/mem:packet",
    ],
)
