load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "libdrampower",
    srcs = glob([
        "src/*.cc",
        "src/libdrampower/*.cc",
    ]),
    hdrs = glob([
        "src/*.h",
        "src/libdrampower/*.h",
    ]),
    copts = [
        "-fexceptions",
        "-Wno-error=nonnull",
    ],
    features = [
        "-use_header_modules",
    ],
    includes = ["src"],
    deps = [
    ],
)
