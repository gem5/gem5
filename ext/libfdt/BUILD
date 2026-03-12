load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "libfdt",
    srcs = [
        "fdt.c",
        "fdt_empty_tree.c",
        "fdt_ro.c",
        "fdt_rw.c",
        "fdt_strerror.c",
        "fdt_sw.c",
        "fdt_wip.c",
    ],
    hdrs = [
        "fdt.h",
        "libfdt.h",
        "libfdt_env.h",
        "libfdt_internal.h",
    ],
    copts = ["-Wno-tautological-compare"],
    includes = ["."],
)
