load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "libfdt",
    srcs = [
        "fdt.c",
        "fdt_ro.c",
        "fdt_rw.c",
        "fdt_sw.c",
        "fdt_wip.c",
        "fdt_empty_tree.c",
        "fdt_strerror.c",
    ],
    hdrs = [
        "fdt.h",
        "libfdt.h",
        "libfdt_env.h",
        "libfdt_internal.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = [
        "fdt.h",
        "libfdt.h",
        "libfdt_env.h",
    ],
    visibility = ["//visibility:public"],
)
