load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "gem5",
    hdrs = [
        "gem5/hypercall_ids.h",
        "gem5/m5ops.h",
    ],
    includes = [
        ".",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gem5_asm_generic",
    ],
)

cc_library(
    name = "gem5_asm_generic",
    hdrs = [
        "gem5/asm/generic/m5ops.h",
    ],
    includes = [
        ".",
    ],
    visibility = ["//visibility:public"],
)
