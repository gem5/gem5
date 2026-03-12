load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "dnet",
    srcs = glob(["*.c"], allow_empty = True),
    hdrs = ["os.h"],
    textual_hdrs = glob(
        ["*.h"],
        exclude = ["os.h"],
    ),
)
