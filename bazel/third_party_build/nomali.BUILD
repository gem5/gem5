load("@rules_cc//cc:defs.bzl", "cc_library")

# Internal lib headers exposed as nomali/lib/*.h for
# #include "nomali/lib/mali_midg_regmap.h"
cc_library(
    name = "nomali_lib_hdrs",
    hdrs = glob(["lib/*.hh", "lib/*.h"]),
    include_prefix = "nomali",
    strip_include_prefix = "",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "nomali",
    srcs = glob(["lib/*.cc"]),
    deps = [":nomali_lib_hdrs"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    deps = [":nomali_lib_hdrs"],
    visibility = ["//visibility:public"],
)
