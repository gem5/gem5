load("@rules_cc//cc:defs.bzl", "cc_library")
config_setting(
    name = "tracing_on",
    values = {
        "define": "tracing=on",
    },
)

package_group(
    name = "gem5_visibility",
    packages = [
        "//...",
    ],
)

exports_files(
    ["LICENSE"],
)

cc_library(
    name = "src_include",
    includes = ["src"],
    visibility = ["//visibility:public"],
)
