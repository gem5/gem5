load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

config_setting(
    name = "macos",
    constraint_values = ["@platforms//os:macos"],
)

config_setting(
    name = "tracing_on",
    values = {"define": "tracing=on"},
)

cc_library(
    name = "kvm_compat",
    hdrs = [
        "compat/kvm_compat.hh",
        "compat/linux/kvm.h",
        "compat/linux/perf_event.h",
    ],
    includes = ["compat"],
)

exports_files([
    "rules.bzl",
    "sim_objects.bzl",
])
