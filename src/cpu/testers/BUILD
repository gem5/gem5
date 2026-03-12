load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "testers_init",
    srcs = ["__init__.py"],
    deps = [
        "//src/cpu/testers/rubytest:rubytest_init",
        "//src/cpu/testers/spatter_gen:spatter_gen_init",
        "//src/cpu/testers/traffic_gen:traffic_gen_init",
    ],
    visibility = ["//visibility:public"],
)
