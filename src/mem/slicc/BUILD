load("@rules_python//python:defs.bzl", "py_library")


py_library(
    name = "slicc",
    srcs = glob([
        "*.py",
    ]),
    visibility = ["//visibility:public"],
    imports = [".."],
    deps = [
        "//build_tools:code_formatter",
        "//build_tools:grammar",
        "//src/mem/slicc/ast",
        "//src/mem/slicc/generate",
        "//src/mem/slicc/symbols",
    ],
)
