load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "isa_parser",
    srcs = glob([
        "*.py",
    ]),
    main = "isa_parser.py",
    imports = [
        "../../../build_tools",
        "../../../ext/ply",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//build_tools:code_formatter",
        "//build_tools:grammar",
    ],
)
