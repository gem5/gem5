load("@rules_python//python:defs.bzl", "py_binary")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "build_slicc",
    srcs = ["build_slicc.py"],
    imports = [
        "../../",
        "../../src/mem",
        "../../build_tools",
        "../../ext/ply",
    ],
    deps = [
        "//src/mem/slicc",
        "//ext/ply",
    ],
)

py_binary(
    name = "create_gdbxml",
    srcs = ["create_gdbxml.py"],
    imports = [
        "../../",
        "../../build_tools",
    ],
    deps = [
        "//build_tools:blob",
        "//build_tools:code_formatter",
    ],
)

# Export scripts just in case
exports_files([
    "build_slicc.py",
    "create_gdbxml.py",
])
