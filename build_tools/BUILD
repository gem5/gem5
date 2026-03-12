load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "blob",
    srcs = ["blob.py"],
)

py_library(
    name = "code_formatter",
    srcs = ["code_formatter.py"],
)

py_library(
    name = "grammar",
    srcs = ["grammar.py"],
    deps = [
        "//ext/ply",
    ],
)

py_binary(
    name = "debugflaghh",
    srcs = ["debugflaghh.py"],
    imports = ["."],
    deps = [
        ":code_formatter",
    ],
)

py_binary(
    name = "debugflagcc",
    srcs = ["debugflagcc.py"],
    imports = ["."],
    deps = [
        ":code_formatter",
    ],
)

py_binary(
    name = "enum_cc",
    srcs = ["enum_cc.py"],
    imports = [
        ".",
        "../",
        "../src/python",
    ],
    deps = [
        ":code_formatter",
        "//src:src_init",
        "//src/python:importer",
    ],
)

py_binary(
    name = "enum_hh",
    srcs = ["enum_hh.py"],
    imports = [
        ".",
        "../",
        "../src/python",
    ],
    deps = [
        ":code_formatter",
        "//src:src_init",
        "//src/python:importer",
    ],
)

py_binary(
    name = "sim_object_param_struct_cc",
    srcs = ["sim_object_param_struct_cc.py"],
    imports = [
        ".",
        "../",
        "../src/python",
    ],
    deps = [
        ":code_formatter",
        ":grammar",
        "//src:src_init",
        "//src/python:importer",
        "//src/python/m5:SimObject",
    ],
)

py_binary(
    name = "sim_object_param_struct_hh",
    srcs = ["sim_object_param_struct_hh.py"],
    imports = [
        ".",
        "../",
        "../src/python",
    ],
    deps = [
        ":code_formatter",
        ":grammar",
        "//src:src_init",
        "//src/python:importer",
        "//src/python/m5:SimObject",
    ],
)

py_binary(
    name = "marshal",
    srcs = ["marshal.py"],
    imports = ["."],
    deps = [
        ":blob",
        ":code_formatter",
    ],
)
