load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")
load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "get_default_gem5_copts")

package(default_visibility = ["//visibility:public"])

exports_files(["_m5.cc"])

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    deps = [
        "//src/python/m5:m5_init",
    ],
)

py_library(
    name = "importer",
    srcs = [
        "importer.py",
    ],
    deps = [
        ":python_init",
    ],
)

pybind_library(
    name = "pybind_init",
    hdrs = ["pybind_init.hh"],
)

pybind_extension(
    name = "_m5",
    srcs = [
        "_m5.cc",
    ],
    copts = get_default_gem5_copts(),
    visibility = ["//visibility:public"],
    deps = [
        ":pybind_init",
        "//bazel/generated:tags",
        "//src/base:cprintf",
        "//src/base:date",
        "//src/base:statistics",
        "//src/base:version",
        "//src/python/pybind11:debug",
        "//src/python/pybind11:event",
        "//src/python/pybind11:gem5_pybind11_core",
        "//src/python/pybind11:port",
        "//src/python/pybind11:pybind",
        "//src/python/pybind11:stats",
        "//src/sim:cur_tick",
        "//src/sim:init",
        "//src/sim:param_sim_object_py",
        "//src/sim:python_py",
        "//src/sim:sim_object",
        "//:src_include",
    ],
)
