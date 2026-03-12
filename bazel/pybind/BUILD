load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")
load("//bazel:rules.bzl", "get_m5_pybind_deps")
load("//bazel:sim_objects.bzl", "SIM_OBJECTS")

pybind_extension(
    name = "_m5",
    srcs = [
        "//src/python:_m5.cc",
    ],
    visibility = ["//visibility:public"],
    linkopts = select({
        "@platforms//os:macos": ["-Wl,-all_load"],
        "//conditions:default": [],
    }),
    deps = [
        "//bazel/generated:tags",
        "//src/base:cprintf",
        "//src/base:date",
        "//src/base:statistics",
        "//src/base:version",
        "//src/python:pybind_init",
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
    ] + get_m5_pybind_deps(SIM_OBJECTS),
)
