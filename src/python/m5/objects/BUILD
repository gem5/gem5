load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "m5_objects_generator")
load("//bazel:sim_objects.bzl", "SIM_OBJECTS")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "objects",
    srcs = [
        "__init__.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":all_objects",
    ],
)

py_library(
    name = "SimObject",
    srcs = [
        "SimObject.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//src/python/m5:SimObject",
    ],
)

m5_objects_generator(
    name = "all_objects",
    simobjects = SIM_OBJECTS,
)
