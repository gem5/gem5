load("@rules_python//python:defs.bzl", "py_binary")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "run_simple",
    srcs = ["run_simple.py"],
    deps = [
        "//src/python/m5",
        "//src/learning_gem5/part2:simple_object_sim_object",
    ],
)
