load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "src_init",
    srcs = ["__init__.py"],
    imports = ["."],
    deps = [
        "//src/arch:arch_init",
        "//src/base:base_init",
        "//src/cpu:cpu_init",
        "//src/dev:dev_init",
        "//src/kern:kern_init",
        "//src/mem:mem_init",
        "//src/python:python_init",
        "//src/sim:sim_init",
        "//src/learning_gem5:learning_gem5_init",
    ],
)
