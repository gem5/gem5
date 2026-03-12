load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "dram_interfaces",
    srcs = ["__init__.py"],
    deps = [
    ],
)

py_library(
    name = "ddr3",
    srcs = ["ddr3.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "ddr4",
    srcs = ["ddr4.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "ddr5",
    srcs = ["ddr5.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "gddr",
    srcs = ["gddr.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "hbm",
    srcs = ["hbm.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "hmc",
    srcs = ["hmc.py"],
    deps = [
        ":ddr3",
    ],
)

py_library(
    name = "lpddr2",
    srcs = ["lpddr2.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "lpddr3",
    srcs = ["lpddr3.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "lpddr5",
    srcs = ["lpddr5.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)

py_library(
    name = "wideio",
    srcs = ["wideio.py"],
    deps = [
        "//src/python/m5/objects:DRAMInterface",
    ],
)
