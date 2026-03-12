load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "resources",
    srcs = ["__init__.py"],
    deps = [
        ":client",
        ":downloader",
        ":elfie",
        ":looppoint",
        ":md5_utils",
        ":resource",
        ":workload",
    ],
)

py_library(
    name = "client",
    srcs = ["client.py"],
    deps = [
        "//src/python:_m5",
        "//src/python/gem5:gem5_default_config",
        "//src/python/gem5/resources/client_api",
    ],
)

py_library(
    name = "downloader",
    srcs = ["downloader.py"],
    deps = [
        "//src/python/gem5/utils:progress_bar",
    ],
)

py_library(
    name = "elfie",
    srcs = ["elfie.py"],
    deps = [
        "//src/python/m5/objects:PcCountTracker",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "looppoint",
    srcs = ["looppoint.py"],
    deps = [
        "//src/python/m5",
        "//src/python/m5/objects:PcCountTracker",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "md5_utils",
    srcs = ["md5_utils.py"],
)

py_library(
    name = "resource",
    srcs = ["resource.py"],
    deps = [
        ":client",
        ":downloader",
        ":elfie",
        ":md5_utils",
        "//src/python:_m5",
        "//src/python/gem5/utils:filelock",
        "//src/python/gem5/utils:socks_ssl_context",
    ],
)

py_library(
    name = "workload",
    srcs = ["workload.py"],
    deps = [
        ":resource",
    ],
)
