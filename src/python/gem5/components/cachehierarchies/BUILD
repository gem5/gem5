load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "cachehierarchies",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_cache_hierarchy",
        ":abstract_three_level_cache_hierarchy",
        ":abstract_two_level_cache_hierarchy",
    ],
)

py_library(
    name = "abstract_cache_hierarchy",
    srcs = ["abstract_cache_hierarchy.py"],
    deps = [
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/m5/objects:Root",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/util:fdthelper",
    ],
)

py_library(
    name = "abstract_two_level_cache_hierarchy",
    srcs = ["abstract_two_level_cache_hierarchy.py"],
)

py_library(
    name = "abstract_three_level_cache_hierarchy",
    srcs = ["abstract_three_level_cache_hierarchy.py"],
)
