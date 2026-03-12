load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "classic",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_classic_cache_hierarchy",
        ":no_cache",
    ],
)

py_library(
    name = "abstract_classic_cache_hierarchy",
    srcs = ["abstract_classic_cache_hierarchy.py"],
    deps = [
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/utils:override",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "no_cache",
    srcs = ["no_cache.py"],
    deps = [
        ":abstract_classic_cache_hierarchy",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:Bridge",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "private_l1_shared_l2_cache_hierarchy",
    srcs = ["private_l1_shared_l2_cache_hierarchy.py"],
    deps = [
        ":abstract_classic_cache_hierarchy",
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies:abstract_two_level_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/classic/caches",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:Cache",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "private_l1_private_l2_cache_hierarchy",
    srcs = ["private_l1_private_l2_cache_hierarchy.py"],
    deps = [
        ":abstract_classic_cache_hierarchy",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies:abstract_two_level_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/classic/caches",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:Cache",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
    ],
)
