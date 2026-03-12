load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "ruby",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_ruby_cache_hierarchy",
    ],
)

py_library(
    name = "abstract_ruby_cache_hierarchy",
    srcs = ["abstract_ruby_cache_hierarchy.py"],
    deps = [
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/utils:override",
    ],
)

py_library(
    name = "mesi_two_level_cache_hierarchy",
    srcs = ["mesi_two_level_cache_hierarchy.py"],
    deps = [
        ":abstract_ruby_cache_hierarchy",
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies:abstract_two_level_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/ruby/caches/mesi_two_level:directory",
        "//src/python/gem5/components/cachehierarchies/ruby/caches/mesi_two_level:dma_controller",
        "//src/python/gem5/components/cachehierarchies/ruby/caches/mesi_two_level:l1_cache",
        "//src/python/gem5/components/cachehierarchies/ruby/caches/mesi_two_level:l2_cache",
        "//src/python/gem5/components/cachehierarchies/ruby/topologies:simple_pt2pt",
        "//src/python/gem5/utils:override",
        "//src/python/gem5/utils:requires",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/objects:Sequencer",
    ],
)
