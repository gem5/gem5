load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "chi",
    srcs = ["__init__.py"],
)

py_library(
    name = "private_l1_cache_hierarchy",
    srcs = ["private_l1_cache_hierarchy.py"],
    deps = [
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:directory",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:dma_requestor",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:l1_cache",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:memory_controller",
        "//src/python/gem5/components/cachehierarchies/ruby:abstract_ruby_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/ruby/topologies:simple_pt2pt",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/gem5/utils:override",
        "//src/python/gem5/utils:requires",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/objects:Sequencer",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "private_l1_private_l2_cache_hierarchy",
    srcs = ["private_l1_private_l2_cache_hierarchy.py"],
    deps = [
        "//src/python/gem5:coherence_protocol",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/cachehierarchies:abstract_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies:abstract_two_level_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:directory",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:dma_requestor",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:l1_cache",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:l2_cache",
        "//src/python/gem5/components/cachehierarchies/chi/nodes:memory_controller",
        "//src/python/gem5/components/cachehierarchies/ruby:abstract_ruby_cache_hierarchy",
        "//src/python/gem5/components/cachehierarchies/ruby/topologies:simple_pt2pt",
        "//src/python/gem5/components/processors:abstract_core",
        "//src/python/gem5/utils:override",
        "//src/python/gem5/utils:requires",
        "//src/python/m5/objects:RubySystem",
        "//src/python/m5/objects:Sequencer",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/params",
    ],
)
