load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "processors",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_core",
        ":abstract_generator",
        ":abstract_generator_core",
        ":abstract_processor",
        ":linear_generator",
        ":linear_generator_core",
    ],
)

py_library(
    name = "abstract_core",
    srcs = ["abstract_core.py"],
    deps = [
        "//src/python/gem5:isas",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:BaseMMU",
        "//src/python/m5/objects:PcCountTracker",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "abstract_processor",
    srcs = ["abstract_processor.py"],
    deps = [
        ":abstract_core",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/gem5/utils:requires",
    ],
)

py_library(
    name = "abstract_generator_core",
    srcs = ["abstract_generator_core.py"],
    deps = [
        ":abstract_core",
        "//src/python/gem5:isas",
        "//src/python/gem5/utils:override",
        "//src/python/m5/objects:PortTerminator",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "abstract_generator",
    srcs = ["abstract_generator.py"],
    deps = [
        ":abstract_generator_core",
        ":abstract_processor",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/boards:mem_mode",
        "//src/python/gem5/utils:override",
    ],
)

py_library(
    name = "linear_generator_core",
    srcs = ["linear_generator_core.py"],
    deps = [
        ":abstract_core",
        ":abstract_generator_core",
        "//src/python/gem5/utils:override",
        "//src/python/m5:ticks",
        "//src/python/m5/objects:BaseTrafficGen",
        "//src/python/m5/objects:PyTrafficGen",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "linear_generator",
    srcs = ["linear_generator.py"],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":abstract_generator",
        ":linear_generator_core",
        "//src/python/gem5/utils:override",
    ],
)

py_library(
    name = "switchable_processor",
    srcs = ["switchable_processor.py"],
    deps = [
        ":abstract_core",
        ":abstract_processor",
        ":cpu_types",
        ":simple_core",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/utils:override",
        "//src/python/m5",
        "//src/python/m5/objects:Root",
    ],
)

py_library(
    name = "cpu_types",
    srcs = ["cpu_types.py"],
    deps = [
        "//src/python/gem5/components/boards:mem_mode",
    ],
)

py_library(
    name = "simple_core",
    srcs = ["simple_core.py"],
    deps = [
        ":base_cpu_core",
        ":cpu_types",
        "//src/python/gem5:isas",
        "//src/python/gem5/utils",
    ],
)

py_library(
    name = "base_cpu_core",
    srcs = ["base_cpu_core.py"],
    deps = [
        ":abstract_core",
        "//src/python/gem5:isas",
        "//src/python/gem5/utils",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/objects:BaseMMU",
        "//src/python/m5/objects:PcCountTracker",
        "//src/python/m5/objects:Process",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "base_cpu_processor",
    srcs = ["base_cpu_processor.py"],
    deps = [
        ":abstract_processor",
        ":base_cpu_core",
        "//src/python/gem5/components/boards:abstract_board",
        "//src/python/gem5/components/boards:mem_mode",
        "//src/python/gem5/utils:override",
        "//src/python/m5",
        "//src/python/m5/objects:BaseAtomicSimpleCPU",
        "//src/python/m5/objects:BaseNonCachingSimpleCPU",
        "//src/python/m5/objects:BaseSimpleCPU",
        "//src/python/m5/objects:BaseTimingSimpleCPU",
        "//src/python/m5/objects:Root",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "simple_processor",
    srcs = ["simple_processor.py"],
    deps = [
        ":base_cpu_processor",
        ":cpu_types",
        ":simple_core",
        "//src/python/gem5:isas",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "random_generator_core",
    srcs = ["random_generator_core.py"],
    deps = [
        ":abstract_core",
        ":abstract_generator_core",
        "//src/python/gem5/utils:override",
        "//src/python/m5:ticks",
        "//src/python/m5/objects:BaseTrafficGen",
        "//src/python/m5/objects:PyTrafficGen",
        "//src/python/m5/util",
    ],
)

py_library(
    name = "random_generator",
    srcs = ["random_generator.py"],
    visibility = [
        "//platforms/performance/gsoc/gsim/runscripts:__pkg__",
        "//visibility:public",
    ],
    deps = [
        ":abstract_generator",
        ":random_generator_core",
        "//src/python/gem5/utils:override",
    ],
)
