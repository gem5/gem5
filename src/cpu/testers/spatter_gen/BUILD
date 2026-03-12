load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "spatter_gen_init",
    srcs = ["__init__.py"],
    deps = [
        ":SpatterGen_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "SpatterGen_py",
    srcs = ["SpatterGen.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:citations",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

gem5_debug_flag(
    name = "debug_SpatterGen",
    flag = "SpatterGen",
)

gem5_debug_flag(
    name = "debug_SpatterKernel",
    flag = "SpatterKernel",
)

gem5_enum_generator(
    name = "enum_SpatterKernelType",
    enum_module = "src.cpu.testers.spatter_gen.SpatterGen",
    enum_name = "SpatterKernelType",
    enum_py_lib = ":SpatterGen_py",
    extra_tool_deps = [
        "//src/python/m5/objects:System",
        "//src/python/m5/objects:ClockedObject",
    ],
)

gem5_enum_generator(
    name = "enum_SpatterProcessingMode",
    enum_module = "src.cpu.testers.spatter_gen.SpatterGen",
    enum_name = "SpatterProcessingMode",
    enum_py_lib = ":SpatterGen_py",
    extra_tool_deps = [
        "//src/python/m5/objects:System",
        "//src/python/m5/objects:ClockedObject",
    ],
)

gem5_sim_object(
    name = "spatter_gen_sim_object",
    extra_deps = [
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
    extra_param_deps = [
        ":enum_SpatterKernelType_hh",
        ":enum_SpatterProcessingMode_hh",
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
    sim_object_header = ":spatter_gen",
    sim_object_module = "src.cpu.testers.spatter_gen.SpatterGen",
    sim_object_name = "SpatterGen",
    sim_object_py_lib = ":SpatterGen_py",
)

gem5_cc_library(
    name = "spatter_gen",
    srcs = ["spatter_gen.cc"],
    hdrs = ["spatter_gen.hh"],
    deps = [
        ":debug_SpatterGen",
        ":debug_SpatterKernel",
        ":enum_SpatterKernelType_hh",
        ":enum_SpatterProcessingMode_hh",
        ":spatter_gen_sim_object",
        ":utility_structs",
        "//src/base:cprintf",
        "//src/base:statistics",
        "//src/base:types",
        "//src/base/stats:group",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/sim:clocked_object",
        "//src/sim:eventq",
        "//src/sim:sim_exit",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "utility_structs",
    hdrs = ["utility_structs.hh"],
    deps = [
        ":enum_SpatterKernelType_hh",
        "//src/base:random",
        "//src/base:types",
        "//src/mem:packet",
    ],
)
