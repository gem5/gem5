load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_debug_flag")

py_library(
    name = "arch_init",
    srcs = ["__init__.py"],
    deps = [
        "//src/arch/generic:generic_init",
        "//src/arch/arm:arch_arm_init",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_IntRegs",
    flag = "IntRegs",
)

gem5_debug_flag(
    name = "debug_FloatRegs",
    flag = "FloatRegs",
)

gem5_debug_flag(
    name = "debug_VecRegs",
    flag = "VecRegs",
)

gem5_debug_flag(
    name = "debug_VecPredRegs",
    flag = "VecPredRegs",
)

gem5_debug_flag(
    name = "debug_MatRegs",
    flag = "MatRegs",
)

gem5_debug_flag(
    name = "debug_CCRegs",
    flag = "CCRegs",
)

gem5_debug_flag(
    name = "debug_MiscRegs",
    flag = "MiscRegs",
)

gem5_debug_flag(
    name = "debug_Decoder",
    flag = "Decoder",
)

gem5_debug_flag(
    name = "debug_Faults",
    desc = "Information about faults, exceptions, interrupts, etc",
    flag = "Faults",
)

gem5_debug_flag(
    name = "debug_TLBVerbose",
    flag = "TLBVerbose",
)
