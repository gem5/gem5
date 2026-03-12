load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "cc",
    hdrs = ["cc.hh"],
    deps = [
        "//src/arch:debug_CCRegs",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "mat",
    hdrs = ["mat.hh"],
    deps = [
        "//src/arch:debug_MatRegs",
        "//src/arch/arm:matrix",
        "//src/arch/arm:types",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "vec",
    hdrs = ["vec.hh"],
    deps = [
        "//src/arch:debug_VecPredRegs",
        "//src/arch:debug_VecRegs",
        "//src/arch/arm:types",
        "//src/arch/generic:vec_pred_reg",
        "//src/arch/generic:vec_reg",
        "//src/cpu:reg_class",
    ],
)

gem5_cc_library(
    name = "misc_types",
    hdrs = ["misc_types.hh"],
    deps = [
        "//src/base:bitunion",
    ],
)

gem5_cc_library(
    name = "misc_accessors",
    hdrs = ["misc_accessors.hh"],
    deps = [
        ":misc_hh",
        ":misc_types",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "misc_info",
    srcs = ["misc_info.cc"],
    hdrs = ["misc_info.hh"],
    deps = [
        ":misc_hh",
        "//src/arch:debug_MiscRegs",
        "//src/arch/arm:types",
        "//src/arch/arm/insts",
        "//src/cpu:reg_class",
        "//src/sim:faults",
    ],
)

gem5_cc_library(
    name = "misc_info_hh",
    hdrs = ["misc_info.hh"],
    deps = [
        ":misc_hh",
        "//src/arch:debug_MiscRegs",
        "//src/arch/arm:types",
        "//src/cpu:reg_class",
        "//src/sim:faults",
    ],
)

gem5_cc_library(
    name = "misc",
    srcs = ["misc.cc"],
    hdrs = ["misc.hh"],
    deps = [
        ":misc_info",
        ":misc_types",
        "//src/arch:debug_MiscRegs",
        "//src/arch/arm:arm_isa_sim_object",
        "//src/arch/arm:isa_hh",
        "//src/arch/arm:types",
        "//src/arch/arm/insts",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/cpu:reg_class",
        "//src/cpu:thread_context",
        "//src/dev/arm:generic_timer_miscregs_types",
        "//src/dev/arm:gic_v3_cpu_interface",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "misc_hh",
    hdrs = ["misc.hh"],
    deps = [
        ":misc_types",
        "//src/arch:debug_MiscRegs",
        "//src/arch/arm:types",
        "//src/base:compiler",
        "//src/cpu:reg_class",
        "//src/dev/arm:generic_timer_miscregs_types",
    ],
)

gem5_cc_library(
    name = "int",
    srcs = ["int.cc"],
    hdrs = ["int.hh"],
    deps = [
        ":misc",
        "//src/arch:debug_IntRegs",
        "//src/arch/arm:isa_hh",
        "//src/arch/arm:types",
        "//src/arch/arm:utility",
        "//src/base:logging",
        "//src/cpu:reg_class",
        "//src/sim:core",
    ],
)

gem5_cc_library(
    name = "int_hh",
    hdrs = ["int.hh"],
    deps = [
        ":misc_hh",
        "//src/arch:debug_IntRegs",
        "//src/arch/arm:types",
        "//src/base:logging",
        "//src/cpu:reg_class",
        "//src/sim:core",
    ],
)
