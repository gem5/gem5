load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "se_workload",
    srcs = ["se_workload.cc"],
    hdrs = ["se_workload.hh"],
    deps = [
        ":linux",
        ":process",
        "//src/arch/arm:arm_emu_linux_sim_object",
        "//src/arch/arm:page_size",
        "//src/arch/arm:se_workload",
        "//src/arch/arm/regs:int",
        "//src/arch/arm/regs:misc",
        "//src/base:trace",
        "//src/base/loader:object_file",
        "//src/cpu:thread_context",
        "//src/mem:se_translating_port_proxy",
        "//src/sim:syscall_desc",
        "//src/sim:syscall_emul",
    ],
)

gem5_cc_library(
    name = "linux",
    srcs = ["linux.cc"],
    hdrs = ["linux.hh"],
    deps = [
        "//src/arch/arm:pcstate",
        "//src/arch/arm:utility",
        "//src/arch/arm/regs:int",
        "//src/arch/arm/regs:misc",
        "//src/base:compiler",
        "//src/base:types",
        "//src/cpu:thread_context",
        "//src/kern/linux",
        "//src/mem:se_translating_port_proxy",
        "//src/sim:debug_SyscallVerbose",
    ],
)

gem5_cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.hh"],
    deps = [
        ":linux",
        "//src/arch/arm:page_size",
        "//src/arch/arm:process",
        "//src/base:trace",
        "//src/base/loader:object_file",
        "//src/cpu:thread_context",
        "//src/kern/linux",
        "//src/mem:se_translating_port_proxy",
        "//src/sim:process",
        "//src/sim:syscall_desc",
        "//src/sim:syscall_emul",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "atag",
    hdrs = ["atag.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "fs_workload",
    srcs = ["fs_workload.cc"],
    hdrs = ["fs_workload.hh"],
    deps = [
        ":atag",
        "//src/arch/arm:aapcs32",
        "//src/arch/arm:aapcs64",
        "//src/arch/arm:arm_fs_linux_sim_object",
        "//src/arch/arm:fs_workload",
        "//src/arch/arm:system_hh",
        "//src/arch/arm:utility",
        "//src/arch/generic/linux:threadinfo",
        "//src/base:output",
        "//src/base/loader:dtb_file",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab",
        "//src/cpu:base_hh",
        "//src/cpu:pc_event",
        "//src/cpu:thread_context",
        "//src/kern:system_events",
        "//src/kern/linux",
        "//src/mem:physical_hh",
        "//src/sim:debug_Loader",
        "//src/sim:stat_control",
    ],
)
