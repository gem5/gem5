load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "linux",
    srcs = [
        "events.cc",
        "helpers.cc",
        "linux.cc",
        "printk.cc",
    ],
    hdrs = [
        "events.hh",
        "flag_tables.hh",
        "helpers.hh",
        "linux.hh",
        "printk.hh",
    ],
    deps = [
        "//src/base:compiler",
        "//src/base:output",
        "//src/base:random",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/loader:object_file",
        "//src/cpu:base",
        "//src/cpu:thread_context",
        "//src/kern:debug_DebugPrintf",
        "//src/kern:operatingsystem",
        "//src/kern:system_events",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:byteswap",
        "//src/sim:core",
        "//src/sim:debug_SyscallVerbose",
        "//src/sim:enum_KernelPanicOopsBehaviour_hh",
        "//src/sim:guest_abi",
        "//src/sim:mem_state",
        "//src/sim:process",
        "//src/sim:se_workload",
        "//src/sim:sim_exit",
        "//src/sim:system",
        "//src/sim:vma",
    ],
)
