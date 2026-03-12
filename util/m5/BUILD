load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "m5_internal_headers",
    includes = ["src"],
    textual_hdrs = [
        "src/args.hh",
        "src/call_type.hh",
        "src/call_type/addr_dt.hh",
        "src/call_type/inst_dt.hh",
        "src/call_type/semi_dt.hh",
        "src/command.hh",
        "src/dispatch_table.hh",
        "src/m5_mmap.h",
        "src/usage.hh",
    ],
)

cc_library(
    name = "call_type_inst",
    srcs = [
        "src/call_type/inst.cc",
    ],
    copts = [
        "-DCALL_TYPE_IS_DEFAULT=0",
        "-Wno-non-virtual-dtor",
        "-DM5OP_ADDR=0x10010000",
    ],
    includes = ["src"],
    linkstatic = True,
    deps = [
        ":m5_internal_headers",
        "//include:gem5",
    ],
)

cc_library(
    name = "call_type_addr",
    srcs = [
        "src/call_type/addr.cc",
    ],
    copts = [
        "-DCALL_TYPE_IS_DEFAULT=1",
        "-Wno-non-virtual-dtor",
        "-DM5OP_ADDR=0x10010000",
    ],
    includes = ["src"],
    linkstatic = True,
    deps = [
        ":m5_internal_headers",
        "//include:gem5",
    ],
)

cc_library(
    name = "call_type_semi",
    srcs = [
        "src/call_type/semi.cc",
    ],
    copts = [
        "-DCALL_TYPE_IS_DEFAULT=0",
        "-Wno-non-virtual-dtor",
        "-DM5OP_ADDR=0x10010000",
    ],
    includes = ["src"],
    linkstatic = True,
    deps = [
        ":m5_internal_headers",
        "//include:gem5",
        "//include:gem5_asm_generic",
    ],
)

cc_library(
    name = "libm5_arm",
    srcs = [
        "src/abi/arm64/m5op.S",
        "src/abi/arm64/m5op_addr.S",
        "src/abi/arm64/m5op_semi.S",
        "src/m5_mmap.c",
    ],
    copts = [
        "-DM5OP_ADDR=0x10010000",
    ],
    includes = [
        "src",
    ],
    linkstatic = True,
    target_compatible_with = ["//third_party/bazel_platforms/cpu:aarch64"],
    deps = [
        ":m5_internal_headers",
        "//include:gem5",
    ],
)

cc_binary(
    name = "m5_arm",
    srcs = [
        "src/args.cc",
        "src/call_type.cc",
        "src/command.cc",
        "src/command/addsymbol.cc",
        "src/command/checkpoint.cc",
        "src/command/dumpresetstats.cc",
        "src/command/dumpstats.cc",
        "src/command/exit.cc",
        "src/command/fail.cc",
        "src/command/hypercall.cc",
        "src/command/initparam.cc",
        "src/command/loadsymbol.cc",
        "src/command/readfile.cc",
        "src/command/resetstats.cc",
        "src/command/sum.cc",
        "src/command/workbegin.cc",
        "src/command/workend.cc",
        "src/command/writefile.cc",
        "src/m5.cc",
        "src/usage.cc",
    ],
    copts = [
        "-Wno-non-virtual-dtor",
        "-Wno-string-conversion",
    ],
    includes = [
        "src",
    ],
    linkstatic = True,
    target_compatible_with = ["//third_party/bazel_platforms/cpu:aarch64"],
    deps = [
        ":call_type_addr",
        ":call_type_inst",
        ":call_type_semi",
        ":libm5_arm",
        ":m5_internal_headers",
        "//include:gem5",
    ],
)
