load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_KvmInt",
    desc = "",
    flag = "KvmInt",
)

py_library(
    name = "KvmGic_py",
    srcs = ["KvmGic.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "muxing_kvm_gic_v2_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/dev/arm:gic_v2_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Gic",
    ],
    sim_object_header = ":gic",
    sim_object_module = "src.arch.arm.kvm.KvmGic",
    sim_object_name = "MuxingKvmGicV2",
    sim_object_py_lib = ":KvmGic_py",
)

gem5_sim_object(
    name = "muxing_kvm_gic_v3_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/dev/arm:gicv3_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Gic",
    ],
    sim_object_header = ":gic",
    sim_object_module = "src.arch.arm.kvm.KvmGic",
    sim_object_name = "MuxingKvmGicV3",
    sim_object_py_lib = ":KvmGic_py",
)

gem5_cc_library(
    name = "gic",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["gic.cc"],
    }),
    hdrs = ["gic.hh"],
    copts = [
        "-include",
        "kvm_compat.hh",
    ],
    deps = [
        ":base_cpu",
        ":muxing_kvm_gic_v2_sim_object",
        ":muxing_kvm_gic_v3_sim_object",
        "//bazel:kvm_compat",
        "//src/arch/arm:system",
        "//src/arch/arm/regs:misc_hh",
        "//src/cpu/kvm:device",
        "//src/cpu/kvm:vm",
        "//src/dev:platform",
        "//src/dev/arm:base_gic",
        "//src/dev/arm:debug_GIC",
        "//src/dev/arm:gic_v2",
        "//src/dev/arm:gic_v3",
        "//src/sim:debug_Interrupt",
    ],
)

gem5_cc_library(
    name = "base_cpu",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["base_cpu.cc"],
    }),
    hdrs = ["base_cpu.hh"],
    copts = [
        "-include",
        "kvm_compat.hh",
    ],
    deps = [
        ":base_arm_kvm_cpu_sim_object",
        ":debug_KvmInt",
        "//bazel:kvm_compat",
        "//src/arch/arm:interrupts_hh",
        "//src/arch/arm:pcstate",
        "//src/base:uncontended_mutex",
        "//src/cpu/kvm:base",
        "//src/dev/arm:base_gic",
        "//src/dev/arm:generic_timer",
        "//src/dev/arm:generic_timer_sim_object",
    ],
)

py_library(
    name = "BaseArmKvmCPU_py",
    srcs = ["BaseArmKvmCPU.py"],
    deps = [
        "//src/python/m5/objects:ArmCPU",
        "//src/python/m5/objects:ArmMMU",
        "//src/python/m5/objects:BaseKvmCPU",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_arm_kvm_cpu_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/cpu/kvm:base_kvm_cpu_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ArmCPU",
        "//src/python/m5/objects:ArmMMU",
        "//src/python/m5/objects:BaseKvmCPU",
        "//src/python/m5/objects:BaseSimpleCPU",
        "//src/python/m5/objects:KvmVM",
    ],
    sim_object_header = ":base_cpu",
    sim_object_module = "src.arch.arm.kvm.BaseArmKvmCPU",
    sim_object_name = "BaseArmKvmCPU",
    sim_object_py_lib = ":BaseArmKvmCPU_py",
)
