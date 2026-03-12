load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

py_library(
    name = "kvm_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseKvmCPU_py",
        ":KvmVM_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_Kvm",
    desc = "",
    flag = "Kvm",
)

gem5_debug_flag(
    name = "debug_KvmTimer",
    desc = "",
    flag = "KvmTimer",
)

gem5_debug_flag(
    name = "debug_KvmIO",
    desc = "",
    flag = "KvmIO",
)

gem5_debug_flag(
    name = "debug_KvmRun",
    desc = "",
    flag = "KvmRun",
)

gem5_cc_library(
    name = "perfevent",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["perfevent.cc"],
    }),
    hdrs = ["perfevent.hh"],
    deps = [
        "//bazel/generated/config:have_perf_attr_exclude_host",
        "//src/base:logging",
    ] + select({
        "//bazel:macos": ["//bazel:kvm_compat"],
        "//conditions:default": [],
    }),
)

gem5_cc_library(
    name = "timer",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["timer.cc"],
    }),
    hdrs = ["timer.hh"],
    deps = [
        ":debug_KvmTimer",
        ":perfevent",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:core",
    ],
)

gem5_cc_library(
    name = "vm_hh",
    hdrs = ["vm.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/sim:sim_object_hh",
    ] + select({
        "//bazel:macos": ["//bazel:kvm_compat"],
        "//conditions:default": [],
    }),
)

gem5_cc_library(
    name = "vm",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["vm.cc"],
    }),
    hdrs = ["vm.hh"],
    deps = [
        ":base_hh",
        ":debug_Kvm",
        ":param_kvm_vm_hh",
        ":vm_hh",
        "//src/base:addr_range",
        "//src/mem:physical",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "device",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["device.cc"],
    }),
    hdrs = ["device.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
    ] + select({
        "//bazel:macos": ["//bazel:kvm_compat"],
        "//conditions:default": [],
    }),
)

gem5_cc_library(
    name = "base_hh",
    hdrs = ["base.hh"],
    deps = [
        ":perfevent",
        ":timer",
        ":vm_hh",
        "//src/base:statistics",
        "//src/cpu:base",
        "//src/cpu:simple_thread",
        "//src/sim:faults",
    ],
)

gem5_cc_library(
    name = "base",
    srcs = select({
        "//bazel:macos": [],
        "//conditions:default": ["base.cc"],
    }),
    hdrs = ["base.hh"],
    deps = [
        ":base_hh",
        ":param_base_kvm_cpu_hh",
        ":debug_Kvm",
        ":debug_KvmIO",
        ":debug_KvmRun",
        ":device",
        ":perfevent",
        ":timer",
        ":vm",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/cpu:base",
        "//src/cpu:simple_thread",
        "//src/cpu:thread_context",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_Drain",
        "//src/sim:faults",
        "//src/sim:process",
        "//src/sim:system",
    ] + select({
        "//bazel:macos": ["//bazel:kvm_compat"],
        "//conditions:default": [],
    }),
)

py_library(
    name = "KvmVM_py",
    srcs = ["KvmVM.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "kvm_vm_sim_object",
    extra_deps = [
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        "//src/base:addr_range",
        "//src/sim:sim_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":vm",
    sim_object_module = "src.cpu.kvm.KvmVM",
    sim_object_name = "KvmVM",
    sim_object_py_lib = ":KvmVM_py",
)

py_library(
    name = "BaseKvmCPU_py",
    srcs = ["BaseKvmCPU.py"],
    deps = [
        ":KvmVM_py",
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:BaseCPU",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_kvm_cpu_sim_object",
    extra_param_deps = [
        "//src/cpu:base_cpu_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:KvmVM",
        "//src/python/m5/objects:BaseCPU",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.cpu.kvm.BaseKvmCPU",
    sim_object_name = "BaseKvmCPU",
    sim_object_py_lib = ":BaseKvmCPU_py",
)
