load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "generic_init",
    srcs = ["__init__.py"],
    deps = [
        ":BaseISA_py",
        ":BaseInterrupts_py",
        ":BaseMMU_py",
        ":BaseSemihosting_py",
        ":BaseTLB_py",
        ":InstDecoder_py",
    ],
)

gem5_debug_flag(
    name = "debug_PageTableWalker",
    desc = "Page table walker state machine debugging",
    flag = "PageTableWalker",
)

gem5_debug_flag(
    name = "debug_TLB",
    flag = "TLB",
)

gem5_debug_flag(
    name = "debug_Semihosting",
    flag = "Semihosting",
)

gem5_cc_library(
    name = "interrupts",
    hdrs = ["interrupts.hh"],
    deps = [
        ":base_interrupts_sim_object",
        "//src/base:logging",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "base_interrupts_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":interrupts",
    sim_object_module = "src.arch.generic.BaseInterrupts",
    sim_object_name = "BaseInterrupts",
    sim_object_py_lib = ":BaseInterrupts_py",
)

py_library(
    name = "BaseInterrupts_py",
    srcs = ["BaseInterrupts.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "BaseSemihosting_py",
    srcs = ["BaseSemihosting.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_semihosting_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":semihosting",
    sim_object_module = "src.arch.generic.BaseSemihosting",
    sim_object_name = "BaseSemihosting",
    sim_object_py_lib = ":BaseSemihosting_py",
)

gem5_cc_library(
    name = "semihosting",
    srcs = ["semihosting.cc"],
    hdrs = ["semihosting.hh"],
    deps = [
        ":base_semihosting_sim_object",
        ":debug_Semihosting",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:time",
        "//src/cpu:thread_context_hh",
        "//src/dev/serial",
        "//src/mem:physical",
        "//src/mem:port_proxy",
        "//src/sim:byteswap",
        "//src/sim:core",
        "//src/sim:guest_abi",
        "//src/sim:pseudo_inst",
        "//src/sim:sim_exit",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "pcstate",
    hdrs = ["pcstate.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "htm",
    srcs = ["htm.cc"],
    hdrs = ["htm.hh"],
    deps = [
        "//src/mem:htm_hh",
    ],
)

gem5_cc_library(
    name = "vec_reg",
    hdrs = ["vec_reg.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:serialize_handlers",
    ],
)

gem5_cc_test(
    name = "vec_reg_test",
    srcs = ["vec_reg.test.cc"],
    deps = [
        ":vec_reg",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "vec_pred_reg",
    hdrs = ["vec_pred_reg.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:types",
        "//src/sim:serialize_handlers",
    ],
)

gem5_cc_test(
    name = "vec_pred_reg_test",
    srcs = ["vec_pred_reg.test.cc"],
    deps = [
        ":vec_pred_reg",
        "//ext/googletest:gtest_main",
        "//src/base:logging",
    ],
)

py_library(
    name = "BaseMMU_py",
    srcs = ["BaseMMU.py"],
    deps = [
        "//src/python/m5/objects:BaseTLB",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_mmu_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":tlb",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_isa_sim_object",
        ":base_tlb_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":mmu",
    sim_object_module = "src.arch.generic.BaseMMU",
    sim_object_name = "BaseMMU",
    sim_object_py_lib = ":BaseMMU_py",
)

gem5_cc_library(
    name = "mmu",
    srcs = ["mmu.cc"],
    hdrs = ["mmu.hh"],
    deps = [
        ":base_mmu_sim_object",
        ":tlb",
        "//src/base:logging",
        "//src/cpu:thread_context_hh",
        "//src/mem:request",
        "//src/mem:translation_gen",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "mmu_hh",
    hdrs = ["mmu.hh"],
    deps = [
        ":base_mmu_sim_object",
        "//src/mem:request",
        "//src/mem:translation_gen",
        "//src/sim:sim_object_hh",
    ],
)

py_library(
    name = "BaseTLB_py",
    srcs = ["BaseTLB.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_tlb_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":enum_TypeTLB_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":tlb",
    sim_object_module = "src.arch.generic.BaseTLB",
    sim_object_name = "BaseTLB",
    sim_object_py_lib = ":BaseTLB_py",
)

gem5_cc_library(
    name = "tlb",
    hdrs = ["tlb.hh"],
    deps = [
        ":base_tlb_sim_object",
        ":enum_TypeTLB_hh",
        ":mmu_hh",
        "//src/base:logging",
        "//src/mem:request",
        "//src/sim:sim_object_hh",
    ],
)

gem5_enum_generator(
    name = "enum_TypeTLB",
    enum_module = "src.arch.generic.BaseTLB",
    enum_name = "TypeTLB",
    enum_py_lib = ":BaseTLB_py",
    extra_tool_deps = [
        "//src/python/m5:SimObject",
    ],
)

py_library(
    name = "InstDecoder_py",
    srcs = ["InstDecoder.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "inst_decoder_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
        ":isa",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":base_isa_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:BaseISA",
    ],
    sim_object_header = ":decoder",
    sim_object_module = "src.arch.generic.InstDecoder",
    sim_object_name = "InstDecoder",
    sim_object_py_lib = ":InstDecoder_py",
)

gem5_cc_library(
    name = "decoder",
    srcs = ["decoder.cc"],
    hdrs = ["decoder.hh"],
    deps = [
        ":inst_decoder_sim_object",
        ":pcstate",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:types",
        "//src/cpu:static_inst_fwd",
        "//src/sim:sim_object_hh",
    ],
)

py_library(
    name = "BaseISA_py",
    srcs = ["BaseISA.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "base_isa_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":isa",
    sim_object_module = "src.arch.generic.BaseISA",
    sim_object_name = "BaseISA",
    sim_object_py_lib = ":BaseISA_py",
)

gem5_cc_library(
    name = "isa",
    hdrs = ["isa.hh"],
    deps = [
        ":base_isa_sim_object",
        ":pcstate",
        "//src/base:logging",
        "//src/cpu:reg_class",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "decode_cache",
    hdrs = ["decode_cache.hh"],
    deps = [
        "//src/base:types",
        "//src/cpu:decode_cache",
        "//src/cpu:static_inst_fwd",
    ],
)

gem5_cc_library(
    name = "memhelpers",
    hdrs = ["memhelpers.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/sim:byteswap",
        "//src/sim:insttracer",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "debugfaults",
    hdrs = ["debugfaults.hh"],
    deps = [
        "//src/base:logging",
        "//src/base:types",
        "//src/cpu:null_static_inst",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/sim:faults",
    ],
)
