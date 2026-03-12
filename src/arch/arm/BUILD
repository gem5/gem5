load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag", "gem5_enum_generator", "gem5_gdbxml_generator", "gem5_isa_cc_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_Arm",
    desc = "",
    flag = "Arm",
)

gem5_debug_flag(
    name = "debug_ArmTme",
    desc = "Transactional Memory Extension",
    flag = "ArmTme",
)

gem5_debug_flag(
    name = "debug_MPAM",
    desc = "MPAM debug flag",
    flag = "MPAM",
)

gem5_debug_flag(
    name = "debug_PMUVerbose",
    desc = "Performance Monitor",
    flag = "PMUVerbose",
)

gem5_isa_cc_library(
    name = "generated_isa",
    arch = "arm",
    constrs_splits = 3,
    exec_splits = 6,
    extra_deps = [
        ":faults",
        ":interrupts_hh",
        ":isa_hh",
        "//include:gem5_asm_generic",
        ":decoder",
        "//src/arch/arm/insts",
        "//src/arch/arm/insts:vector_element_traits",
        "//src/arch/arm/insts:static_inst",
        ":enum_DecoderFlavor_hh",
        ":htm",
        ":pauth_helpers",
        ":reg_abi",
        ":semihosting",
        ":utility_hh",
        "//src/arch/generic:memhelpers",
        "//src/base:condcodes",
        "//src/base:crc",
        "//src/base:fenv",
        "//src/cpu:base_hh",
        "//src/cpu:thread_context",
        ":debug_Arm",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:pseudo_inst",
        "//src/sim:sim_exit",
        "//src/sim:faults",
        "//src/base:bitfield",
        "//src/base:cprintf",
        "//src/base/loader:symtab",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_info",
    ],
    extra_hdrs = [
        ":decoder",
        ":enum_DecoderFlavor_hh",
        "//src/arch/arm/insts:insts_hh",
        "//src/arch/arm/insts:static_inst",
        "//src/arch/arm/insts:vector_element_traits",
        "//src/arch/arm/regs:misc_info_hh",
    ],
    isa_srcs = glob(["isa/**"]),
    main_isa_file = "isa/main.isa",
)

gem5_gdbxml_generator(
    name = "gdb_xml_arm_target",
    isa = "arm",
    symbol = "gdb_xml_arm_target",
    xml_file = "gdb-xml/arm-with-neon.xml",
)

gem5_gdbxml_generator(
    name = "gdb_xml_arm_core",
    isa = "arm",
    symbol = "gdb_xml_arm_core",
    xml_file = "gdb-xml/arm-core.xml",
)

gem5_gdbxml_generator(
    name = "gdb_xml_arm_vfpv3",
    isa = "arm",
    symbol = "gdb_xml_arm_vfpv3",
    xml_file = "gdb-xml/arm-vfpv3.xml",
)

gem5_gdbxml_generator(
    name = "gdb_xml_aarch64_target",
    isa = "arm",
    symbol = "gdb_xml_aarch64_target",
    xml_file = "gdb-xml/aarch64.xml",
)

gem5_gdbxml_generator(
    name = "gdb_xml_aarch64_core",
    isa = "arm",
    symbol = "gdb_xml_aarch64_core",
    xml_file = "gdb-xml/aarch64-core.xml",
)

gem5_gdbxml_generator(
    name = "gdb_xml_aarch64_fpu",
    isa = "arm",
    symbol = "gdb_xml_aarch64_fpu",
    xml_file = "gdb-xml/aarch64-fpu.xml",
)

py_library(
    name = "ArmCPU_py",
    srcs = ["ArmCPU.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ArmDecoder",
        "//src/python/m5/objects:ArmISA",
        "//src/python/m5/objects:ArmInterrupts",
        "//src/python/m5/objects:ArmMMU",
        "//src/python/m5/objects:BaseAtomicSimpleCPU",
        "//src/python/m5/objects:BaseMinorCPU",
        "//src/python/m5/objects:BaseNonCachingSimpleCPU",
        "//src/python/m5/objects:BaseO3CPU",
        "//src/python/m5/objects:BaseO3Checker",
        "//src/python/m5/objects:BaseTimingSimpleCPU",
    ],
)

py_library(
    name = "ArmDecoder_py",
    srcs = ["ArmDecoder.py"],
    deps = [
        "//src/python/m5/objects:InstDecoder",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "ArmFsWorkload_py",
    srcs = ["ArmFsWorkload.py"],
    deps = [
        "//src/python/m5:options",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Workload",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "arm_fs_workload_sim_object",
    extra_param_deps = [
        ":enum_ArmMachineType_hh",
        "//src/sim:kernel_workload_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":fs_workload",
    sim_object_module = "src.arch.arm.ArmFsWorkload",
    sim_object_name = "ArmFsWorkload",
    sim_object_py_lib = "//src/arch/arm:ArmFsWorkload_py",
)

gem5_enum_generator(
    name = "enum_ArmMachineType",
    enum_module = "src.arch.arm.ArmFsWorkload",
    enum_name = "ArmMachineType",
    enum_py_lib = "//src/arch/arm:ArmFsWorkload_py",
)

py_library(
    name = "ArmInterrupts_py",
    srcs = ["ArmInterrupts.py"],
    deps = [
        "//src/python/m5/objects:BaseInterrupts",
    ],
)

gem5_sim_object(
    name = "arm_interrupts_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/arch/generic:base_interrupts_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":interrupts",
    sim_object_module = "src.arch.arm.ArmInterrupts",
    sim_object_name = "ArmInterrupts",
    sim_object_py_lib = ":ArmInterrupts_py",
)

py_library(
    name = "ArmISA_py",
    srcs = ["ArmISA.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ArmPMU",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:BaseISA",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "ArmMMU_py",
    srcs = ["ArmMMU.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:ArmTLB",
        "//src/python/m5/objects:BaseMMU",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "ArmNativeTrace_py",
    srcs = ["ArmNativeTrace.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:CPUTracers",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "ArmSemihosting_py",
    srcs = ["ArmSemihosting.py"],
    deps = [
        "//src/python/m5/objects:BaseSemihosting",
    ],
)

py_library(
    name = "ArmSeWorkload_py",
    srcs = ["ArmSeWorkload.py"],
    deps = [
        "//src/python/m5/objects:Workload",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "ArmSystem_py",
    srcs = ["ArmSystem.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:options",  # TODO(jlowepower): Remove this.
        "//src/python/m5/objects:ArmSemihosting",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

py_library(
    name = "ArmTLB_py",
    srcs = ["ArmTLB.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:BaseTLB",
        "//src/python/m5/objects:ReplacementPolicies",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_ArmLookupLevel",
    enum_module = "src.arch.arm.ArmTLB",
    enum_name = "ArmLookupLevel",
    enum_py_lib = ":ArmTLB_py",
)

gem5_sim_object(
    name = "tlb_indexing_policy_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":pagetable",
    sim_object_module = "src.arch.arm.ArmTLB",
    sim_object_name = "TLBIndexingPolicy",
    sim_object_py_lib = ":ArmTLB_py",
)

gem5_sim_object(
    name = "tlb_set_associative_sim_object",
    extra_param_deps = [
        ":tlb_indexing_policy_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":pagetable",
    sim_object_module = "src.arch.arm.ArmTLB",
    sim_object_name = "TLBSetAssociative",
    sim_object_py_lib = ":ArmTLB_py",
)

gem5_sim_object(
    name = "arm_tlb_sim_object",
    extra_param_deps = [
        "//src/sim:system_sim_object",
        "//src/mem/cache/replacement_policies:base_replacement_policy_sim_object",
        ":tlb_indexing_policy_sim_object",
        ":enum_ArmLookupLevel_hh",
        "//src/arch/generic:base_tlb_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
        ":pagetable",
        "//src/mem/cache/replacement_policies:base",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":tlb",
    sim_object_module = "src.arch.arm.ArmTLB",
    sim_object_name = "ArmTLB",
    sim_object_py_lib = ":ArmTLB_py",
)

py_library(
    name = "ArmPMU_py",
    srcs = ["ArmPMU.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "arm_semihosting_sim_object",
    extra_param_deps = [
        "//src/arch/generic:base_semihosting_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":semihosting",
    sim_object_module = "src.arch.arm.ArmSemihosting",
    sim_object_name = "ArmSemihosting",
    sim_object_py_lib = ":ArmSemihosting_py",
)

gem5_sim_object(
    name = "arm_se_workload_sim_object",
    extra_param_deps = [
        "//src/sim:se_workload_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":se_workload",
    sim_object_module = "src.arch.arm.ArmSeWorkload",
    sim_object_name = "ArmSEWorkload",
    sim_object_py_lib = ":ArmSeWorkload_py",
)

gem5_sim_object(
    name = "arm_emu_linux_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:se_workload_sim_object",
        "//src/sim:sim_object",
        ":arm_se_workload_sim_object",
    ],
    sim_object_header = "//src/arch/arm/linux:se_workload",
    sim_object_module = "src.arch.arm.ArmSeWorkload",
    sim_object_name = "ArmEmuLinux",
    sim_object_py_lib = ":ArmSeWorkload_py",
)

gem5_sim_object(
    name = "arm_native_trace_sim_object",
    extra_param_deps = [
        "//src/cpu:native_trace_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":nativetrace",
    sim_object_module = "src.arch.arm.ArmNativeTrace",
    sim_object_name = "ArmNativeTrace",
    sim_object_py_lib = ":ArmNativeTrace_py",
)

gem5_sim_object(
    name = "arm_table_walker_sim_object",
    extra_param_deps = [
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/arch/generic:base_tlb_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
        ":arm_walk_unit_sim_object",
    ],
    sim_object_header = ":table_walker",
    sim_object_module = "src.arch.arm.ArmMMU",
    sim_object_name = "ArmTableWalker",
    sim_object_py_lib = ":ArmMMU_py",
)

gem5_sim_object(
    name = "arm_walk_unit_sim_object",
    extra_param_deps = [
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/arch/generic:base_tlb_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
        "//src/arch/generic:enum_TypeTLB_hh",
    ],
    sim_object_header = ":table_walker",
    sim_object_module = "src.arch.arm.ArmMMU",
    sim_object_name = "ArmWalkUnit",
    sim_object_py_lib = ":ArmMMU_py",
)

gem5_sim_object(
    name = "arm_mmu_sim_object",
    extra_param_deps = [
        ":arm_table_walker_sim_object",
        ":arm_release_sim_object",
        ":arm_tlb_sim_object",
        "//src/sim:system_sim_object",
        "//src/arch/generic:base_mmu_sim_object",
        "//src/sim:sim_object",
        ":table_walker",
        ":system",
        ":tlb",
        "//src/sim:system",
    ],
    sim_object_header = ":mmu",
    sim_object_module = "src.arch.arm.ArmMMU",
    sim_object_name = "ArmMMU",
    sim_object_py_lib = ":ArmMMU_py",
)

gem5_sim_object(
    name = "arm_release_sim_object",
    extra_param_deps = [
        ":enum_ArmExtension_hh",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":system",
    sim_object_module = "src.arch.arm.ArmSystem",
    sim_object_name = "ArmRelease",
    sim_object_py_lib = ":ArmSystem_py",
)

gem5_sim_object(
    name = "arm_system_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        ":arm_release_sim_object",
        ":arm_semihosting_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object_hh",
        ":semihosting",
    ],
    sim_object_header = ":system",
    sim_object_module = "src.arch.arm.ArmSystem",
    sim_object_name = "ArmSystem",
    sim_object_py_lib = ":ArmSystem_py",
)

gem5_enum_generator(
    name = "enum_ArmExtension",
    enum_module = "src.arch.arm.ArmSystem",
    enum_name = "ArmExtension",
    enum_py_lib = ":ArmSystem_py",
)

gem5_sim_object(
    name = "arm_isa_sim_object",
    extra_param_deps = [
        ":arm_pmu_sim_object",
        ":enum_DecoderFlavor_hh",
        ":enum_EventTypeId_hh",
        ":arm_release_sim_object",
        "//src/sim:system_sim_object",
        "//src/arch/generic:base_isa_sim_object",
        "//src/sim:sim_object",
        ":pmu",
        ":system",
        "//src/sim:system",
        ":generated_isa",  # TODO(jlowepower): Figure out the right dependence here
    ],
    sim_object_header = ":isa_hh",
    sim_object_module = "src.arch.arm.ArmISA",
    sim_object_name = "ArmISA",
    sim_object_py_lib = ":ArmISA_py",
)

gem5_enum_generator(
    name = "enum_DecoderFlavor",
    enum_module = "src.arch.arm.ArmISA",
    enum_name = "DecoderFlavor",
    enum_py_lib = ":ArmISA_py",
)

gem5_sim_object(
    name = "arm_pmu_sim_object",
    extra_param_deps = [
        "//src/dev/arm:arm_interrupt_pin_sim_object",
        "//src/dev:platform_sim_object",
        ":enum_EventTypeId_hh",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
        "//src/dev/arm:base_gic",
        "//src/dev:platform",
    ],
    sim_object_header = ":pmu",
    sim_object_module = "src.arch.arm.ArmPMU",
    sim_object_name = "ArmPMU",
    sim_object_py_lib = ":ArmPMU_py",
)

gem5_enum_generator(
    name = "enum_EventTypeId",
    enum_module = "src.arch.arm.ArmPMU",
    enum_name = "EventTypeId",
    enum_py_lib = ":ArmPMU_py",
)

gem5_sim_object(
    name = "arm_decoder_sim_object",
    extra_param_deps = [
        "//src/arch/generic:inst_decoder_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":decoder",
    sim_object_module = "src.arch.arm.ArmDecoder",
    sim_object_name = "ArmDecoder",
    sim_object_py_lib = ":ArmDecoder_py",
)

gem5_cc_library(
    name = "faults",
    srcs = ["faults.cc"],
    hdrs = ["faults.hh"],
    deps = [
        ":interrupts_hh",
        ":isa_hh",
        ":pagetable",
        ":self_debug_hh",
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch:debug_Faults",
        "//src/arch/arm/insts:static_inst",
        "//src/arch/arm/regs:misc_accessors",
        "//src/arch/arm/regs:misc_hh",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base",
        "//src/cpu:null_static_inst",
        "//src/cpu:thread_context",
        "//src/sim:faults",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "faults_hh",
    hdrs = ["faults.hh"],
    deps = [
        ":pagetable",
        ":types",
        "//src/arch/arm/regs:misc_hh",
        "//src/base:logging",
        "//src/cpu:null_static_inst",
        "//src/sim:faults",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "interrupts",
    srcs = ["interrupts.cc"],
    hdrs = ["interrupts.hh"],
    deps = [
        ":faults",
        ":system",
        ":utility_hh",
        "//src/arch/arm:arm_interrupts_sim_object",
        "//src/arch/arm:enum_ArmExtension_hh",
        "//src/arch/arm/regs:misc",
        "//src/arch/generic:interrupts",
        "//src/cpu:thread_context",
        "//src/sim:debug_Interrupt",
    ],
)

gem5_cc_library(
    name = "interrupts_hh",
    hdrs = ["interrupts.hh"],
    deps = [
        ":faults_hh",
        ":utility_hh",
        "//src/arch/arm:enum_ArmExtension_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/generic:interrupts",
        "//src/cpu:thread_context",
        "//src/sim:debug_Interrupt",
    ],
)

gem5_cc_library(
    name = "utility_hh",
    hdrs = ["utility.hh"],
    deps = [
        ":types",
        "//src/arch/arm:enum_ArmExtension_hh",
        "//src/arch/arm/regs:cc",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "utility",
    srcs = ["utility.cc"],
    hdrs = ["utility.hh"],
    deps = [
        ":enum_ArmExtension_hh",
        ":faults_hh",
        ":interrupts_hh",
        ":isa_hh",
        ":mmu_hh",
        ":page_size",
        ":system_hh",
        ":types",
        "//src/arch/arm/regs:cc",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/arch/arm/regs:vec",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/cpu/checker:cpu",
        "//src/mem:port_proxy",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "isa_hh",
    hdrs = ["isa.hh"],
    deps = [
        ":enum_DecoderFlavor_hh",
        ":isa_device_hh",
        ":mmu_hh",
        ":pcstate",
        ":self_debug_hh",
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:mat",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:vec",
        "//src/arch/generic:isa",
        "//src/base:random",
        "//src/sim:debug_Checkpoint",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "isa",
    srcs = ["isa.cc"],
    hdrs = ["isa.hh"],
    deps = [
        ":arm_isa_sim_object",
        ":debug_Arm",
        ":decoder",
        ":enum_DecoderFlavor_hh",
        ":faults",
        ":htm",
        ":interrupts",
        ":isa_device",
        ":mmu",
        ":pcstate",
        ":pmu",
        ":self_debug",
        ":system",
        ":types",
        ":utility_hh",
        "//src/arch:debug_MatRegs",
        "//src/arch:debug_VecPredRegs",
        "//src/arch:debug_VecRegs",
        "//src/arch/arm/regs:int",
        "//src/arch/arm/regs:mat",
        "//src/arch/arm/regs:misc",
        "//src/arch/arm/regs:misc_accessors",
        "//src/arch/arm/regs:misc_info",
        "//src/arch/arm/regs:vec",
        "//src/arch/generic:decoder",
        "//src/arch/generic:isa",
        "//src/base:cprintf",
        "//src/base:random",
        "//src/cpu:base",
        "//src/cpu:reg_class",
        "//src/cpu/checker:cpu",
        "//src/dev/arm:generic_timer",
        "//src/dev/arm:gic_v3",
        "//src/dev/arm:gic_v3_cpu_interface_hh",
        "//src/mem:debug_LLSC",
        "//src/sim:debug_Checkpoint",
        "//src/sim:faults",
        "//src/sim:sim_object",
        "//src/sim:stat_control",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "pagetable",
    srcs = ["pagetable.cc"],
    hdrs = ["pagetable.hh"],
    deps = [
        ":page_size",
        ":tlb_indexing_policy_sim_object",
        ":tlb_set_associative_sim_object",
        ":types",
        ":utility_hh",
        "//src/arch/arm:enum_ArmLookupLevel_hh",
        "//src/arch/generic:enum_TypeTLB_hh",
        "//src/arch/generic:mmu",
        "//src/base:bitfield",
        "//src/base:logging",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "page_size",
    hdrs = ["page_size.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "self_debug",
    srcs = ["self_debug.cc"],
    hdrs = ["self_debug.hh"],
    deps = [
        ":faults",
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/arch/generic:tlb",
        "//src/base:bitfield",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "self_debug_hh",
    hdrs = ["self_debug.hh"],
    deps = [
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/generic:tlb",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "types",
    hdrs = ["types.hh"],
    deps = [
        ":pcstate",
        "//src/base:bitunion",
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "pcstate",
    hdrs = ["pcstate.hh"],
    deps = [
        "//src/arch:debug_Decoder",
        "//src/arch/generic:pcstate",
        "//src/base:bitunion",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "system",
    srcs = ["system.cc"],
    hdrs = ["system.hh"],
    deps = [
        ":arm_release_sim_object",
        ":arm_system_sim_object",
        ":enum_ArmExtension_hh",
        ":fs_workload",
        ":page_size",
        ":pagetable",
        ":semihosting",
        ":types",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab",
        "//src/cpu:thread_context",
        "//src/dev/arm:fvp_base_pwr_ctrl",
        "//src/dev/arm:gic_v2",
        "//src/kern/linux",
        "//src/mem:physical",
        "//src/sim:full_system",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "system_hh",
    hdrs = ["system.hh"],
    deps = [
        ":enum_ArmExtension_hh",
        ":page_size",
        ":types",
        "//src/kern/linux",
        "//src/sim:full_system",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "isa_device",
    srcs = ["isa_device.cc"],
    hdrs = ["isa_device.hh"],
    deps = [
        "//src/arch/arm/regs:misc",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "isa_device_hh",
    hdrs = ["isa_device.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "matrix",
    hdrs = ["matrix.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:serialize_handlers",
    ],
)

gem5_cc_test(
    name = "matrix_test",
    srcs = ["matrix.test.cc"],
    deps = [
        ":matrix",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "mmu",
    srcs = ["mmu.cc"],
    hdrs = ["mmu.hh"],
    deps = [
        ":arm_mmu_sim_object",
        ":enum_ArmLookupLevel_hh",
        ":isa_hh",
        ":mpam",
        ":page_size",
        ":reg_abi",
        ":self_debug",
        ":stage2_lookup",
        ":table_walker",
        ":tlb",
        ":tlbi_op",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/arch/generic:mmu",
        "//src/base:addr_range",
        "//src/base:memoizer",
        "//src/base:statistics",
        "//src/mem:debug_MMU",
        "//src/mem:packet_access",
        "//src/sim:process",
        "//src/sim:pseudo_inst",
    ],
)

gem5_cc_library(
    name = "mmu_hh",
    hdrs = ["mmu.hh"],
    deps = [
        ":enum_ArmLookupLevel_hh",
        ":page_size",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/arch/generic:mmu",
        "//src/base:addr_range",
        "//src/base:memoizer",
        "//src/base:statistics",
    ],
)

gem5_cc_library(
    name = "tlb",
    srcs = ["tlb.cc"],
    hdrs = ["tlb.hh"],
    deps = [
        ":arm_tlb_sim_object",
        ":faults",
        ":page_size",
        ":pagetable",
        ":table_walker",
        ":tlbi_op",
        ":utility_hh",
        "//src/arch:debug_TLBVerbose",
        "//src/arch/generic:debug_TLB",
        "//src/arch/generic:enum_TypeTLB_hh",
        "//src/arch/generic:mmu",
        "//src/arch/generic:tlb",
        "//src/base:memoizer",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base/cache:associative_cache",
        "//src/cpu:thread_context",
        "//src/mem:request",
        "//src/sim/probe:pmu",
    ],
)

gem5_cc_library(
    name = "tlb_hh",
    hdrs = ["tlb.hh"],
    deps = [
        ":faults_hh",
        ":pagetable",
        ":utility_hh",
        "//src/arch/generic:enum_TypeTLB_hh",
        "//src/arch/generic:tlb",
        "//src/base:statistics",
        "//src/base/cache:associative_cache",
        "//src/mem:request",
        "//src/sim/probe:pmu",
    ],
)

gem5_cc_library(
    name = "tlbi_op",
    srcs = ["tlbi_op.cc"],
    hdrs = ["tlbi_op.hh"],
    deps = [
        ":mmu_hh",
        ":system_hh",
        ":tlb_hh",
        "//src/cpu:thread_context",
        "//src/cpu/checker:cpu",
    ],
)

gem5_cc_library(
    name = "fs_workload",
    srcs = ["fs_workload.cc"],
    hdrs = ["fs_workload.hh"],
    deps = [
        ":aapcs32",
        ":aapcs64",
        ":arm_fs_workload_sim_object",
        ":arm_system_sim_object",
        ":faults_hh",
        ":remote_gdb",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab",
        "//src/cpu:thread_context",
        "//src/dev/arm:gic_v2",
        "//src/kern:system_events",
        "//src/kern/linux",
        "//src/sim:kernel_workload",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "aapcs32",
    hdrs = ["aapcs32.hh"],
    deps = [
        ":utility_hh",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:vec",
        "//src/base:intmath",
        "//src/cpu:thread_context",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:full_system",
        "//src/sim:guest_abi",
        "//src/sim:proxy_ptr",
    ],
)

gem5_cc_library(
    name = "aapcs64",
    hdrs = ["aapcs64.hh"],
    deps = [
        ":utility_hh",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:vec",
        "//src/base:intmath",
        "//src/cpu:thread_context",
        "//src/sim:guest_abi",
        "//src/sim:proxy_ptr",
    ],
)

# TODO(jlowepower): This test won't compile in google3.
# This depdends on python things (StaticInstFlags_py)
# gem5_cc_test(
#     name = "aapcs64_test",
#     srcs = ["aapcs64.test.cc"],
#     deps = [
#         ":aapcs64",
#         ":decoder",
#         ":faults",
#         ":isa",
#         ":isa_device",
#         ":system",
#         ":utility",
#         "//ext/googletest:gtest_main",
#         "//src/cpu:enum_StaticInstFlags_py",
#         "//src/dev/arm:realview",
#     ],
# )

gem5_cc_library(
    name = "remote_gdb",
    srcs = ["remote_gdb.cc"],
    hdrs = ["remote_gdb.hh"],
    deps = [
        ":decoder",
        ":pagetable",
        ":system_hh",
        ":utility_hh",
        "//src/arch/arm:gdb_xml_aarch64_core",
        "//src/arch/arm:gdb_xml_aarch64_fpu",
        "//src/arch/arm:gdb_xml_aarch64_target",
        "//src/arch/arm:gdb_xml_arm_core",
        "//src/arch/arm:gdb_xml_arm_target",
        "//src/arch/arm:gdb_xml_arm_vfpv3",
        "//src/arch/arm/regs:vec",
        "//src/arch/generic:mmu",
        "//src/base:chunk_generator",
        "//src/base:compiler",
        "//src/base:debug_GDBAcc",
        "//src/base:debug_GDBMisc",
        "//src/base:intmath",
        "//src/base:remote_gdb",
        "//src/base:socket",
        "//src/base:trace",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/cpu:thread_state",
        "//src/mem:page_table",
        "//src/mem:physical",
        "//src/mem:port",
        "//src/sim:full_system",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "semihosting",
    srcs = ["semihosting.cc"],
    hdrs = ["semihosting.hh"],
    deps = [
        ":arm_semihosting_sim_object",
        ":utility_hh",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/generic:debug_Semihosting",
        "//src/arch/generic:semihosting",
        "//src/base:output",
        "//src/base:time",
        "//src/cpu:thread_context",
        "//src/dev/serial",
        "//src/mem:physical",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:byteswap",
        "//src/sim:core",
        "//src/sim:full_system",
        "//src/sim:guest_abi",
        "//src/sim:pseudo_inst",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "decoder",
    srcs = ["decoder.cc"],
    hdrs = ["decoder.hh"],
    deps = [
        ":arm_decoder_sim_object",
        ":enum_DecoderFlavor_hh",
        ":isa_hh",
        ":types",
        ":utility_hh",
        "//src/arch:debug_Decoder",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/generic:decode_cache",
        "//src/arch/generic:decoder",
        "//src/base:cast",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:debug_Decode",
        "//src/cpu:static_inst",
        "//src/sim:full_system",
    ],
)

gem5_cc_library(
    name = "table_walker",
    srcs = ["table_walker.cc"],
    hdrs = ["table_walker.hh"],
    deps = [
        ":arm_table_walker_sim_object",
        ":arm_walk_unit_sim_object",
        ":enum_ArmLookupLevel_hh",
        ":faults_hh",
        ":mmu_hh",
        ":mpam",
        ":pagetable",
        ":system_hh",
        ":tlb_hh",
        ":types",
        "//src/arch:debug_TLBVerbose",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/generic:debug_PageTableWalker",
        "//src/arch/generic:debug_TLB",
        "//src/arch/generic:mmu",
        "//src/base:compiler",
        "//src/cpu:base",
        "//src/cpu:thread_context",
        "//src/mem:packet_queue",
        "//src/mem:qport",
        "//src/mem:request",
        "//src/sim:clocked_object",
        "//src/sim:debug_Checkpoint",
        "//src/sim:debug_Drain",
        "//src/sim:eventq",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "mpam",
    srcs = ["mpam.cc"],
    hdrs = ["mpam.hh"],
    deps = [
        ":debug_MPAM",
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_accessors",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:misc_types",
        "//src/base:extensible",
        "//src/cpu:base",
        "//src/cpu:thread_context",
        "//src/mem:packet",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "reg_abi",
    srcs = ["reg_abi.cc"],
    hdrs = ["reg_abi.hh"],
    deps = [
        "//src/arch/arm/regs:int_hh",
        "//src/base:logging",
        "//src/sim:pseudo_inst",
        "//src/sim:syscall_abi",
    ],
)

gem5_cc_library(
    name = "stage2_lookup",
    srcs = ["stage2_lookup.cc"],
    hdrs = ["stage2_lookup.hh"],
    deps = [
        ":faults_hh",
        ":system_hh",
        ":table_walker",
        ":tlb_hh",
        "//src/arch:debug_TLBVerbose",
        "//src/arch/generic:debug_TLB",
        "//src/arch/generic:mmu",
        "//src/cpu:base",
        "//src/cpu:thread_context",
        "//src/mem:request",
        "//src/sim:debug_Checkpoint",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "htm",
    srcs = ["htm.cc"],
    hdrs = ["htm.hh"],
    deps = [
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:vec",
        "//src/arch/generic:htm",
        "//src/base:types",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "pauth_helpers",
    srcs = ["pauth_helpers.cc"],
    hdrs = ["pauth_helpers.hh"],
    deps = [
        ":faults_hh",
        ":qarma",
        ":system_hh",
        ":types",
        ":utility_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/base:bitfield",
        "//src/base:bitunion",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "qarma",
    srcs = ["qarma.cc"],
    hdrs = ["qarma.hh"],
    deps = [
        "//src/base:bitfield",
        "//src/base:bitunion",
    ],
)

gem5_cc_library(
    name = "se_workload",
    hdrs = ["se_workload.hh"],
    deps = [
        ":arm_se_workload_sim_object",
        ":reg_abi",
        ":remote_gdb",
        "//src/sim:se_workload",
    ],
)

gem5_cc_library(
    name = "nativetrace",
    srcs = ["nativetrace.cc"],
    hdrs = ["nativetrace.hh"],
    deps = [
        ":arm_native_trace_sim_object",
        ":decoder",
        "//src/arch/arm/regs:cc",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/arch/arm/regs:vec",
        "//src/base:compiler",
        "//src/base:types",
        "//src/cpu:debug_ExecRegDelta",
        "//src/cpu:nativetrace",
        "//src/cpu:thread_context",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "pmu",
    srcs = ["pmu.cc"],
    hdrs = ["pmu.hh"],
    deps = [
        ":arm_pmu_sim_object",
        ":debug_PMUVerbose",
        ":enum_EventTypeId_hh",
        ":isa_device_hh",
        ":isa_hh",
        ":system_hh",
        ":utility_hh",
        "//src/base:cprintf",
        "//src/base:trace",
        "//src/cpu:base_hh",
        "//src/dev/arm:base_gic",
        "//src/dev/arm:generic_timer",
        "//src/mem/cache:cache_probe_arg",
        "//src/sim:debug_Checkpoint",
        "//src/sim:eventq",
        "//src/sim:sim_exit",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.hh"],
    deps = [
        ":page_size",
        ":types",
        "//src/arch/arm/regs:cc",
        "//src/arch/arm/regs:int_hh",
        "//src/arch/arm/regs:misc_hh",
        "//src/base:logging",
        "//src/base/loader:elf_object",
        "//src/base/loader:object_file",
        "//src/cpu:thread_context",
        "//src/mem:page_table",
        "//src/sim:aux_vector",
        "//src/sim:byteswap",
        "//src/sim:debug_Stack",
        "//src/sim:process",
        "//src/sim:process_impl",
        "//src/sim:process_sim_object",
        "//src/sim:syscall_abi",
        "//src/sim:syscall_return",
        "//src/sim:system",
    ],
)

gem5_sim_object(
    name = "arm_fs_linux_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/arch/arm:arm_fs_workload_sim_object",
    ],
    sim_object_header = "//src/arch/arm/linux:fs_workload",
    sim_object_module = "src.arch.arm.ArmFsWorkload",
    sim_object_name = "ArmFsLinux",
    sim_object_py_lib = ":ArmFsWorkload_py",
)

py_library(
    name = "arch_arm_init",
    srcs = ["__init__.py"],
    deps = [
        ":ArmCPU_py",
        ":ArmDecoder_py",
        ":ArmFsWorkload_py",
        ":ArmInterrupts_py",
        ":ArmISA_py",
        ":ArmMMU_py",
        ":ArmNativeTrace_py",
        ":ArmSemihosting_py",
        ":ArmSeWorkload_py",
        ":ArmSystem_py",
        ":ArmTLB_py",
        ":ArmPMU_py",
    ],
)
