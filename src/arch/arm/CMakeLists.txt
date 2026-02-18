# src/arch/arm/CMakeLists.txt
# ARM ISA

# fplib.cc is compiled unconditionally (used by amdgpu too).
# See comment in original SConscript about amdgpu dependencies.
gem5_add_source(insts/fplib.cc)

if(NOT CONF_USE_ARM_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    decoder.cc
    faults.cc
    htm.cc
    insts/branch.cc
    insts/branch64.cc
    insts/data64.cc
    insts/macromem.cc
    insts/mem.cc
    insts/mem64.cc
    insts/misc.cc
    insts/misc64.cc
    insts/pred_inst.cc
    insts/pseudo.cc
    insts/sme.cc
    insts/static_inst.cc
    insts/sve.cc
    insts/sve_mem.cc
    insts/sve_macromem.cc
    insts/vfp.cc
    insts/crypto.cc
    insts/tme64.cc
    interrupts.cc
    isa.cc
    isa_device.cc
    linux/linux.cc
    linux/process.cc
    linux/se_workload.cc
    linux/fs_workload.cc
    freebsd/fs_workload.cc
    freebsd/se_workload.cc
    fs_workload.cc
    regs/int.cc
    regs/misc.cc
    regs/misc_info.cc
    mmu.cc
    mpam.cc
    nativetrace.cc
    pagetable.cc
    pauth_helpers.cc
    pmu.cc
    process.cc
    qarma.cc
    remote_gdb.cc
    reg_abi.cc
    semihosting.cc
    system.cc
    table_walker.cc
    self_debug.cc
    stage2_lookup.cc
    tlb.cc
    tlbi_op.cc
    utility.cc
)

# TME64: Ruby protocol variant
if(CONF_RUBY_PROTOCOL_MESI_Three_Level_HTM)
    gem5_add_source(insts/tme64ruby.cc)
else()
    gem5_add_source(insts/tme64classic.cc)
endif()

# --- SimObjects ---
gem5_add_simobject(ArmDecoder.py SIM_OBJECTS ArmDecoder)
gem5_add_simobject(ArmFsWorkload.py
    SIM_OBJECTS ArmFsWorkload ArmFsLinux ArmFsFreebsd
    ENUMS ArmMachineType)
gem5_add_simobject(ArmInterrupts.py SIM_OBJECTS ArmInterrupts)
gem5_add_simobject(ArmISA.py SIM_OBJECTS ArmISA ENUMS DecoderFlavor)
gem5_add_simobject(ArmMMU.py SIM_OBJECTS ArmTableWalker ArmWalkUnit ArmMMU)
gem5_add_simobject(ArmNativeTrace.py SIM_OBJECTS ArmNativeTrace)
gem5_add_simobject(ArmSemihosting.py SIM_OBJECTS ArmSemihosting)
gem5_add_simobject(ArmSeWorkload.py
    SIM_OBJECTS ArmSEWorkload ArmEmuLinux ArmEmuFreebsd)
gem5_add_simobject(ArmSystem.py
    SIM_OBJECTS ArmSystem ArmRelease
    ENUMS ArmExtension)
gem5_add_simobject(ArmTLB.py
    SIM_OBJECTS ArmTLB TLBIndexingPolicy TLBSetAssociative
    ENUMS ArmLookupLevel)
gem5_add_simobject(ArmPMU.py SIM_OBJECTS ArmPMU ENUMS EventTypeId)
gem5_add_simobject(ArmCPU.py)

# --- Debug flags ---
gem5_add_debug_flag(Arm "ARM ISA debugging")
gem5_add_debug_flag(ArmTme "Transactional Memory Extension")
gem5_add_debug_flag(MPAM "MPAM debug flag")
gem5_add_debug_flag(PMUVerbose "Performance Monitor")

# --- ISA parser ---
# ARM uses decoder_splits=3, exec_splits=6
gem5_isa_parser(arm
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/arm/generated"
    DECODER_SPLITS 3
    EXEC_SPLITS 6)

# --- Tracers ---
gem5_add_sources(
    tracers/tarmac_base.cc
    tracers/tarmac_parser.cc
    tracers/tarmac_tracer.cc
    tracers/tarmac_record.cc
    tracers/tarmac_record_v8.cc
)
gem5_add_simobject(tracers/TarmacTrace.py
    SIM_OBJECTS TarmacParser TarmacTracer
    ENUMS TarmacDump)

if(HAVE_CAPSTONE)
    gem5_add_source(tracers/capstone.cc)
    gem5_add_simobject(tracers/ArmCapstone.py
        SIM_OBJECTS ArmCapstoneDisassembler)
endif()

# --- GDB XML ---
gem5_add_blob(gdb_xml_arm_target
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/arm-with-neon.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_target.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_target.hh")
gem5_add_blob(gdb_xml_arm_core
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/arm-core.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_core.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_core.hh")
gem5_add_blob(gdb_xml_arm_vfpv3
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/arm-vfpv3.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_vfpv3.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_arm_vfpv3.hh")
gem5_add_blob(gdb_xml_aarch64_target
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/aarch64.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_target.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_target.hh")
gem5_add_blob(gdb_xml_aarch64_core
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/aarch64-core.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_core.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_core.hh")
gem5_add_blob(gdb_xml_aarch64_fpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/aarch64-fpu.xml"
    "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_fpu.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/arm/gdb-xml/gdb_xml_aarch64_fpu.hh")

# --- KVM ---
if(CONF_KVM_ISA STREQUAL "arm")
    gem5_add_sources(
        kvm/gic.cc
        kvm/base_cpu.cc
    )
    gem5_add_simobject(kvm/KvmGic.py SIM_OBJECTS MuxingKvmGicV2 MuxingKvmGicV3)
    gem5_add_simobject(kvm/BaseArmKvmCPU.py SIM_OBJECTS BaseArmKvmCPU)

    if(GEM5_BIN_TARGET_ARCH STREQUAL "aarch64")
        gem5_add_source(kvm/armv8_cpu.cc)
        gem5_add_simobject(kvm/ArmV8KvmCPU.py SIM_OBJECTS ArmV8KvmCPU)
    else()
        gem5_add_source(kvm/arm_cpu.cc)
        gem5_add_simobject(kvm/ArmKvmCPU.py SIM_OBJECTS ArmKvmCPU)
    endif()
endif()

# --- Fast Model (optional) ---
if(CONF_USE_ARM_FASTMODEL)
    gem5_add_sources(
        fastmodel/fastmodel.cc
        fastmodel/amba_to_tlm_bridge.cc
        fastmodel/amba_from_tlm_bridge.cc
        fastmodel/remote_gdb.cc
    )
    gem5_add_pysource(m5 fastmodel/arm_fast_model.py)
    gem5_add_simobject(fastmodel/FastModel.py
        SIM_OBJECTS AmbaToTlmBridge64 AmbaFromTlmBridge64)

    # Iris
    gem5_add_sources(
        fastmodel/iris/cpu.cc
        fastmodel/iris/interrupts.cc
        fastmodel/iris/isa.cc
        fastmodel/iris/tlb.cc
        fastmodel/iris/thread_context.cc
    )
    gem5_add_simobject(fastmodel/iris/Iris.py
        SIM_OBJECTS IrisTLB IrisMMU IrisInterrupts IrisISA IrisBaseCPU)

    # Cortex-A76
    gem5_add_sources(
        fastmodel/CortexA76/cortex_a76.cc
        fastmodel/CortexA76/evs.cc
        fastmodel/CortexA76/thread_context.cc
    )
    gem5_add_simobject(fastmodel/CortexA76/FastModelCortexA76.py
        SIM_OBJECTS
            FastModelCortexA76 FastModelCortexA76Cluster
            FastModelScxEvsCortexA76x1 FastModelScxEvsCortexA76x2
            FastModelScxEvsCortexA76x3 FastModelScxEvsCortexA76x4)

    # Cortex-R52
    gem5_add_sources(
        fastmodel/CortexR52/cortex_r52.cc
        fastmodel/CortexR52/evs.cc
        fastmodel/CortexR52/thread_context.cc
    )
    gem5_add_simobject(fastmodel/CortexR52/FastModelCortexR52.py
        SIM_OBJECTS
            FastModelCortexR52 FastModelCortexR52Cluster
            FastModelScxEvsCortexR52x1 FastModelScxEvsCortexR52x2
            FastModelScxEvsCortexR52x3 FastModelScxEvsCortexR52x4)

    # GIC
    gem5_add_source(fastmodel/GIC/gic.cc)
    gem5_add_simobject(fastmodel/GIC/FastModelGIC.py
        SIM_OBJECTS SCFastModelGIC FastModelGIC)

    # Reset controller
    gem5_add_source(fastmodel/reset_controller/example.cc)
    gem5_add_simobject(fastmodel/reset_controller/FastModelResetControllerExample.py
        SIM_OBJECTS FastModelResetControllerExample)
endif()
