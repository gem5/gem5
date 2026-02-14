# src/arch/x86/CMakeLists.txt
# X86 ISA

if(NOT CONF_USE_X86_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    cpuid.cc
    decoder.cc
    decoder_tables.cc
    emulenv.cc
    faults.cc
    fs_workload.cc
    insts/badmicroop.cc
    insts/microop.cc
    insts/microregop.cc
    insts/static_inst.cc
    interrupts.cc
    isa.cc
    nativetrace.cc
    pagetable.cc
    pagetable_walker.cc
    process.cc
    remote_gdb.cc
    tlb.cc
    types.cc
    utility.cc
)

# bare_metal
gem5_add_source(bare_metal/workload.cc)

# bios
gem5_add_sources(
    bios/e820.cc
    bios/smbios.cc
    bios/intelmp.cc
    bios/acpi.cc
)

# linux
gem5_add_sources(
    linux/fs_workload.cc
    linux/se_workload.cc
    linux/syscalls.cc
    linux/syscall_tbl32.cc
    linux/syscall_tbl64.cc
)

# regs
gem5_add_sources(
    regs/float.cc
    regs/int.cc
    regs/msr.cc
)

# --- SimObjects ---
gem5_add_simobject(X86SeWorkload.py SIM_OBJECTS X86EmuLinux)
gem5_add_simobject(X86FsWorkload.py
    SIM_OBJECTS X86BareMetalWorkload X86FsWorkload X86FsLinux)
gem5_add_simobject(X86Decoder.py SIM_OBJECTS X86Decoder)
gem5_add_simobject(X86ISA.py SIM_OBJECTS X86ISA)
gem5_add_simobject(X86LocalApic.py SIM_OBJECTS X86LocalApic)
gem5_add_simobject(X86MMU.py SIM_OBJECTS X86MMU)
gem5_add_simobject(X86NativeTrace.py SIM_OBJECTS X86NativeTrace)
gem5_add_simobject(X86TLB.py SIM_OBJECTS X86PagetableWalker X86TLB)
gem5_add_simobject(X86CPU.py)

# bios SimObjects
gem5_add_simobject(bios/E820.py SIM_OBJECTS X86E820Entry X86E820Table)
gem5_add_simobject(bios/SMBios.py
    SIM_OBJECTS X86SMBiosSMBiosStructure X86SMBiosBiosInformation X86SMBiosSMBiosTable
    ENUMS Characteristic ExtCharacteristic)
gem5_add_simobject(bios/IntelMP.py
    SIM_OBJECTS
        X86IntelMPFloatingPointer X86IntelMPConfigTable
        X86IntelMPBaseConfigEntry X86IntelMPExtConfigEntry
        X86IntelMPProcessor X86IntelMPBus X86IntelMPIOAPIC
        X86IntelMPIOIntAssignment X86IntelMPLocalIntAssignment
        X86IntelMPAddrSpaceMapping X86IntelMPBusHierarchy
        X86IntelMPCompatAddrSpaceMod
    ENUMS
        X86IntelMPInterruptType X86IntelMPPolarity
        X86IntelMPTriggerMode X86IntelMPAddressType
        X86IntelMPRangeList)
gem5_add_simobject(bios/ACPI.py
    SIM_OBJECTS
        X86ACPISysDescTable X86ACPIRSDT X86ACPIXSDT
        X86ACPIMadtRecord X86ACPIMadt X86ACPIMadtLAPIC
        X86ACPIMadtIOAPIC X86ACPIMadtIntSourceOverride
        X86ACPIMadtNMI X86ACPIMadtLAPICOverride X86ACPIRSDP)

# --- Debug flags ---
gem5_add_debug_flag(LocalApic "Local APIC debugging")
gem5_add_debug_flag(X86 "Generic X86 ISA debugging")
gem5_add_debug_flag(ACPI "ACPI debugging")

# --- ISA parser ---
# X86 uses default splits (decoder_splits=1, exec_splits=1)
gem5_isa_parser(x86
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/x86/generated")

# --- KVM ---
if(CONF_KVM_ISA STREQUAL "x86")
    gem5_add_source(kvm/x86_cpu.cc)
    gem5_add_simobject(kvm/X86KvmCPU.py SIM_OBJECTS X86KvmCPU)
endif()
