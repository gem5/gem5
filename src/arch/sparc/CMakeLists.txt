# src/arch/sparc/CMakeLists.txt
# SPARC ISA

if(NOT CONF_USE_SPARC_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    asi.cc
    decoder.cc
    faults.cc
    fs_workload.cc
    insts/blockmem.cc
    insts/branch.cc
    insts/integer.cc
    insts/mem.cc
    insts/micro.cc
    insts/priv.cc
    insts/static_inst.cc
    insts/trap.cc
    isa.cc
    linux/se_workload.cc
    linux/syscalls.cc
    nativetrace.cc
    pagetable.cc
    process.cc
    regs/int.cc
    remote_gdb.cc
    se_workload.cc
    tlb.cc
    ua2005.cc
)

# --- SimObjects ---
gem5_add_simobject(SparcDecoder.py SIM_OBJECTS SparcDecoder)
gem5_add_simobject(SparcFsWorkload.py SIM_OBJECTS SparcFsWorkload)
gem5_add_simobject(SparcInterrupts.py SIM_OBJECTS SparcInterrupts)
gem5_add_simobject(SparcISA.py SIM_OBJECTS SparcISA)
gem5_add_simobject(SparcMMU.py SIM_OBJECTS SparcMMU)
gem5_add_simobject(SparcNativeTrace.py SIM_OBJECTS SparcNativeTrace)
gem5_add_simobject(SparcSeWorkload.py SIM_OBJECTS SparcSEWorkload SparcEmuLinux)
gem5_add_simobject(SparcTLB.py SIM_OBJECTS SparcTLB)
gem5_add_simobject(SparcCPU.py)

# --- Debug flags ---
gem5_add_debug_flag(Sparc "Generic SPARC ISA stuff")
gem5_add_debug_flag(RegisterWindows "Register window manipulation")

# --- ISA parser ---
gem5_isa_parser(sparc
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/sparc/generated")
