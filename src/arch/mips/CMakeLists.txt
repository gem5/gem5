# src/arch/mips/CMakeLists.txt
# MIPS ISA

if(NOT CONF_USE_MIPS_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    decoder.cc
    dsp.cc
    faults.cc
    idle_event.cc
    interrupts.cc
    isa.cc
    linux/se_workload.cc
    pagetable.cc
    process.cc
    remote_gdb.cc
    se_workload.cc
    tlb.cc
    utility.cc
)

# --- SimObjects ---
gem5_add_simobject(MipsDecoder.py SIM_OBJECTS MipsDecoder)
gem5_add_simobject(MipsInterrupts.py SIM_OBJECTS MipsInterrupts)
gem5_add_simobject(MipsISA.py SIM_OBJECTS MipsISA)
gem5_add_simobject(MipsMMU.py SIM_OBJECTS MipsMMU)
gem5_add_simobject(MipsSeWorkload.py SIM_OBJECTS MipsSEWorkload MipsEmuLinux)
gem5_add_simobject(MipsTLB.py SIM_OBJECTS MipsTLB)
gem5_add_simobject(MipsCPU.py)

# --- Debug flags ---
gem5_add_debug_flag(MipsPRA "MIPS PRA debugging")

# --- ISA parser ---
gem5_isa_parser(mips
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/mips/generated")

# --- GDB XML ---
gem5_add_blob(gdb_xml_mips
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/mips.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_mips.cc")
