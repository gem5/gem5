# src/arch/power/CMakeLists.txt
# POWER ISA

if(NOT CONF_USE_POWER_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    decoder.cc
    faults.cc
    insts/branch.cc
    insts/mem.cc
    insts/integer.cc
    insts/floating.cc
    insts/condition.cc
    insts/static_inst.cc
    linux/se_workload.cc
    isa.cc
    pagetable.cc
    process.cc
    remote_gdb.cc
    se_workload.cc
    tlb.cc
)

# --- SimObjects ---
gem5_add_simobject(PowerDecoder.py SIM_OBJECTS PowerDecoder)
gem5_add_simobject(PowerInterrupts.py SIM_OBJECTS PowerInterrupts)
gem5_add_simobject(PowerISA.py SIM_OBJECTS PowerISA)
gem5_add_simobject(PowerMMU.py SIM_OBJECTS PowerMMU)
gem5_add_simobject(PowerSeWorkload.py SIM_OBJECTS PowerSEWorkload PowerEmuLinux)
gem5_add_simobject(PowerTLB.py SIM_OBJECTS PowerTLB)
gem5_add_simobject(PowerCPU.py)

# --- Debug flags ---
gem5_add_debug_flag(Power "POWER ISA debugging")

# --- ISA parser ---
gem5_isa_parser(power
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/power/generated")

# --- GDB XML ---
gem5_add_blob(gdb_xml_power_core
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/power-core.xml"
    "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power_core.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power_core.hh")
gem5_add_blob(gdb_xml_power64_core
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/power64-core.xml"
    "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power64_core.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power64_core.hh")
gem5_add_blob(gdb_xml_power_fpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/power-fpu.xml"
    "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power_fpu.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_power_fpu.hh")
gem5_add_blob(gdb_xml_powerpc_32
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/powerpc-32.xml"
    "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_powerpc_32.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_powerpc_32.hh")
gem5_add_blob(gdb_xml_powerpc_64
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/powerpc-64.xml"
    "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_powerpc_64.cc"
    OUTPUT_HH "${GEM5_GEN_DIR}/arch/power/gdb-xml/gdb_xml_powerpc_64.hh")
