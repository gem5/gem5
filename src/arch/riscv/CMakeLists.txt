# src/arch/riscv/CMakeLists.txt
# RISC-V ISA

if(NOT CONF_USE_RISCV_ISA)
    return()
endif()

# --- Sources ---
gem5_add_sources(
    decoder.cc
    faults.cc
    interrupts.cc
    isa.cc
    process.cc
    pagetable.cc
    pagetable_walker.cc
    pma_checker.cc
    pmp.cc
    reg_abi.cc
    remote_gdb.cc
    semihosting.cc
    system.cc
    tlb.cc
    linux/se_workload.cc
    linux/fs_workload.cc
    bare_metal/fs_workload.cc
)

# --- SimObjects ---
gem5_add_simobject(PMAChecker.py SIM_OBJECTS PMAChecker BasePMAChecker)
gem5_add_simobject(PMP.py SIM_OBJECTS PMP)
gem5_add_simobject(RiscvDecoder.py SIM_OBJECTS RiscvDecoder)
gem5_add_simobject(RiscvFsWorkload.py
    SIM_OBJECTS RiscvBareMetal RiscvLinux RiscvBootloaderKernelWorkload)
gem5_add_simobject(RiscvInterrupts.py SIM_OBJECTS RiscvInterrupts)
gem5_add_simobject(RiscvISA.py SIM_OBJECTS RiscvISA ENUMS RiscvType PrivilegeModeSet)
gem5_add_simobject(RiscvMMU.py SIM_OBJECTS RiscvMMU)
gem5_add_simobject(RiscvSemihosting.py SIM_OBJECTS RiscvSemihosting)
gem5_add_simobject(RiscvSeWorkload.py SIM_OBJECTS RiscvSEWorkload RiscvEmuLinux)
gem5_add_simobject(RiscvTLB.py SIM_OBJECTS RiscvPagetableWalker RiscvTLB)
gem5_add_simobject(RiscvCPU.py)
gem5_add_simobject(RiscvSystem.py SIM_OBJECTS RiscvSystem)

# --- Debug flags ---
gem5_add_debug_flag(RiscvMisc "RISC-V miscellaneous debugging")
gem5_add_debug_flag(PMP "PMP debugging")

# --- ISA parser ---
gem5_isa_parser(riscv
    "${CMAKE_CURRENT_SOURCE_DIR}/isa/main.isa"
    "${GEM5_GEN_DIR}/arch/riscv/generated")

# --- GDB XML ---
gem5_add_blob(gdb_xml_riscv_32bit_target
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-32bit.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_32bit_target.cc")
gem5_add_blob(gdb_xml_riscv_32bit_cpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-32bit-cpu.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_32bit_cpu.cc")
gem5_add_blob(gdb_xml_riscv_32bit_fpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-32bit-fpu.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_32bit_fpu.cc")
gem5_add_blob(gdb_xml_riscv_32bit_csr
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-32bit-csr.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_32bit_csr.cc")
gem5_add_blob(gdb_xml_riscv_64bit_target
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-64bit.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_64bit_target.cc")
gem5_add_blob(gdb_xml_riscv_64bit_cpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-64bit-cpu.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_64bit_cpu.cc")
gem5_add_blob(gdb_xml_riscv_64bit_fpu
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-64bit-fpu.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_64bit_fpu.cc")
gem5_add_blob(gdb_xml_riscv_64bit_csr
    "${CMAKE_CURRENT_SOURCE_DIR}/gdb-xml/riscv-64bit-csr.xml"
    "${GEM5_GEN_DIR}/blobs/gdb_xml_riscv_64bit_csr.cc")
