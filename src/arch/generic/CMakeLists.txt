# src/arch/generic/CMakeLists.txt
# Base architecture classes used by all ISAs.

# --- Sources ---
gem5_add_source(htm.cc)
gem5_add_source(mmu.cc)
gem5_add_source(decoder.cc)

# --- SimObjects ---
gem5_add_simobject(BaseInterrupts.py SIM_OBJECTS BaseInterrupts)
gem5_add_simobject(BaseISA.py SIM_OBJECTS BaseISA)
gem5_add_simobject(BaseMMU.py SIM_OBJECTS BaseMMU)
gem5_add_simobject(BaseTLB.py SIM_OBJECTS BaseTLB ENUMS TypeTLB)
gem5_add_simobject(InstDecoder.py SIM_OBJECTS InstDecoder)

# --- Debug flags ---
gem5_add_debug_flag(PageTableWalker "Page table walker state machine debugging")
gem5_add_debug_flag(TLB "TLB debugging")

# --- Semihosting (ARM and RISC-V only) ---
if(CONF_USE_ARM_ISA OR CONF_USE_RISCV_ISA)
    gem5_add_source(semihosting.cc)
    gem5_add_simobject(BaseSemihosting.py SIM_OBJECTS BaseSemihosting)
    gem5_add_debug_flag(Semihosting "Semihosting debugging")
endif()
