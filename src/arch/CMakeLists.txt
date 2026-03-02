# src/arch/CMakeLists.txt
# Top-level architecture directory.
# Includes generic base classes, then each ISA conditionally.

# --- Debug flags (always available, used by generic arch code) ---
gem5_add_debug_flag(IntRegs "Integer register debugging")
gem5_add_debug_flag(FloatRegs "Floating-point register debugging")
gem5_add_debug_flag(VecRegs "Vector register debugging")
gem5_add_debug_flag(VecPredRegs "Vector predicate register debugging")
gem5_add_debug_flag(MatRegs "Matrix register debugging")
gem5_add_debug_flag(CCRegs "Condition code register debugging")
gem5_add_debug_flag(MiscRegs "Miscellaneous register debugging")
gem5_add_debug_flag(Registers "All register debugging"
    COMPONENTS IntRegs FloatRegs VecRegs VecPredRegs MatRegs CCRegs MiscRegs)

gem5_add_debug_flag(Decoder "Decoder debug output")
gem5_add_debug_flag(Faults "Information about faults, exceptions, interrupts, etc")
gem5_add_debug_flag(TLBVerbose "TLB verbose debugging")

# --- Generic base classes (always included) ---
add_subdirectory(generic)

# --- ISA subdirectories (conditional) ---
if(CONF_BUILD_ISA)
    add_subdirectory(arm)
    add_subdirectory(mips)
    add_subdirectory(power)
    add_subdirectory(riscv)
    add_subdirectory(sparc)
    add_subdirectory(x86)
endif()

# --- GPU ISA ---
# Register GPU tests here (always parsed) so the safety check in
# Gem5Targets.cmake sees them even when CONF_BUILD_GPU is false.
gem5_add_test_source(amdgpu/common/dtype/mxfp.test.cc CONDITION CONF_BUILD_GPU)

if(CONF_BUILD_GPU)
    add_subdirectory(amdgpu)
endif()
