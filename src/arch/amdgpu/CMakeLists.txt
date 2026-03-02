# src/arch/amdgpu/CMakeLists.txt
# AMD GPU ISA
#
# Test registration for common/dtype/mxfp.test.cc is in the parent
# src/arch/CMakeLists.txt (always parsed), because this file is only
# included when CONF_BUILD_GPU is true.

if(NOT CONF_BUILD_GPU)
    return()
endif()

# --- common/ ---
# X86 GPU TLB (for vega GPU ISA)
if(CONF_TARGET_GPU_ISA STREQUAL "vega")
    gem5_add_sources(
        common/tlb.cc
        common/tlb_coalescer.cc
    )
    gem5_add_simobject(common/X86GPUTLB.py SIM_OBJECTS X86GPUTLB TLBCoalescer)
endif()

# --- vega/ ---
gem5_add_sources(
    vega/faults.cc
    vega/pagetable.cc
    vega/pagetable_walker.cc
    vega/tlb.cc
    vega/tlb_coalescer.cc
)
gem5_add_simobject(vega/VegaGPUTLB.py
    SIM_OBJECTS VegaPagetableWalker VegaGPUTLB VegaTLBCoalescer VegaPWCIndexingPolicy)

gem5_add_debug_flag(GPUPTWalker "Debug flag for GPU page table walker")

# Vega instruction set (only when TARGET_GPU_ISA is vega)
if(CONF_TARGET_GPU_ISA STREQUAL "vega")
    gem5_add_sources(
        vega/gpu_decoder.cc
        vega/insts/gpu_static_inst.cc
        vega/insts/op_encodings.cc
        vega/gpu_isa.cc
        vega/gpu_registers.cc
        vega/insts/sop2.cc
        vega/insts/sopk.cc
        vega/insts/sop1.cc
        vega/insts/sopc.cc
        vega/insts/sopp.cc
        vega/insts/smem.cc
        vega/insts/vop2.cc
        vega/insts/vop1.cc
        vega/insts/vopc.cc
        vega/insts/vinterp.cc
        vega/insts/vop3.cc
        vega/insts/vop3_cmp.cc
        vega/insts/ds.cc
        vega/insts/mubuf.cc
        vega/insts/mtbuf.cc
        vega/insts/mimg.cc
        vega/insts/exp.cc
        vega/insts/flat.cc
        vega/insts/vop3p.cc
        vega/insts/vop3p_mai.cc
    )

    gem5_add_debug_flag(VEGA "Debug flag for VEGA GPU ISA")
endif()

add_subdirectory(common/dtype)
