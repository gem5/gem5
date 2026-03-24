# cmake/Gem5SwitchingHeaders.cmake
# Generate ISA and GPU switching headers.
#
# In the SCons build, switching headers are generated in src/arch/
# to redirect includes to the correct ISA-specific headers.
# For example, when USE_X86_ISA is set, including "arch/decoder.hh"
# should resolve to "arch/x86/decoder.hh".
#
# GPU switching headers similarly redirect to the correct GPU ISA.
# For example, "arch/gpu_decoder.hh" -> "arch/amdgpu/vega/gpu_decoder.hh".
#
# These are generated at configure time to ${GEM5_GEN_DIR}/arch/.

file(MAKE_DIRECTORY "${GEM5_GEN_DIR}/arch")

# ---------------------------------------------------------------------------
# GPU switching headers
# ---------------------------------------------------------------------------
if(CONF_BUILD_GPU AND CONF_TARGET_GPU_ISA)
    set(_gpu_headers
        gpu_decoder.hh
        gpu_isa.hh
        gpu_registers.hh
        gpu_types.hh
    )

    # Determine the GPU ISA subdirectory path
    set(_gpu_isa "${CONF_TARGET_GPU_ISA}")
    # The amdgpu ISAs (like vega) are under arch/amdgpu/<isa>/
    set(_amdgpu_isas "vega")
    list(FIND _amdgpu_isas "${_gpu_isa}" _is_amdgpu)
    if(NOT _is_amdgpu EQUAL -1)
        set(_gpu_isa_dir "amdgpu/${_gpu_isa}")
    else()
        set(_gpu_isa_dir "${_gpu_isa}")
    endif()

    foreach(_hdr ${_gpu_headers})
        gem5_write_if_unchanged("${GEM5_GEN_DIR}/arch/${_hdr}"
            "#include \"arch/${_gpu_isa_dir}/${_hdr}\"\n"
        )
    endforeach()

    message(STATUS "Generated GPU switching headers -> arch/${_gpu_isa_dir}/")
endif()

# ---------------------------------------------------------------------------
# ISA switching headers (will be fully populated in Phase 4 when src/arch/
# CMakeLists.txt is created, since the list of headers depends on each ISA)
#
# The generic function gem5_add_switching_header() in Gem5CodeGen.cmake
# handles individual headers.  This module provides the arch-level
# infrastructure.
# ---------------------------------------------------------------------------

# For now, record the list of active ISAs so src/arch/CMakeLists.txt can use them.
set(_isa_conf_pairs
    CONF_USE_X86_ISA   x86
    CONF_USE_ARM_ISA   arm
    CONF_USE_RISCV_ISA riscv
    CONF_USE_SPARC_ISA sparc
    CONF_USE_MIPS_ISA  mips
    CONF_USE_POWER_ISA power
)
set(GEM5_ACTIVE_ISAS "")
list(LENGTH _isa_conf_pairs _isa_len)
math(EXPR _isa_last "${_isa_len} - 1")
foreach(_i RANGE 0 ${_isa_last} 2)
    math(EXPR _j "${_i} + 1")
    list(GET _isa_conf_pairs ${_i} _var)
    list(GET _isa_conf_pairs ${_j} _name)
    if(${_var})
        list(APPEND GEM5_ACTIVE_ISAS "${_name}")
    endif()
endforeach()

message(STATUS "Active ISAs: ${GEM5_ACTIVE_ISAS}")
