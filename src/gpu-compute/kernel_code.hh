/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GPU_COMPUTE_KERNEL_CODE_HH__
#define __GPU_COMPUTE_KERNEL_CODE_HH__

#include <bitset>
#include <cstdint>

namespace gem5
{

/**
 * these enums represent the indices into the
 * initialRegState bitfields in HsaKernelInfo.
 * each bit specifies whether or not the
 * particular piece of state that the bit
 * corresponds to should be initialized into
 * the VGPRs/SGPRs. the order in which the
 * fields are placed matters, as all enabled
 * pieces of state will be initialized into
 * contiguous registers in the same order
 * as their position in the bitfield - which
 * is specified in the HSA ABI.
 */
enum ScalarRegInitFields : int
{
    PrivateSegBuf = 0,
    DispatchPtr = 1,
    QueuePtr = 2,
    KernargSegPtr = 3,
    DispatchId = 4,
    FlatScratchInit = 5,
    PrivateSegSize = 6,
    GridWorkgroupCountX = 7,
    GridWorkgroupCountY = 8,
    GridWorkgroupCountZ = 9,
    WorkgroupIdX = 10,
    WorkgroupIdY = 11,
    WorkgroupIdZ = 12,
    WorkgroupInfo = 13,
    PrivSegWaveByteOffset = 14,
    NumScalarInitFields = 15
};

enum VectorRegInitFields : int
{
    WorkitemIdX = 0,
    WorkitemIdY = 1,
    WorkitemIdZ = 2,
    NumVectorInitFields = 3
};

struct AMDKernelCode
{
    uint32_t amd_kernel_code_version_major;
    uint32_t amd_kernel_code_version_minor;
    uint16_t amd_machine_kind;
    uint16_t amd_machine_version_major;
    uint16_t amd_machine_version_minor;
    uint16_t amd_machine_version_stepping;
    int64_t kernel_code_entry_byte_offset;
    int64_t kernel_code_prefetch_byte_offset;
    uint64_t kernel_code_prefetch_byte_size;
    uint64_t max_scratch_backing_memory_byte_size;

    /**
     * The fields below are used to set program settings for
     * compute shaders. Here they are primarily used to setup
     * initial register state. See the following for full details
     * about kernel launch, state initialization, and the AMD kernel
     * code object: https://github.com/RadeonOpenCompute/ROCm_Documentation/
     *              blob/master/ROCm_Compiler_SDK/ROCm-Codeobj-format.rst
     *              #initial-kernel-register-state
     */

    // the 32b below here represent the fields of
    // the COMPUTE_PGM_RSRC1 register
    uint32_t granulated_workitem_vgpr_count : 6;
    uint32_t granulated_wavefront_sgpr_count : 4;
    uint32_t priority : 2;
    uint32_t float_mode_round_32 : 2;
    uint32_t float_mode_round_16_64 : 2;
    uint32_t float_mode_denorm_32 : 2;
    uint32_t float_mode_denorm_16_64 : 2;
    uint32_t priv : 1;
    uint32_t enable_dx10_clamp : 1;
    uint32_t debug_mode : 1;
    uint32_t enable_ieee_mode : 1;
    uint32_t bulky : 1;
    uint32_t cdbg_user : 1;
    uint32_t compute_pgm_rsrc1_reserved : 6;
    // end COMPUTE_PGM_RSRC1 register

    // the 32b below here represent the fields of
    // the COMPUTE_PGM_RSRC2 register
    uint32_t enable_sgpr_private_segment_wave_byte_offset : 1;
    uint32_t user_sgpr_count : 5;
    uint32_t enable_trap_handler : 1;
    uint32_t enable_sgpr_workgroup_id_x : 1;
    uint32_t enable_sgpr_workgroup_id_y : 1;
    uint32_t enable_sgpr_workgroup_id_z : 1;
    uint32_t enable_sgpr_workgroup_info : 1;
    uint32_t enable_vgpr_workitem_id : 2;
    uint32_t enable_exception_address_watch : 1;
    uint32_t enable_exception_memory_violation : 1;
    uint32_t granulated_lds_size : 9;
    uint32_t enable_exception_ieee_754_fp_invalid_operation : 1;
    uint32_t enable_exception_fp_denormal_source : 1;
    uint32_t enable_exception_ieee_754_fp_division_by_zero : 1;
    uint32_t enable_exception_ieee_754_fp_overflow : 1;
    uint32_t enable_exception_ieee_754_fp_underflow : 1;
    uint32_t enable_exception_ieee_754_fp_inexact : 1;
    uint32_t enable_exception_int_divide_by_zero : 1;
    uint32_t compute_pgm_rsrc2_reserved : 1;
    // end COMPUTE_PGM_RSRC2

    // the 32b below here represent the fields of
    // KERNEL_CODE_PROPERTIES
    uint32_t enable_sgpr_private_segment_buffer : 1;
    uint32_t enable_sgpr_dispatch_ptr : 1;
    uint32_t enable_sgpr_queue_ptr : 1;
    uint32_t enable_sgpr_kernarg_segment_ptr : 1;
    uint32_t enable_sgpr_dispatch_id : 1;
    uint32_t enable_sgpr_flat_scratch_init : 1;
    uint32_t enable_sgpr_private_segment_size : 1;
    uint32_t enable_sgpr_grid_workgroup_count_x : 1;
    uint32_t enable_sgpr_grid_workgroup_count_y : 1;
    uint32_t enable_sgpr_grid_workgroup_count_z : 1;
    uint32_t kernel_code_properties_reserved1 : 6;
    uint32_t enable_ordered_append_gds : 1;
    uint32_t private_element_size : 2;
    uint32_t is_ptr64 : 1;
    uint32_t is_dynamic_callstack : 1;
    uint32_t is_debug_enabled : 1;
    uint32_t is_xnack_enabled : 1;
    uint32_t kernel_code_properties_reserved2 : 9;
    // end KERNEL_CODE_PROPERTIES

    uint32_t workitem_private_segment_byte_size;
    uint32_t workgroup_group_segment_byte_size;
    uint32_t gds_segment_byte_size;
    uint64_t kernarg_segment_byte_size;
    uint32_t workgroup_fbarrier_count;
    uint16_t wavefront_sgpr_count;
    uint16_t workitem_vgpr_count;
    uint16_t reserved_vgpr_first;
    uint16_t reserved_vgpr_count;
    uint16_t reserved_sgpr_first;
    uint16_t reserved_sgpr_count;
    uint16_t debug_wavefront_private_segment_offset_sgpr;
    uint16_t debug_private_segment_buffer_sgpr;
    uint8_t kernarg_segment_alignment;
    uint8_t group_segment_alignment;
    uint8_t private_segment_alignment;
    uint8_t wavefront_size;
    int32_t call_convention;
    uint8_t reserved[12];
    uint64_t runtime_loader_kernel_symbol;
    uint64_t control_directives[16];
};

} // namespace gem5

#endif // __GPU_COMPUTE_KERNEL_CODE_HH__
