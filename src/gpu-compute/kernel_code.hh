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
    WorkgroupIdX = 7,
    WorkgroupIdY = 8,
    WorkgroupIdZ = 9,
    WorkgroupInfo = 10,
    PrivSegWaveByteOffset = 11,
    NumScalarInitFields = 12
};

enum VectorRegInitFields : int
{
    WorkitemIdX = 0,
    WorkitemIdY = 1,
    WorkitemIdZ = 2,
    NumVectorInitFields = 3
};

// Kernel code object based on the table on LLVM's website:
// https://llvm.org/docs/AMDGPUUsage.html#code-object-v3-kernel-descriptor
typedef struct GEM5_PACKED
{
    uint32_t group_segment_fixed_size;
    uint32_t private_segment_fixed_size;
    uint32_t kernarg_size;
    uint8_t reserved0[4];
    int64_t kernel_code_entry_byte_offset;
    uint8_t reserved1[20];

    // the 32b below here represent the fields of
    // the COMPUTE_PGM_RSRC3 register for GFX90A, GFX940
    uint32_t accum_offset : 6;
    uint32_t compute_pgm_rsrc3_reserved1 : 10;
    uint32_t tg_split : 1;
    uint32_t compute_pgm_rsrc3_reserved2 : 15;
    // end COMPUTE_PGM_RSRC3 register

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
    uint32_t fp16_ovfl : 1;
    uint32_t compute_pgm_rsrc1_reserved : 2;
    uint32_t wgp_mode : 1;
    uint32_t mem_ordered : 1;
    uint32_t fwd_progress : 1;
    // end COMPUTE_PGM_RSRC1 register

    // the 32b below here represent the fields of
    // the COMPUTE_PGM_RSRC2 register
    uint32_t enable_private_segment : 1;
    uint32_t user_sgpr_count : 5;
    uint32_t enable_trap_handler : 1;
    uint32_t enable_sgpr_workgroup_id_x : 1;
    uint32_t enable_sgpr_workgroup_id_y : 1;
    uint32_t enable_sgpr_workgroup_id_z : 1;
    uint32_t enable_sgpr_workgroup_info : 1;
    uint32_t enable_vgpr_workitem_id : 2;
    uint32_t enable_exception_address_watch : 1;
    uint32_t enable_exception_memory : 1;
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
    uint32_t kernel_code_properties_reserved1 : 3;
    uint32_t enable_wavefront_size32 : 1;
    uint32_t use_dynamic_stack : 1;
    uint32_t kernel_code_properties_reserved2 : 4;
    // end KERNEL_CODE_PROPERTIES

    uint32_t kernarg_preload_spec_length : 7;
    uint32_t kernarg_preload_spec_offset : 9;
    uint8_t reserved2[4];
} AMDKernelCode;

static_assert(sizeof(AMDKernelCode) == 64);

} // namespace gem5

#endif // __GPU_COMPUTE_KERNEL_CODE_HH__
