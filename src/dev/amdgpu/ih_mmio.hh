/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __DEV_AMDGPU_IH_MMIO_HH__
#define __DEV_AMDGPU_IH_MMIO_HH__

/**
 * MMIO offsets for interrupt handler. These values were taken from the linux
 * header for IH. The header files can be found here:
 *
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *      drivers/gpu/drm/amd/include/asic_reg/oss/osssys_4_0_offset.h
 */
#define mmIH_RB_CNTL 0x0080
#define mmIH_RB_BASE 0x0081
#define mmIH_RB_BASE_HI 0x0082
#define mmIH_RB_RPTR 0x0083
#define mmIH_RB_WPTR 0x0084
#define mmIH_RB_WPTR_ADDR_HI 0x0085
#define mmIH_RB_WPTR_ADDR_LO 0x0086
#define mmIH_DOORBELL_RPTR 0x0087

#endif // __DEV_AMDGPU_IH_MMIO_HH__
