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

#ifndef __DEV_AMDGPU_SDMA_COMMANDS_HH__
#define __DEV_AMDGPU_SDMA_COMMANDS_HH__

/**
 * Commands for the SDMA engine. The header files can be found here:
 *
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *      drivers/gpu/drm/amd/amdgpu/vega10_sdma_pkt_open.h
 */
#define SDMA_OP_NOP  0
#define SDMA_OP_COPY  1
#define SDMA_OP_WRITE  2
#define SDMA_OP_INDIRECT  4
#define SDMA_OP_FENCE  5
#define SDMA_OP_TRAP  6
#define SDMA_OP_SEM  7
#define SDMA_OP_POLL_REGMEM  8
#define SDMA_OP_COND_EXE  9
#define SDMA_OP_ATOMIC  10
#define SDMA_OP_CONST_FILL  11
#define SDMA_OP_PTEPDE  12
#define SDMA_OP_TIMESTAMP  13
#define SDMA_OP_SRBM_WRITE  14
#define SDMA_OP_PRE_EXE  15
#define SDMA_OP_DUMMY_TRAP  16
#define SDMA_SUBOP_TIMESTAMP_SET  0
#define SDMA_SUBOP_TIMESTAMP_GET  1
#define SDMA_SUBOP_TIMESTAMP_GET_GLOBAL  2
#define SDMA_SUBOP_COPY_LINEAR  0
#define SDMA_SUBOP_COPY_LINEAR_SUB_WIND  4
#define SDMA_SUBOP_COPY_TILED  1
#define SDMA_SUBOP_COPY_TILED_SUB_WIND  5
#define SDMA_SUBOP_COPY_T2T_SUB_WIND  6
#define SDMA_SUBOP_COPY_SOA  3
#define SDMA_SUBOP_COPY_DIRTY_PAGE  7
#define SDMA_SUBOP_COPY_LINEAR_PHY  8
#define SDMA_SUBOP_WRITE_LINEAR  0
#define SDMA_SUBOP_WRITE_TILED  1
#define SDMA_SUBOP_PTEPDE_GEN  0
#define SDMA_SUBOP_PTEPDE_COPY  1
#define SDMA_SUBOP_PTEPDE_RMW  2
#define SDMA_SUBOP_PTEPDE_COPY_BACKWARDS  3
#define SDMA_SUBOP_DATA_FILL_MULTI  1
#define SDMA_SUBOP_POLL_REG_WRITE_MEM  1
#define SDMA_SUBOP_POLL_DBIT_WRITE_MEM  2
#define SDMA_SUBOP_POLL_MEM_VERIFY  3
#define HEADER_AGENT_DISPATCH  4
#define HEADER_BARRIER  5
#define SDMA_OP_AQL_COPY  0
#define SDMA_OP_AQL_BARRIER_OR  0

#endif // __DEV_AMDGPU_SDMA_COMMANDS_HH__
