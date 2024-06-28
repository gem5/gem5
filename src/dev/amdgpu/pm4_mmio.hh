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
 *
 */

#ifndef __DEV_AMDGPU_PM4_MMIO_HH__
#define __DEV_AMDGPU_PM4_MMIO_HH__

namespace gem5
{

#define mmCP_RB0_BASE                                                 0x040
#define mmCP_RB0_CNTL                                                 0x041
#define mmCP_RB_WPTR_POLL_ADDR_LO                                     0x046
#define mmCP_RB_WPTR_POLL_ADDR_HI                                     0x047
#define mmCP_RB_VMID                                                  0x051
#define mmCP_RB0_RPTR_ADDR                                            0x043
#define mmCP_RB0_RPTR_ADDR_HI                                         0x044
#define mmCP_RB0_WPTR                                                 0x054
#define mmCP_RB0_WPTR_HI                                              0x055
#define mmCP_RB_DOORBELL_CONTROL                                      0x059
#define mmCP_RB_DOORBELL_RANGE_LOWER                                  0x05a
#define mmCP_RB_DOORBELL_RANGE_UPPER                                  0x05b
#define mmCP_RB0_BASE_HI                                              0x0b1

#define mmCP_HQD_ACTIVE                                               0x247
#define mmCP_HQD_VMID                                                 0x248
#define mmCP_HQD_PQ_BASE                                              0x24d
#define mmCP_HQD_PQ_BASE_HI                                           0x24e
#define mmCP_HQD_PQ_DOORBELL_CONTROL                                  0x254
#define mmCP_HQD_PQ_RPTR                                              0x24f
#define mmCP_HQD_PQ_RPTR_REPORT_ADDR                                  0x250
#define mmCP_HQD_PQ_RPTR_REPORT_ADDR_HI                               0x251
#define mmCP_HQD_PQ_WPTR_POLL_ADDR                                    0x252
#define mmCP_HQD_PQ_WPTR_POLL_ADDR_HI                                 0x253
#define mmCP_HQD_PQ_CONTROL                                           0x256
#define mmCP_HQD_IB_CONTROL                                           0x25a
#define mmCP_HQD_PQ_WPTR_LO                                           0x27b
#define mmCP_HQD_PQ_WPTR_HI                                           0x27c

} // namespace gem5

#endif // __DEV_AMDGPU_PM4_MMIO_HH__
