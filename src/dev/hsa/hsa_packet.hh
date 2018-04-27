/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 * Authors: Eric van Tassell
 */

#ifndef __DEV_HSA_HSA_PACKET__
#define __DEV_HSA_HSA_PACKET__

#define _HSA_PACKET_TYPE_VENDOR_SPECIFIC 0

#include <stdint.h>

typedef struct hsa_packet_header_s {
        // TODO: replace with more portable impl based on offset, length
        uint16_t type:8;
        uint16_t barrier:1;
        uint16_t acquire_fence_scope:2;
        uint16_t release_fence_scope:2;
        uint16_t reserved:3;
} hsa_packet_header_bitfield_t;

//TODO: put an _ in front of these guys to avoud prob with hsa.h for now
typedef struct _hsa_dispatch_packet_s {
    uint16_t header;
    uint16_t setup;
    uint16_t workgroup_size_x;
    uint16_t workgroup_size_y;
    uint16_t workgroup_size_z;
    uint16_t reserved0;
    uint32_t grid_size_x;
    uint32_t grid_size_y;
    uint32_t grid_size_z;
    uint32_t private_segment_size;
    uint32_t group_segment_size;
    uint64_t kernel_object;
    uint64_t kernarg_address;
    uint64_t reserved1;
    uint64_t completion_signal;
} _hsa_dispatch_packet_t;

typedef struct _hsa_agent_dispatch_packet_s {
    uint16_t header;
    uint16_t type;
    uint32_t reserved0;
    uint64_t return_address;
    uint64_t arg[4];
    uint64_t reserved2;
    uint64_t completion_signal;
} _hsa_agent_dispatch_packet_t;

typedef struct _hsa_barrier_and_packet_s {
    uint16_t header;
    uint16_t reserved0;
    uint32_t reserved1;
    uint64_t dep_signal[5];
    uint64_t reserved2;
    uint64_t completion_signal;
} _hsa_barrier_and_packet_t;

typedef struct _hsa_barrier_or_packet_s {
    uint16_t header;
    uint16_t reserved0;
    uint32_t reserved1;
    uint64_t dep_signal[5];
    uint64_t reserved2;
    uint64_t completion_signal;
} _hsa_barrier_or_packet_t;

#endif // __DEV_HSA_HSA_PACKET_HH__
