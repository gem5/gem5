/*
 * Copyright (c) 2016-2017 Advanced Micro Devices, Inc.
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
 * Authors: Anthony Gutierrez
 */

#ifndef __DEV_HSA_HSA_QUEUE_HH__
#define __DEV_HSA_HSA_QUEUE_HH__

#include <cstdint>

typedef enum
{
    _HSA_QUEUE_TYPE_MULTI = 0,
    _HSA_QUEUE_TYPE_SINGLE = 1
} _hsa_queue_type_t;

typedef struct _hsa_signal_s
{
    uint64_t handle;
} _hsa_signal_t;

typedef struct _hsa_queue_s
{
    _hsa_queue_type_t type;
    uint32_t features;
    void *base_address;
    _hsa_signal_t doorbell_signal;
    uint32_t size;
    uint32_t reserved1;
    uint64_t id;
} _hsa_queue_t;

typedef uint32_t _amd_queue_properties32_t;

typedef struct _amd_queue_s
{
    _hsa_queue_t hsa_queue;
    uint32_t reserved1[4];
    volatile uint64_t write_dispatch_id;
    uint32_t group_segment_aperture_base_hi;
    uint32_t private_segment_aperture_base_hi;
    uint32_t max_cu_id;
    uint32_t max_wave_id;
    volatile uint64_t max_legacy_doorbell_dispatch_id_plus_1;
    volatile uint32_t legacy_doorbell_lock;
    uint32_t reserved2[9];
    volatile uint64_t read_dispatch_id;
    uint32_t read_dispatch_id_field_base_byte_offset;
    uint32_t compute_tmpring_size_waves : 12;
    uint32_t compute_tmpring_size_wavesize : 13;
    uint32_t compute_tmpring_size_pad : 7;
    uint32_t scratch_resource_descriptor[4];
    uint64_t scratch_backing_memory_location;
    uint64_t scratch_backing_memory_byte_size;
    uint32_t scratch_workitem_byte_size;
    _amd_queue_properties32_t queue_properties;
    uint32_t reserved3[2];
    _hsa_signal_t queue_inactive_signal;
    uint32_t reserved4[14];
} _amd_queue_t;

#endif // __DEV_HSA_HSA_QUEUE_HH__
