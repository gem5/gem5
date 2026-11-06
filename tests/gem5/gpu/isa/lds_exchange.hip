/*
 * Copyright (c) 2026 Boyana Norris
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

#include "test_common.hh"

namespace
{

constexpr unsigned NumLanes = 64;
constexpr uint32_t Sentinel = 0xdeadbeef;
constexpr uint64_t EvenLanes = 0x5555555555555555ULL;

// Offset, masked, empty-EXEC, and contention expectations follow exchange
// semantics. These cases have not been measured on physical hardware.
enum Test
{
    Independent,
    Offset,
    Masked,
    Empty,
    Contended,
    NumTests
};

struct Output
{
    uint32_t returned[NumTests], memory[NumTests];
    uint32_t neighbor, gap;
};

__device__ uint32_t
exchange(uint32_t address, uint32_t replacement)
{
    uint32_t result;
    asm volatile("ds_wrxchg_rtn_b32 %0, %1, %2\n\t"
                 "s_waitcnt lgkmcnt(0)\n\t"
                 : "=v"(result)
                 : "v"(address), "v"(replacement)
                 : "memory");
    return result;
}

__device__ uint32_t
maskedExchange(uint32_t address, uint32_t replacement, uint64_t mask)
{
    uint32_t result = Sentinel;
    uint64_t saved_exec;
    // EXEC has its original value when the asm statement finishes.
    asm volatile("s_and_saveexec_b64 %1, %4\n\t"
                 "ds_wrxchg_rtn_b32 %0, %2, %3\n\t"
                 "s_waitcnt lgkmcnt(0)\n\t"
                 "s_mov_b64 exec, %1\n\t"
                 : "+&v"(result), "=&s"(saved_exec)
                 : "v"(address), "v"(replacement), "s"(mask)
                 : "scc", "memory");
    return result;
}

__global__ void
ldsExchange(Output *outputs)
{
    // With no static __shared__ data, dynamic LDS starts at address zero.
    // The byte addresses below are relative to that base.
    HIP_DYNAMIC_SHARED(uint32_t, lds);
    const unsigned lane = threadIdx.x;
    Output &out = outputs[lane];

    lds[lane] = 0xa0000000U | lane;
    __syncthreads();
    out.returned[Independent] = exchange(lane * 4, 0xb0000000U | lane);
    __syncthreads();
    out.memory[Independent] = lds[lane];

    lds[lane] = 0x90000000U | lane;
    lds[65 + lane] = 0xc0000000U | lane;
    if (lane == 0) {
        lds[64] = 0xabcdef01;
    }
    __syncthreads();
    uint32_t offset_result;
    // 260 = 0x104 exercises both bytes of the instruction's byte offset.
    asm volatile("ds_wrxchg_rtn_b32 %0, %1, %2 offset:260\n\t"
                 "s_waitcnt lgkmcnt(0)\n\t"
                 : "=v"(offset_result)
                 : "v"(lane * 4), "v"(0xd0000000U | lane)
                 : "memory");
    out.returned[Offset] = offset_result;
    __syncthreads();
    out.memory[Offset] = lds[65 + lane];
    out.neighbor = lds[lane];
    out.gap = lds[64];

    lds[lane] = 0x10000000U | lane;
    __syncthreads();
    out.returned[Masked] =
        maskedExchange(lane * 4, 0x20000000U | lane, EvenLanes);
    __syncthreads();
    out.memory[Masked] = lds[lane];

    lds[lane] = 0x30000000U | lane;
    __syncthreads();
    out.returned[Empty] = maskedExchange(lane * 4, 0x40000000U | lane, 0);
    __syncthreads();
    out.memory[Empty] = lds[lane];

    if (lane == 0) {
        lds[0] = 0xffffffff;
    }
    __syncthreads();
    out.returned[Contended] = exchange(0, lane);
    __syncthreads();
    out.memory[Contended] = lds[0];
}

} // anonymous namespace

int
main()
{
    requireMi300x();
    Output outputs[NumLanes];
    Output *device_outputs;
    HIP_CHECK(hipMalloc(&device_outputs, sizeof(outputs)));
    ldsExchange<<<1, NumLanes, 129 * sizeof(uint32_t)>>>(device_outputs);
    HIP_CHECK(hipGetLastError());
    HIP_CHECK(hipDeviceSynchronize());
    HIP_CHECK(hipMemcpy(outputs, device_outputs, sizeof(outputs),
                        hipMemcpyDeviceToHost));
    HIP_CHECK(hipFree(device_outputs));

    unsigned checks = 0, errors = 0;
    for (unsigned lane = 0; lane < NumLanes; ++lane) {
        const auto &out = outputs[lane];
        expectBits("independent-return", lane, out.returned[Independent],
                   0xa0000000U | lane, checks, errors);
        expectBits("independent-memory", lane, out.memory[Independent],
                   0xb0000000U | lane, checks, errors);
        expectBits("offset-return", lane, out.returned[Offset],
                   0xc0000000U | lane, checks, errors);
        expectBits("offset-memory", lane, out.memory[Offset],
                   0xd0000000U | lane, checks, errors);
        expectBits("offset-neighbor", lane, out.neighbor, 0x90000000U | lane,
                   checks, errors);
        expectBits("offset-gap", lane, out.gap, 0xabcdef01, checks, errors);
        expectBits("masked-return", lane, out.returned[Masked],
                   lane % 2 ? Sentinel : 0x10000000U | lane, checks, errors);
        expectBits("masked-memory", lane, out.memory[Masked],
                   (lane % 2 ? 0x10000000U : 0x20000000U) | lane, checks,
                   errors);
        expectBits("empty-return", lane, out.returned[Empty], Sentinel, checks,
                   errors);
        expectBits("empty-memory", lane, out.memory[Empty], 0x30000000U | lane,
                   checks, errors);
        expectBits("contended-memory", lane, out.memory[Contended],
                   outputs[0].memory[Contended], checks, errors);
    }

    // Atomic exchanges form a chain. The returned old values and final
    // memory value must contain the initial value and each replacement
    // exactly once, irrespective of the order in which lanes execute.
    bool seen[NumLanes + 1] = {};
    auto mark = [&](uint32_t value, unsigned lane) {
        ++checks;
        const unsigned index = value == 0xffffffff
                                   ? NumLanes
                                   : (value < NumLanes ? value : NumLanes + 1);
        if (index > NumLanes || seen[index]) {
            if (errors++ < 20) {
                std::fprintf(stderr, "contended lane %u: invalid %08x\n", lane,
                             value);
            }
        } else {
            seen[index] = true;
        }
    };
    for (unsigned lane = 0; lane < NumLanes; ++lane) {
        mark(outputs[lane].returned[Contended], lane);
    }
    mark(outputs[0].memory[Contended], NumLanes);
    return finish("lds-exchange", checks, errors);
}
