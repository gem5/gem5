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

#ifndef __DEV_AMDGPU_SDMA_PACKETS_HH__
#define __DEV_AMDGPU_SDMA_PACKETS_HH__

namespace gem5
{

/**
 * SDMA packets - see src/core/inc/sdma_registers.h in ROCR-Runtime
 */
typedef struct GEM5_PACKED
{
    uint32_t count : 30;
    uint32_t res0 : 2;
    uint32_t res1 : 16;
    uint32_t sdw : 2;
    uint32_t res2 : 6;
    uint32_t ddw : 2;
    uint32_t res3 : 6;
    uint64_t source;
    uint64_t dest;
} sdmaCopy;

static_assert(sizeof(sdmaCopy) == 24);

typedef struct GEM5_PACKED
{
    uint64_t dest;
    uint32_t count : 20;
    uint32_t reserved0 : 4;
    uint32_t sw : 2;
    uint32_t reserved1 : 6;
} sdmaWrite;

static_assert(sizeof(sdmaWrite) == 12);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t addrLo;
            uint32_t addrHi;
        };

        Addr addr;
    };

    uint32_t srcData;
    uint32_t unused : 10;
    uint32_t count : 22;
} sdmaConstFill;

static_assert(sizeof(sdmaConstFill) == 16);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t op : 8;
            uint32_t sub_op : 8;
            uint32_t sw : 2;
            uint32_t res0 : 12;
            uint32_t fillsize : 2;
        };

        uint32_t ordinal;
    };
} sdmaConstFillHeader;

static_assert(sizeof(sdmaConstFillHeader) == 4);

typedef struct GEM5_PACKED
{
    uint32_t key0;
    uint32_t key1;
    uint32_t key2;
    uint32_t key3;
    uint32_t count0;
    uint32_t count1;
    uint32_t count2;
    uint32_t count3;
} sdmaAESKey;

static_assert(sizeof(sdmaAESKey) == 32);

typedef struct GEM5_PACKED
{
    uint32_t countData0;
    uint32_t countData1;
    uint32_t countData2;
    uint32_t countData3;
} sdmaAESCounter;

static_assert(sizeof(sdmaAESCounter) == 16);

typedef struct GEM5_PACKED
{
    uint32_t countKey0;
    uint32_t countKey1;
    uint32_t countKey2;
    uint32_t countKey3;
} sdmaAESLoad;

static_assert(sizeof(sdmaAESLoad) == 16);

typedef struct GEM5_PACKED
{
    uint32_t reserved : 6;
    uint32_t offset : 26;
} sdmaAESOffset;

static_assert(sizeof(sdmaAESOffset) == 4);

typedef struct GEM5_PACKED
{
    uint64_t base;
    uint32_t size : 20;
    uint32_t reserved : 12;
    uint64_t csaAddr;
} sdmaIndirectBuffer;

static_assert(sizeof(sdmaIndirectBuffer) == 20);

typedef struct GEM5_PACKED
{
    uint32_t priv : 1;
    uint32_t reserved1 : 11;
    uint32_t vmid : 4;
    uint32_t reserved2 : 16;
} sdmaIndirectBufferHeader;

static_assert(sizeof(sdmaIndirectBufferHeader) == 4);

typedef struct GEM5_PACKED
{
    uint64_t dest;
    uint32_t data;
} sdmaFence;

static_assert(sizeof(sdmaFence) == 12);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t contextId : 3;
            uint32_t rbRptr : 13;
            uint32_t ibOffset : 12;
            uint32_t reserved : 4;
        };

        uint32_t intrContext;
    };
} sdmaTrap;

static_assert(sizeof(sdmaTrap) == 4);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t reserved : 3;
            uint32_t addrLo : 29;
            uint32_t addrHi;
        };

        Addr addr;
    };
} sdmaSemaphore;

static_assert(sizeof(sdmaSemaphore) == 8);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t reserved : 3;
            uint32_t addrLo : 29;
            uint32_t addrHi;
        };

        Addr addr;
    };
} sdmaMemInc;

static_assert(sizeof(sdmaMemInc) == 8);

typedef struct GEM5_PACKED
{
    uint32_t regAddr : 18;
    uint32_t reserved : 2;
    uint32_t apertureId : 12;
    uint32_t data;
} sdmaSRBMWrite;

static_assert(sizeof(sdmaSRBMWrite) == 8);

typedef struct GEM5_PACKED
{
    uint32_t reserved : 28;
    uint32_t byteEnable : 4;
} sdmaSRBMWriteHeader;

static_assert(sizeof(sdmaSRBMWriteHeader) == 4);

typedef struct GEM5_PACKED
{
    uint64_t address;
    uint32_t ref;
    uint32_t mask;
    uint32_t pollInt : 16;
    uint32_t retryCount : 12;
    uint32_t reserved1 : 4;
} sdmaPollRegMem;

static_assert(sizeof(sdmaPollRegMem) == 20);

typedef struct GEM5_PACKED
{
    uint32_t reserved : 26;
    uint32_t op : 2;   // Operation
    uint32_t func : 3; // Comparison function
    uint32_t mode : 1; // Mode: register or memory polling
} sdmaPollRegMemHeader;

static_assert(sizeof(sdmaPollRegMemHeader) == 4);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t addrLo;
            uint32_t addrHi;
        };

        Addr addr;
    };

    uint32_t reference;

    union
    {
        struct
        {
            uint32_t execCount : 14;
            uint32_t unused : 18;
        };

        uint32_t ordinal;
    };
} sdmaCondExec;

static_assert(sizeof(sdmaCondExec) == 16);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t addrLo;
            uint32_t addrHi;
        };

        Addr addr;
    };

    union
    {
        struct
        {
            uint32_t srcDataLo;
            uint32_t srdDataHi;
        };

        uint64_t srcData;
    };

    union
    {
        struct
        {
            uint32_t cmpDataLo;
            uint32_t cmpDataHi;
        };

        uint64_t cmpData;
    };

    uint32_t loopInt : 13;
    uint32_t reserved : 19;
} sdmaAtomic;

static_assert(sizeof(sdmaAtomic) == 28);

typedef struct GEM5_PACKED
{
    int unused2 : 16;
    int loop : 1;
    int unused1 : 8;
    int opcode : 7;
} sdmaAtomicHeader;

static_assert(sizeof(sdmaAtomicHeader) == 4);

constexpr unsigned int SDMA_ATOMIC_ADD64 = 47;

typedef struct GEM5_PACKED
{
    uint64_t dest;
    uint64_t mask;
    uint64_t initValue;
    uint64_t increment;
    uint32_t count : 19;
    uint32_t reserved : 13;
} sdmaPtePde;

static_assert(sizeof(sdmaPtePde) == 36);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t initDataLo;
            uint32_t initDataHi;
        };

        uint64_t initData;
    };
} sdmaTimestamp;

static_assert(sizeof(sdmaTimestamp) == 8);

typedef struct GEM5_PACKED
{
    uint32_t execCount : 14;
    uint32_t reserved : 18;
} sdmaPredExec;

static_assert(sizeof(sdmaPredExec) == 4);

typedef struct GEM5_PACKED
{
    uint32_t opcode : 8;
    uint32_t subOpcode : 8;
    uint32_t device : 8;
    uint32_t unused : 8;
} sdmaPredExecHeader;

static_assert(sizeof(sdmaPredExecHeader) == 4);

typedef struct GEM5_PACKED
{
    uint32_t contextId : 3;
    uint32_t rbRptr : 13;
    uint32_t ibOffset : 12;
    uint32_t reserved : 4;
} sdmaDummyTrap;

static_assert(sizeof(sdmaDummyTrap) == 4);

typedef struct GEM5_PACKED
{
    uint32_t byteStride;
    uint32_t dmaCount;

    union
    {
        struct
        {
            uint32_t destLo;
            uint32_t destHi;
        };

        uint64_t dest;
    };

    uint32_t byteCount : 26;
} sdmaDataFillMulti;

static_assert(sizeof(sdmaDataFillMulti) == 20);

typedef struct GEM5_PACKED
{
    uint16_t format : 8;
    uint16_t barrier : 1;
    uint16_t acqFenceScope : 2;
    uint16_t relFenceScope : 2;
    uint16_t reserved : 3;
} sdmaHeaderAgentDisp;

static_assert(sizeof(sdmaHeaderAgentDisp) == 2);

typedef struct GEM5_PACKED
{
    sdmaHeaderAgentDisp header;
    uint16_t res0;
    uint32_t res1;

    union
    {
        struct
        {
            uint32_t retLo;
            uint32_t retHi;
        };

        Addr ret;
    };

    uint32_t count : 22;
    uint32_t res2 : 10;
    uint32_t res3 : 16;
    uint32_t swDest : 2;
    uint32_t res4 : 6;
    uint32_t swSrc : 2;
    uint32_t unused : 6;

    union
    {
        struct
        {
            uint32_t srcLo;
            uint32_t srcHi;
        };

        Addr src;
    };

    union
    {
        struct
        {
            uint32_t destLo;
            uint32_t destHi;
        };

        Addr dest;
    };

    uint64_t res5;
    uint64_t res6;

    union
    {
        struct
        {
            uint32_t compSignalLo;
            uint32_t compSignalHi;
        };

        Addr compSignal;
    };
} sdmaAQLCopy;

static_assert(sizeof(sdmaAQLCopy) == 64);

typedef struct GEM5_PACKED
{
    sdmaHeaderAgentDisp header;
    uint16_t res0;
    uint32_t res1;
    Addr depSignal0;
    Addr depSignal1;
    Addr depSignal2;
    Addr depSignal3;
    Addr depSignal4;
    uint64_t res2;

    union
    {
        struct
        {
            uint32_t compSignalLo;
            uint32_t compSignalHi;
        };

        Addr compSignal;
    };
} sdmaAQLBarrierOr;

static_assert(sizeof(sdmaAQLBarrierOr) == 64);

} // namespace gem5

#endif // __DEV_AMDGPU_SDMA_PACKETS_HH__
