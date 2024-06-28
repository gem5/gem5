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

#ifndef __DEV_AMDGPU_PM4_DEFINES_H__
#define __DEV_AMDGPU_PM4_DEFINES_H__

#include <cstdlib>
#include <iostream>
#include <vector>

#include "base/types.hh"

namespace gem5
{

/**
 * PM4 opcodes. Taken from linux tree from the following locations:
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/amdkfd/kfd_pm4_opcodes.h
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/amdgpu/soc15d.h
 */
enum it_opcode_type
{
    IT_NOP                               = 0x10,
    IT_WRITE_DATA                        = 0x37,
    IT_WAIT_REG_MEM                      = 0x3C,
    IT_INDIRECT_BUFFER                   = 0x3F,
    IT_RELEASE_MEM                       = 0x49,
    IT_SET_UCONFIG_REG                   = 0x79,
    IT_SWITCH_BUFFER                     = 0x8B,
    IT_INVALIDATE_TLBS                   = 0x98,
    IT_MAP_PROCESS                       = 0xA1,
    IT_MAP_QUEUES                        = 0xA2,
    IT_UNMAP_QUEUES                      = 0xA3,
    IT_QUERY_STATUS                      = 0xA4,
    IT_RUN_LIST                          = 0xA5,
};

/**
 * Value from vega10/pm4_header.h.
 */
#define PACKET3_SET_UCONFIG_REG_START                   0x0000c000

/**
 * PM4 packets
 */
typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint16_t predicated : 1;
            uint16_t shader : 1;
            uint16_t reserved : 6;
            uint16_t opcode : 8;
            uint16_t count : 14;
            uint16_t type : 2;
        };
        uint32_t ordinal;
    };
} PM4Header;
static_assert(sizeof(PM4Header) == 4);

typedef struct GEM5_PACKED
{
    uint32_t reserved1 : 8;
    uint32_t destSel : 4;
    uint32_t reserved2 : 4;
    uint32_t addrIncr : 1;
    uint32_t reserved3 : 2;
    uint32_t resume : 1;
    uint32_t writeConfirm : 1;
    uint32_t reserved4 : 4;
    uint32_t cachePolicy : 2;
    uint32_t reserved5 : 5;
    union
    {
        struct
        {
            uint32_t destAddrLo;
            uint32_t destAddrHi;
        };
        uint64_t destAddr;
    };
    uint32_t data;
}  PM4WriteData;
static_assert(sizeof(PM4WriteData) == 16);

typedef struct GEM5_PACKED
{
    uint32_t reserved1 : 4;
    uint32_t queueSel : 2;
    uint32_t reserved2 : 2;
    uint32_t vmid : 4;
    uint32_t reserved3 : 1;
    uint32_t queueSlot : 3;
    uint32_t pipe : 2;
    uint32_t me : 1;
    uint32_t reserved6 : 2;
    uint32_t queueType : 3;
    uint32_t allocFormat : 2;
    uint32_t engineSel : 3;
    uint32_t numQueues : 3;
    uint32_t reserved4 : 1;
    uint32_t checkDisable : 1;
    uint32_t doorbellOffset : 26;
    uint32_t reserved5 : 4;
    union
    {
        struct
        {
            uint32_t mqdAddrLo : 32;
            uint32_t mqdAddrHi : 32;
        };
        uint64_t mqdAddr;
    };
    union
    {
        struct
        {
            uint32_t wptrAddrLo : 32;
            uint32_t wptrAddrHi : 32;
        };
        uint64_t wptrAddr;
    };
}  PM4MapQueues;
static_assert(sizeof(PM4MapQueues) == 24);

typedef struct GEM5_PACKED
{
    uint32_t action : 2;
    uint32_t reserved : 2;
    uint32_t queueSel : 2;
    uint32_t reserved1 : 20;
    uint32_t engineSel : 3;
    uint32_t numQueues : 3;
    union
    {
        struct
        {
            uint32_t pasid : 16;
            uint32_t reserved2 : 16;
        };
        struct
        {
            uint32_t reserved3 : 2;
            uint32_t doorbellOffset0 : 26;
            uint32_t reserved4 : 4;
        };
    };
    uint32_t reserved5 : 2;
    uint32_t doorbellOffset1 : 26;
    uint32_t reserved6 : 4;
    uint32_t reserved7 : 2;
    uint32_t doorbellOffset2 : 26;
    uint32_t reserved8 : 4;
    uint32_t reserved9 : 2;
    uint32_t doorbellOffset3 : 26;
    uint32_t reserved10 : 4;
}  PM4UnmapQueues;
static_assert(sizeof(PM4UnmapQueues) == 20);

typedef struct GEM5_PACKED
{
    uint32_t vmidMask : 16;
    uint32_t unmapLatency : 8;
    uint32_t reserved : 5;
    uint32_t queueType : 3;
    union
    {
        struct
        {
            uint32_t queueMaskLo;
            uint32_t queueMaskHi;
        };
        uint64_t queueMask;
    };
    union
    {
        struct
        {
            uint32_t gwsMaskLo;
            uint32_t gwsMaskHi;
        };
        uint64_t gwsMask;
    };
    uint16_t oacMask;
    uint16_t reserved1;
    uint32_t gdsHeapBase : 6;
    uint32_t reserved2 : 5;
    uint32_t gdsHeapSize : 6;
    uint32_t reserved3 : 15;
}  PM4SetResources;
static_assert(sizeof(PM4SetResources) == 28);

typedef struct GEM5_PACKED
{
    uint32_t pasid : 16;
    uint32_t reserved0 : 8;
    uint32_t diq : 1;
    uint32_t processQuantum : 7;
    union
    {
        struct
        {
            uint32_t ptBaseLo;
            uint32_t ptBaseHi;
        };
        uint64_t ptBase;
    };
    uint32_t shMemBases;
    uint32_t shMemConfig;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
    uint32_t reserved5;
    union
    {
        struct
        {
            uint32_t gdsAddrLo;
            uint32_t gdsAddrHi;
        };
        uint64_t gdsAddr;
    };
    uint32_t numGws : 6;
    uint32_t reserved7 : 2;
    uint32_t numOac : 4;
    uint32_t reserved8 : 4;
    uint32_t gdsSize : 6;
    uint32_t numQueues : 10;
    union
    {
        struct
        {
            uint32_t completionSignalLo;
            uint32_t completionSignalHi;
        };
        uint64_t completionSignal;
    };
}  PM4MapProcess;
static_assert(sizeof(PM4MapProcess) == 60);

typedef struct GEM5_PACKED
{
    uint32_t pasid : 16;
    uint32_t reserved0 : 8;
    uint32_t diq : 1;
    uint32_t processQuantum : 7;
    union
    {
        struct
        {
            uint32_t ptBaseLo;
            uint32_t ptBaseHi;
        };
        uint64_t ptBase;
    };
    uint32_t shMemBases;
    uint32_t shMemConfig;
    uint32_t sqShaderTbaLo;
    uint32_t sqShaderTbaHi;
    uint32_t sqShaderTmaLo;
    uint32_t sqShaderTmaHi;
    uint32_t reserved1;
    union
    {
        struct
        {
            uint32_t gdsAddrLo;
            uint32_t gdsAddrHi;
        };
        uint64_t gdsAddr;
    };
    union
    {
        struct
        {
            uint32_t numGws : 7;
            uint32_t sdma_enable : 1;
            uint32_t numOac : 4;
            uint32_t reserved3 : 4;
            uint32_t gdsSize : 6;
            uint32_t numQueues : 10;
        };
        uint32_t ordinal14;
    };
    uint32_t spiGdbgPerVmidCntl;
    uint32_t tcpWatchCntl[4];
    union
    {
        struct
        {
            uint32_t completionSignalLo;
            uint32_t completionSignalHi;
        };
        uint64_t completionSignal;
    };
}  PM4MapProcessV2;
static_assert(sizeof(PM4MapProcessV2) == 80);

typedef struct GEM5_PACKED
{
    uint32_t function : 4;
    uint32_t memSpace : 2;
    uint32_t operation : 2;
    uint32_t reserved1 : 24;
    union
    {
        struct
        {
            uint32_t regAddr1 : 18;
            uint32_t reserved2 : 14;
        };
        uint32_t memAddrLo;
    };
    union
    {
        struct
        {
            uint32_t regAddr2 : 18;
            uint32_t reserved3 : 14;
        };
        uint32_t memAddrHi;
    };
    uint32_t reference;
    uint32_t mask;
    uint32_t pollInterval;
}  PM4WaitRegMem;
static_assert(sizeof(PM4WaitRegMem) == 24);

typedef struct GEM5_PACKED
{
    uint32_t regOffset : 16;
    uint32_t reserved : 16;
    uint32_t regData;
}  PM4SetUConfig;
static_assert(sizeof(PM4SetUConfig) == 8);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t ibBaseLo;
            uint32_t ibBaseHi;
        };
        uint64_t ibBase;
    };
    uint32_t ibSize : 20;
    uint32_t chain : 1;
    uint32_t poll : 1;
    uint32_t reserved0 : 1;
    uint32_t valid: 1;
    uint32_t vmid : 4;
    uint32_t cachePolicy : 2;
    uint32_t reserved1 : 1;
    uint32_t priv : 1;
}  PM4IndirectBuf;
static_assert(sizeof(PM4IndirectBuf) == 12);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t tmz : 1;
            uint32_t reserved : 31;
        };
        uint32_t dummy;
    };
}  PM4SwitchBuf;
static_assert(sizeof(PM4SwitchBuf) == 4);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t ibBaseLo;
            uint32_t ibBaseHi;
        };
        uint64_t ibBase;
    };
    uint32_t ibSize : 20;
    uint32_t chain : 1;
    uint32_t ena : 1;
    uint32_t reserved1 : 2;
    uint32_t vmid : 4;
    uint32_t cachePolicy : 2;
    uint32_t preResume : 1;
    uint32_t priv : 1;
}  PM4IndirectBufConst;
static_assert(sizeof(PM4IndirectBufConst) == 12);

typedef struct GEM5_PACKED
{
    uint32_t tmz : 1;
    uint32_t reserved : 27;
    uint32_t command : 4;
}  PM4FrameCtrl;
static_assert(sizeof(PM4FrameCtrl) == 4);

typedef struct GEM5_PACKED
{
    uint32_t event : 6;
    uint32_t reserved0 : 2;
    uint32_t eventIdx : 4;
    uint32_t l1Volatile : 1;
    uint32_t l2Volatile : 1;
    uint32_t reserved1 : 1;
    uint32_t l2WB : 1;
    uint32_t l1Inv : 1;
    uint32_t l2Inv : 1;
    uint32_t reserved2 : 1;
    uint32_t l2NC : 1;
    uint32_t l2WC : 1;
    uint32_t l2Meta : 1;
    uint32_t reserved3 : 3;
    uint32_t cachePolicy : 2;
    uint32_t reserved4 : 1;
    uint32_t execute : 1;
    uint32_t reserved5 : 3;
    uint32_t reserved6 : 16;
    uint32_t destSelect : 2;
    uint32_t reserved7 : 6;
    uint32_t intSelect : 3;
    uint32_t reserved8 : 2;
    uint32_t dataSelect : 3;
    union
    {
        struct
        {
            uint32_t addrLo;
            uint32_t addrHi;
        };
        uint64_t addr;
    };
    union
    {
        struct
        {
            union
            {
                struct
                {
                    uint32_t dwOffset : 16;
                    uint32_t numDws : 16;
                };
                uint32_t dataLo : 32;
            };
            uint32_t dataHi;
        };
        uint64_t data;
    };
    uint32_t intCtxId;
}  PM4ReleaseMem;
static_assert(sizeof(PM4ReleaseMem) == 28);

typedef struct GEM5_PACKED
{
    uint32_t offset : 16;
    uint32_t reserved : 16;
    uint32_t data;
}  PM4SetUconfigReg;
static_assert(sizeof(PM4SetUconfigReg) == 8);

typedef struct GEM5_PACKED
{
    union
    {
        struct
        {
            uint32_t ibBaseLo;
            uint32_t ibBaseHi;
        };
        uint64_t ibBase;
    };
    uint32_t ibSize : 20;
    uint32_t chain : 1;
    uint32_t offleadPolling : 1;
    uint32_t reserved1 : 1;
    uint32_t valid : 1;
    uint32_t processCnt : 4;
    uint32_t reserved2 : 4;
}  PM4RunList;
static_assert(sizeof(PM4RunList) == 12);

typedef struct GEM5_PACKED
{
    uint32_t contextId : 28;
    uint32_t interruptSel : 2;
    uint32_t command : 2;
    union
    {
        struct
        {
            uint32_t pasid : 16;
            uint32_t reserved0 : 16;
        };
        struct
        {
            uint32_t reserved1 : 2;
            uint32_t doorbellOffset : 26;
            uint32_t engineSel : 3;
            uint32_t reserved2 : 1;
        };
    };
    union
    {
        struct
        {
            uint32_t addrLo;
            uint32_t addrHi;
        };
        uint64_t addr;
    };
    union
    {
        struct
        {
            uint32_t dataLo;
            uint32_t dataHi;
        };
        uint64_t data;
    };
}  PM4QueryStatus;
static_assert(sizeof(PM4QueryStatus) == 24);

} // namespace gem5

#endif // __DEV_AMDGPU_PM4_DEFINES_HH__
