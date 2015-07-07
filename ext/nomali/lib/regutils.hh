/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#ifndef _LIBNOMALIMODEL_REGUTILS_HH
#define _LIBNOMALIMODEL_REGUTILS_HH

#include <cassert>

#include "types.hh"
#include "mali_midg_regmap.h"

namespace NoMali {

/** Size of a function block in bytes */
static const uint32_t BLOCK_REGS_SIZE(0x1000);
/** Number of registers in a function block */
static const uint32_t BLOCK_NUM_REGS = BLOCK_REGS_SIZE >> 2;

/**
 * Register blocks within the GPU.
 *
 * The GPU splits its register space into chunks belonging to specific
 * blocks. This enum lists those blocks.
 */
enum class RegBlock : uint16_t {
    GPU = 0x0,
    JOB = 0x1,
    MMU = 0x2,

    UNKNOWN = 0xFFFF,
};

/** Get the register block from a GPU register address */
static inline RegBlock
getRegBlock(RegAddr addr)
{
    return RegBlock(addr.value >> 12);
}

/**
 * Get the register address within a GPU block.
 *
 * This method masks away the block offset from a GPU register
 * address. The resulting address is the address <i>within</i> a
 * function block.
 */
static inline RegAddr
getBlockReg(RegAddr addr)
{
    static_assert((BLOCK_REGS_SIZE & (BLOCK_REGS_SIZE - 1)) == 0,
                  "BLOCK_REGS_SIZE is not a power of 2");
    return RegAddr(addr.value & (BLOCK_REGS_SIZE - 1));
}

/**
 * Get the slot number owning an address within the JobControl block.
 *
 * @param Address relative to the JobControl block.
 */
static inline unsigned
getJobSlotNo(const RegAddr &addr)
{
    assert(addr.value >= JOB_SLOT0);
    assert(addr.value <= 0xFFF);
    return (addr.value - JOB_SLOT0) >> 7;
}

/**
 * Get a JobSlot-relative address from a JobControl-relative address.
 *
 * @param Address relative to the JobControl block.
 * @return Address relative the start of the JobSlot.
 */
static inline RegAddr
getJobSlotAddr(const RegAddr &addr)
{
    const unsigned slot_no(getJobSlotNo(addr));
    const RegAddr slot_base(RegAddr(JOB_SLOT0 + slot_no * 0x80));
    return addr - slot_base;
}

/** Number of registers per job slot */
static const unsigned JSn_NO_REGS = 0x20;

}

#endif //_LIBNOMALIMODEL_REGUTILS_HH
