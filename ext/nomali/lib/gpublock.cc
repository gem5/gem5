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

#include "gpublock.hh"

#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

GPUBlock::GPUBlock(GPU &_gpu)
    : gpu(_gpu), regs(BLOCK_NUM_REGS)
{
}

GPUBlock::GPUBlock(GPU &_gpu, RegVector::size_type no_regs)
    : gpu(_gpu), regs(no_regs)
{
}

GPUBlock::GPUBlock(GPUBlock &&rhs)
    : gpu(rhs.gpu),
      regs(std::move(rhs.regs))
{
}

GPUBlock::~GPUBlock()
{
}

void
GPUBlock::reset()
{
    for (auto &r : regs)
        r = 0;
}

uint32_t
GPUBlock::readReg(RegAddr addr)
{
    return readRegRaw(addr);
}

void
GPUBlock::writeReg(RegAddr addr, uint32_t value)
{
    writeRegRaw(addr, value);
}

uint32_t
GPUBlock::readRegRaw(RegAddr addr)
{
    return regs[addr];
}

void
GPUBlock::writeRegRaw(RegAddr addr, uint32_t value)
{
    regs[addr] = value;
}



GPUBlockInt::GPUBlockInt(GPU &_gpu,
                         const RegAddr &irq_raw_stat,
                         const RegAddr &irq_clear,
                         const RegAddr &irq_mask,
                         const RegAddr &irq_stat)
    : GPUBlock(_gpu),
      addrIrqRawStat(irq_raw_stat), addrIrqClear(irq_clear),
      addrIrqMask(irq_mask), addrIrqStat(irq_stat)
{
}

GPUBlockInt::~GPUBlockInt()
{
}

uint32_t
GPUBlockInt::readReg(RegAddr addr)
{
    if (addr == addrIrqStat) {
        return irqStatus();
    } else {
        return GPUBlock::readReg(addr);
    }
}

void
GPUBlockInt::writeReg(RegAddr addr, uint32_t value)
{
    if (addr == addrIrqRawStat) {
        raiseInterrupt(value);
    } else if (addr == addrIrqClear) {
        clearInterrupt(value);
    } else if (addr == addrIrqMask ) {
        const bool old_int(intAsserted());
        GPUBlock::writeReg(addr, value);
        if (old_int != intAsserted())
            onInterrupt(intAsserted());
    } else if (addr == addrIrqStat ) {
        // Ignore writes to the IRQ status register
    } else {
        // Handle addrIrqMask & defaults
        GPUBlock::writeReg(addr, value);
    }
}



void
GPUBlockInt::raiseInterrupt(uint32_t ints)
{
    const bool old_int(intAsserted());

    regs[addrIrqRawStat] |= ints;
    // Is the interrupt line going high?
    if (!old_int && intAsserted())
        onInterrupt(1);
}

void
GPUBlockInt::clearInterrupt(uint32_t ints)
{
    const bool old_int(intAsserted());

    regs[addrIrqRawStat] &= ~ints;
    // Is the interrupt line going low?
    if (old_int && !intAsserted())
        onInterrupt(0);
}

uint32_t
GPUBlockInt::irqStatus() const
{
    return regs[addrIrqRawStat] & regs[addrIrqMask];
}


}
