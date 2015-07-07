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
 */

#include "gpu.hh"

#include "gpucontrol.hh"
#include "jobcontrol.hh"
#include "mmu.hh"
#include "regutils.hh"

namespace NoMali {

GPU::GPU(GPUControl &gc, JobControl &jc, MMU &_mmu)
    : gpuControl(gc), jobControl(jc), mmu(_mmu),
      blocks({&gpuControl, // REG_BLOCK_GPU
              &jobControl, // REG_BLOCK_JOB
              &mmu})       // REG_BLOCK_MMU
{
}

GPU::~GPU()
{
}

void
GPU::reset()
{
    for (auto *block : blocks)
        block->reset();
}

uint32_t
GPU::readReg(RegAddr addr)
{
    GPUBlock * const block(getGPUBlock(addr));

    return block ? block->readReg(getBlockReg(addr)) : 0;
}

void
GPU::writeReg(RegAddr addr, uint32_t value)
{
    GPUBlock * const block(getGPUBlock(addr));

    if (block)
        block->writeReg(getBlockReg(addr), value);
}

uint32_t
GPU::readRegRaw(RegAddr addr)
{
    GPUBlock * const block(getGPUBlock(addr));

    return block ? block->readRegRaw(getBlockReg(addr)) : 0;
}

void
GPU::writeRegRaw(RegAddr addr, uint32_t value)
{
    GPUBlock * const block(getGPUBlock(addr));

    if (block)
        block->writeRegRaw(getBlockReg(addr), value);
}


bool
GPU::intGPUAsserted() const
{
    return gpuControl.intAsserted();
}

bool
GPU::intJobAsserted() const
{
    return jobControl.intAsserted();
}

bool
GPU::intMMUAsserted() const
{
    return mmu.intAsserted();
}


GPUBlock *
GPU::getGPUBlock(RegAddr addr)
{
    const RegBlock block(getRegBlock(addr));
    const uint16_t block_no(static_cast<uint16_t>(block));

    if (block_no < blocks.size())
        return blocks[block_no];
    else
        return nullptr;
}

}
