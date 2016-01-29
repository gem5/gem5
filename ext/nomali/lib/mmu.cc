/*
 * Copyright (c) 2014-2016 ARM Limited
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

#include "mmu.hh"

#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

MMU::MMU(GPU &_gpu)
    : GPUBlockInt(_gpu,
                  RegAddr(MMU_IRQ_RAWSTAT),
                  RegAddr(MMU_IRQ_CLEAR),
                  RegAddr(MMU_IRQ_MASK),
                  RegAddr(MMU_IRQ_STATUS))
{
    spaces.reserve(16);
    for (int i = 0; i < 16; ++i)
        spaces.emplace_back(_gpu, *this, i);
}

MMU::~MMU()
{
}

void
MMU::reset()
{
    GPUBlockInt::reset();

    for (auto &as : spaces)
        as.reset();
}

uint32_t
MMU::readReg(RegAddr addr)
{
    if (isAddrSpaceReg(addr)) {
        return spaces[getAddrSpaceNo(addr)].readReg(getAddrSpaceAddr(addr));
    } else {
        return GPUBlockInt::readReg(addr);
    }
}

void
MMU::writeReg(RegAddr addr, uint32_t value)
{
    switch (addr.value) {
      case MMU_IRQ_RAWSTAT:
      case MMU_IRQ_CLEAR:
      case MMU_IRQ_MASK:
      case MMU_IRQ_STATUS:
        GPUBlockInt::writeReg(addr, value);
        break;

      default:
        if (isAddrSpaceReg(addr)) {
            AddrSpace &as(spaces[getAddrSpaceNo(addr)]);
            as.writeReg(getAddrSpaceAddr(addr), value);
        }
        break;
    };
}

uint32_t
MMU::readRegRaw(RegAddr addr)
{
    if (isAddrSpaceReg(addr)) {
        return spaces[getAddrSpaceNo(addr)].readRegRaw(getAddrSpaceAddr(addr));
    } else {
        return GPUBlockInt::readRegRaw(addr);
    }
}

void
MMU::writeRegRaw(RegAddr addr, uint32_t value)
{
    if (isAddrSpaceReg(addr)) {
        spaces[getAddrSpaceNo(addr)].writeRegRaw(getAddrSpaceAddr(addr), value);
    } else {
        GPUBlockInt::writeRegRaw(addr, value);
    }
}

void
MMU::onInterrupt(int set)
{
    gpu.intMMU(set);
}

}
