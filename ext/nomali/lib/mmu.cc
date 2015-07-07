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

#include "mmu.hh"

#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

MMU::MMU(GPU &_gpu)
    : GPUBlockInt(_gpu,
                  RegAddr(MMU_IRQ_RAWSTAT),
                  RegAddr(MMU_IRQ_CLEAR),
                  RegAddr(MMU_IRQ_MASK),
                  RegAddr(MMU_IRQ_STATUS)),
      regs(BLOCK_NUM_REGS)
{
}

MMU::~MMU()
{
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
        // Ignore writes by default
        break;
    };
}

void
MMU::onInterrupt(int set)
{
    gpu.intMMU(set);
}

}
