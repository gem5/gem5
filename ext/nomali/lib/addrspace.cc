/*
 * Copyright (c) 2016 ARM Limited
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

#include "jobslot.hh"

#include <cassert>
#include <cstdlib>

#include "addrspace.hh"
#include "gpu.hh"
#include "regutils.hh"

namespace NoMali {

const std::vector<AddrSpace::cmd_t> AddrSpace::cmds {
    &AddrSpace::cmdNop,                     // ASn_COMMAND_NOP
    &AddrSpace::cmdUpdate,                  // ASn_COMMAND_UPDATE
    &AddrSpace::cmdLock,                    // ASn_COMMAND_LOCK
    &AddrSpace::cmdUnlock,                  // ASn_COMMAND_UNLOCK
    &AddrSpace::cmdFlushPT,                 // ASn_COMMAND_FLUSH_PT
    &AddrSpace::cmdFlushMem,                // ASn_COMMAND_FLUSH_MEM
};

AddrSpace::AddrSpace(GPU &_gpu, MMU &_mmu, uint8_t _id)
    : GPUBlock(_gpu, ASn_NO_REGS),
      id(_id),
      mmu(_mmu)
{
}

AddrSpace::AddrSpace(AddrSpace &&rhs)
    : GPUBlock(std::move(rhs)),
      id(std::move(rhs.id)),
      mmu(rhs.mmu)
{
}

AddrSpace::~AddrSpace()
{
}

void
AddrSpace::writeReg(RegAddr addr, uint32_t value)
{
    switch (addr.value) {
      case ASn_COMMAND:
        asCommand(value);
        break;

      case ASn_TRANSTAB_LO:
      case ASn_TRANSTAB_HI:
      case ASn_MEMATTR_LO:
      case ASn_MEMATTR_HI:
      case ASn_LOCKADDR_LO:
      case ASn_LOCKADDR_HI:
        GPUBlock::writeReg(addr, value);
        break;

      default:
        // Ignore writes by default
        break;
    };
}

void
AddrSpace::asCommand(uint32_t cmd)
{
    if (cmd < cmds.size())
        (this->*cmds[cmd])(cmd);
}

void
AddrSpace::cmdNop(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_NOP);
}


void
AddrSpace::cmdUpdate(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_UPDATE);
}

void
AddrSpace::cmdLock(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_LOCK);
}

void
AddrSpace::cmdUnlock(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_UNLOCK);
}

void
AddrSpace::cmdFlushPT(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_FLUSH_PT);
}

void
AddrSpace::cmdFlushMem(uint32_t cmd)
{
    assert(cmd == ASn_COMMAND_FLUSH_MEM);
}

}
