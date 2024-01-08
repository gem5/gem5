/*
 * Copyright (c) 2012 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Brad Beckmann
 *
 */

#include "mem/ruby/structures/BankedArray.hh"

#include "base/intmath.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace ruby
{

BankedArray::BankedArray(unsigned int banks, Cycles accessLatency,
                         unsigned int startIndexBit, RubySystem *rs)
    : m_ruby_system(rs)
{
    this->banks = banks;
    this->accessLatency = accessLatency;
    this->startIndexBit = startIndexBit;

    if (banks != 0) {
        bankBits = floorLog2(banks);
    }

    busyBanks.resize(banks);
}

bool
BankedArray::tryAccess(int64_t idx)
{
    if (accessLatency == 0)
        return true;

    unsigned int bank = mapIndexToBank(idx);
    assert(bank < banks);

    if (busyBanks[bank].endAccess >= curTick()) {
            return false;
    }

    return true;
}

void
BankedArray::reserve(int64_t idx)
{
    if (accessLatency == 0)
        return;

    unsigned int bank = mapIndexToBank(idx);
    assert(bank < banks);

    if (busyBanks[bank].endAccess >= curTick()) {
        if (busyBanks[bank].startAccess == curTick() &&
             busyBanks[bank].idx == idx) {
            // this is the same reservation (can happen when
            // e.g., reserve the same resource for read and write)
            return; // OK
        } else {
            panic("BankedArray reservation error");
        }
    }

    busyBanks[bank].idx = idx;
    busyBanks[bank].startAccess = curTick();
    busyBanks[bank].endAccess = curTick() +
        (accessLatency-1) * m_ruby_system->clockPeriod();
}

unsigned int
BankedArray::mapIndexToBank(int64_t idx)
{
    if (banks == 1) {
        return 0;
    }
    return idx % banks;
}

} // namespace ruby
} // namespace gem5
