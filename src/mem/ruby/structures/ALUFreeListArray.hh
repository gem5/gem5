/*
 * Copyright (c) 2023 The University of Wisconsin
 *
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
 */

#ifndef __MEM_RUBY_STRUCTURES_ALUFREELISTARRAY_HH__
#define __MEM_RUBY_STRUCTURES_ALUFREELISTARRAY_HH__

#include <deque>

#include "base/intmath.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace ruby
{

class ALUFreeListArray
{
  private:
    unsigned int numALUs;
    Cycles accessClocks;
    Tick accessLatency = 0;

    class AccessRecord
    {
      public:
        AccessRecord(Addr line_addr, Tick start_tick) {
          this->lineAddr = line_addr;
          this->startTick = start_tick;
        }

        Addr lineAddr;
        Tick startTick;
    };

    // Queue of accesses from past accessLatency cycles
    std::deque<AccessRecord> accessQueue;

    int m_block_size_bits = 0;

  public:
    ALUFreeListArray(unsigned int num_ALUs, Cycles access_clocks);

    bool tryAccess(Addr addr);

    void reserve(Addr addr);

    Tick
    getLatency() const
    {
        assert(accessLatency > 0);
        return accessLatency;
    }

    void
    setClockPeriod(Tick clockPeriod)
    {
        accessLatency = accessClocks * clockPeriod;
    }

    void
    setBlockSize(int block_size)
    {
        m_block_size_bits = floorLog2(block_size);
    }
};

} // namespace ruby
} // namespace gem5

#endif
