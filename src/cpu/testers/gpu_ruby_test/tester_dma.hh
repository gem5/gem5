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
 */

/*
 * This is a fake DMA device to pass to Ruby.py so it will create the DMA
 * sequencers and controllers without being protocol specific. It otherwise
 * does nothing.
 */

#ifndef __CPU_TESTERS_GPU_RUBY_TEST_TESTER_DMA_HH__
#define __CPU_TESTERS_GPU_RUBY_TEST_TESTER_DMA_HH__

#include "dev/dma_device.hh"
#include "params/TesterDma.hh"

namespace gem5
{

class TesterDma : public DmaDevice
{
  public:
    typedef TesterDmaParams Params;

    TesterDma(const Params &p) : DmaDevice(p) {}

    virtual ~TesterDma() {}

    // The tester does not use a huge memory range. The range itself is
    // choosen arbitrarily
    AddrRangeList
    getAddrRanges() const override
    {
        AddrRangeList ranges;
        ranges.push_back(RangeSize(0, 0xc0000000));
        return ranges;
    }

    // These latencies are not important. Return any integer.
    Tick
    read(PacketPtr) override
    {
        return 10;
    }

    Tick
    write(PacketPtr) override
    {
        return 10;
    }
};

} // namespace gem5

#endif /* __CPU_TESTERS_GPU_RUBY_TEST_TESTER_DMA_HH__ */
