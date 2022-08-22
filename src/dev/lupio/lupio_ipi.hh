/*
 * Copyright (c) 2021 The Regents of the University of California
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

#ifndef __DEV_LUPIO_LUPIO_IPI_HH__
#define __DEV_LUPIO_LUPIO_IPI_HH__

#include "arch/riscv/interrupts.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/LupioIPI.hh"

namespace gem5
{

/**
 * LupioIPI:
 * An inter-processor interrupt virtual device
 */

class LupioIPI : public BasicPioDevice
{
  private:
    const ByteOrder byteOrder = ByteOrder::little;
    System *system;
    int intType;

    // Register map
    enum
    {
        LUPIO_IPI_MASK = 0x0,
        LUPIO_IPI_PEND = 0x4,

        // Max offset
        LUPIO_IPI_MAX = 0x8,
    };

    uint32_t nThread;
    /**
     * Set of registers corresponding to each CPU for sending
     * inter-processor interrupts
     */
    std::vector<uint32_t> mask;
    std::vector<uint32_t> pending;

    /**
     * Function to return the value in the word register of the corresponding
     * CPU and lower the IRQ
     */
    uint64_t lupioIPIRead(const uint8_t addr, int size);
    /**
     * Function to write to the word register of the corresponding CPU and
     * raise the IRQ
     */
    void lupioIPIWrite(const uint8_t addr, uint64_t val64, int size);
    /**
     * Function to post and clear interrupts
     **/
    void lupioIPIUpdateIRQ();

  public:
    PARAMS(LupioIPI);
    LupioIPI(const Params &params);

    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_LUPIO_LUPIO_IPI_HH
