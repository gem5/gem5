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

#ifndef __DEV_LUPIO_LUPIO_RNG_HH__
#define __DEV_LUPIO_LUPIO_RNG_HH__

#include <random>

#include "debug/LupioRNG.hh"
#include "dev/io_device.hh"
#include "params/LupioRNG.hh"
#include "sim/system.hh"

namespace gem5
{

/**
 * LupioRNG:
 * A Random Number Generator virtual device that returns either a random
 * value, or a seed that can be configured by the user.
 */
class LupioRNG : public BasicPioDevice
{
  protected:
    const ByteOrder byteOrder = ByteOrder::little;

    // Register map
  private:
    enum
    {
        LUPIO_RNG_RAND,
        LUPIO_RNG_SEED,
        LUPIO_RNG_CTRL,
        LUPIO_RNG_STAT,
    };

    std::mt19937 mt;
    uint64_t user_seed;
    bool interrupt_enable = false;

    /**
     * Function to return a random number, the current seed, or the device
     * status
     */
    uint64_t lupioRNGRead(const uint8_t addr);
    /**
     * Function to allow the user to change the current seed, and configure
     * the device using interrupts
     */
    void lupioRNGWrite(const uint8_t addr, const uint64_t val64);

  public:
    PARAMS(LupioRNG);
    LupioRNG(const Params &params);

    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_LUPIO_LUPIO_RNG_HH__
