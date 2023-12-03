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

#include "dev/lupio/lupio_rng.hh"

#include <random>

#include "debug/LupioRNG.hh"
#include "mem/packet_access.hh"
#include "params/LupioRNG.hh"

/* CTRL fields */
#define LUPIO_RNG_IRQE 0x1
/* STAT fields */
#define LUPIO_RNG_BUSY 0x1

namespace gem5
{

LupioRNG::LupioRNG(const Params &params)
    : BasicPioDevice(params, params.pio_size), user_seed(params.seed)
{
    mt.seed(user_seed);
    DPRINTF(LupioRNG, "LupioRNG initalized with seed: %d\n", user_seed);
}

uint64_t
LupioRNG::lupioRNGRead(uint8_t addr)
{
    uint32_t r = 0;
    std::uniform_int_distribution<uint32_t> distrib(0);

    switch (addr >> 2) {
    // returns a random number
    case LUPIO_RNG_RAND:
        r = distrib(mt);
        DPRINTF(LupioRNG, "Random number: %#x\n", r);
        break;
    // returns the current seed being used
    case LUPIO_RNG_SEED:
        r = user_seed;
        DPRINTF(LupioRNG, "RNG_SEED Call: %d\n", r);
        break;
    // returns status of the device
    case LUPIO_RNG_STAT:
        r = 0; /* Always ready */
        DPRINTF(LupioRNG, "RNG_STAT Call: %d\n", r);
        break;
    default:
        r = -1;
        panic("Unexpected read to the LupioRTC device at address %d!", addr);
        break;
    }
    return r;
}

void
LupioRNG::lupioRNGWrite(uint8_t addr, uint64_t val64)
{
    switch (addr >> 2) {
    // allows the user to configure the seed value
    case LUPIO_RNG_SEED:
        user_seed = val64;
        mt.seed(user_seed);
        DPRINTF(LupioRNG, "RNG_SEED_WRITE: %d\n", user_seed);
        break;
    // configures the device using interrupts
    case LUPIO_RNG_CTRL:
        interrupt_enable = !!(val64 & LUPIO_RNG_IRQE);
        DPRINTF(LupioRNG, "RNG_CTRL Call: %d\n", interrupt_enable);
        break;
    default:
        panic("Unexpected write to the LupioRTC device at address %d!", addr);
        break;
    }
}

Tick
LupioRNG::read(PacketPtr pkt)
{
    Addr rng_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioRNG, "Read request - addr: %#x, size: %#x\n", rng_addr,
            pkt->getSize());

    uint64_t rand_read = lupioRNGRead(rng_addr);
    DPRINTF(LupioRNG, "Packet Read: %#x\n", rand_read);
    pkt->setUintX(rand_read, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioRNG::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioRNG, "Write register %#x value %#x\n", daddr,
            pkt->getUintX(byteOrder));

    lupioRNGWrite(daddr, pkt->getUintX(byteOrder));
    DPRINTF(LupioRNG, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5
