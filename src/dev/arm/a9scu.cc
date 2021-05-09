/*
 * Copyright (c) 2010,2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "dev/arm/a9scu.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

A9SCU::A9SCU(const Params &p)
    : BasicPioDevice(p, 0x60)
{
}

Tick
A9SCU::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;

    switch(daddr) {
      case Control:
        pkt->setLE(1); // SCU already enabled
        break;
      case Config:
        {
            /* Without making a completely new SCU, we can use the core count
             * field as 4 bits and inform the OS of up to 16 CPUs.  Although
             * the core count is technically bits [1:0] only, bits [3:2] are
             * SBZ for future expansion like this.
             */
            int threads = sys->threads.size();
            if (threads > 4) {
                warn_once("A9SCU with >4 CPUs is unsupported");
                fatal_if(threads > 15,
                        "Too many CPUs (%d) for A9SCU!", threads);
            }
            int smp_bits, core_cnt;
            smp_bits = (1 << threads) - 1;
            core_cnt = threads - 1;
            pkt->setLE(smp_bits << 4 | core_cnt);
        }
        break;
      default:
        // Only configuration register is implemented
        panic("Tried to read SCU at offset %#x\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;

}

Tick
A9SCU::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;
    switch (daddr) {
      default:
        // Nothing implemented at this point
        warn("Tried to write SCU at offset %#x\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

} // namespace gem5
