/*
 * Copyright (c) 2010 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#include "dev/arm/amba_device.hh"

#include "base/trace.hh"
#include "debug/AMBA.hh"
#include "dev/arm/amba_fake.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

const uint64_t AmbaVendor = ULL(0xb105f00d00000000);

AmbaPioDevice::AmbaPioDevice(const Params *p, Addr pio_size)
    : BasicPioDevice(p, pio_size), ambaId(AmbaVendor | p->amba_id)
{
}

AmbaIntDevice::AmbaIntDevice(const Params *p, Addr pio_size)
    : AmbaPioDevice(p, pio_size),
      intNum(p->int_num), gic(p->gic), intDelay(p->int_delay)
{
}



AmbaDmaDevice::AmbaDmaDevice(const Params *p, Addr pio_size)
    : DmaDevice(p), ambaId(AmbaVendor | p->amba_id),
      pioAddr(p->pio_addr), pioSize(pio_size),
      pioDelay(p->pio_latency),intNum(p->int_num), gic(p->gic)
{
}

bool
AmbaDevice::readId(PacketPtr pkt, uint64_t amba_id, Addr pio_addr)
{
    Addr daddr = pkt->getAddr() - pio_addr;
    if (daddr < AMBA_PER_ID0 || daddr > AMBA_CEL_ID3)
        return false;

    int byte = (daddr - AMBA_PER_ID0) << 1;
    // Too noisy right now
    DPRINTF(AMBA, "Returning %#x for offset %#x(%d)\n",
            (amba_id >> byte) & 0xFF,
            pkt->getAddr() - pio_addr, byte);
    assert(pkt->getSize() == 4);
    pkt->setLE<uint32_t>((amba_id >> byte) & 0xFF);
    return true;
}
