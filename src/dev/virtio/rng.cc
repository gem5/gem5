/*
 * Copyright (c) 2022  Institute of Computing Technology, Chinese Academy
 *                     of Sciences
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

#include "dev/virtio/rng.hh"

#include "base/random.hh"
#include "debug/VIORng.hh"
#include "params/VirtIORng.hh"
#include "sim/system.hh"

namespace gem5
{

VirtIORng::VirtIORng(const Params &params)
    : VirtIODeviceBase(params, ID_RNG, 0, 0),
      qReq(params.system->physProxy, byteOrder, params.qSize, *this)
{
    registerQueue(qReq);
}

VirtIORng::~VirtIORng() {}

VirtIORng::RngQueue::RngQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                              VirtIORng &_parent)
    : VirtQueue(proxy, bo, size), parent(_parent)
{}

void
VirtIORng::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    // There are no configuration for RNG device
    pkt->makeResponse();
}

void
VirtIORng::RngQueue::trySend()
{
    DPRINTF(VIORng, "try send\n");

    VirtDescriptor *d;
    while ((d = consumeDescriptor())) {
        DPRINTF(VIORng, "Got descriptor (len: %i)\n", d->size());
        size_t len = 0;
        while (len < d->size()) {
            uint8_t byte = gem5::random_mt.random<uint8_t>();
            d->chainWrite(len, &byte, sizeof(uint8_t));
            ++len;
        }

        // Tell the guest that we are done with this descriptor.
        produceDescriptor(d, len);
        parent.kick();
    }
}

} // namespace gem5
