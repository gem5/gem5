/*
 * Copyright (c) 2014 ARM Limited
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

#include "dev/virtio/console.hh"

#include "debug/VIOConsole.hh"
#include "params/VirtIOConsole.hh"
#include "sim/system.hh"

VirtIOConsole::VirtIOConsole(Params *params)
    : VirtIODeviceBase(params, ID_CONSOLE, sizeof(Config), F_SIZE),
      qRecv(params->system->physProxy, byteOrder, params->qRecvSize, *this),
      qTrans(params->system->physProxy, byteOrder, params->qTransSize, *this),
      device(*params->device)
{
    registerQueue(qRecv);
    registerQueue(qTrans);

    config.cols = 80;
    config.rows = 24;

    device.regInterfaceCallback([this]() { qRecv.trySend(); });
}


VirtIOConsole::~VirtIOConsole()
{
}
void
VirtIOConsole::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    Config cfg_out;
    cfg_out.rows = htog(config.rows, byteOrder);
    cfg_out.cols = htog(config.cols, byteOrder);

    readConfigBlob(pkt, cfgOffset, (uint8_t *)&cfg_out);
}

void
VirtIOConsole::TermRecvQueue::trySend()
{
    DPRINTF(VIOConsole, "trySend\n");

    // Send data as long as the terminal has outgoing data and we can
    // get free descriptors (i.e., there are buffers available to
    // send) from the guest.
    VirtDescriptor *d;
    while (parent.device.dataAvailable() && (d = consumeDescriptor())) {
        DPRINTF(VIOConsole, "Got descriptor (len: %i)\n", d->size());
        size_t len(0);
        while (parent.device.dataAvailable() && len < d->size()) {
            uint8_t in(parent.device.readData());
            d->chainWrite(len, &in, sizeof(uint8_t));
            ++len;
        }

        // Tell the guest that we are done with this descriptor.
        produceDescriptor(d, len);
        parent.kick();
    }
}

void
VirtIOConsole::TermTransQueue::onNotifyDescriptor(VirtDescriptor *desc)
{
    DPRINTF(VIOConsole, "Got input data descriptor (len: %i)\n",
            desc->size());

    // Copy the data from the guest and forward it to the
    // terminal.
    const size_t size(desc->chainSize());
    uint8_t data[size];
    desc->chainRead(0, data, size);
    for (int i = 0; i < desc->size(); ++i)
        parent.device.writeData(data[i]);

    // Tell the guest that we are done with this descriptor.
    produceDescriptor(desc, 0);
    parent.kick();
}

VirtIOConsole *
VirtIOConsoleParams::create()
{
    return new VirtIOConsole(this);
}
