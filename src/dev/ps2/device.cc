/*
 * Copyright (c) 2017-2018 ARM Limited
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
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#include "dev/ps2/device.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PS2.hh"
#include "dev/ps2/types.hh"
#include "params/PS2Device.hh"

PS2Device::PS2Device(const PS2DeviceParams *p)
    : SimObject(p)
{
    inBuffer.reserve(16);
}

void
PS2Device::serialize(CheckpointOut &cp) const
{
    std::vector<uint8_t> buffer(outBuffer.size());
    std::copy(outBuffer.begin(), outBuffer.end(), buffer.begin());
    arrayParamOut(cp, "outBuffer", buffer);

    SERIALIZE_CONTAINER(inBuffer);
}

void
PS2Device::unserialize(CheckpointIn &cp)
{
    std::vector<uint8_t> buffer;
    arrayParamIn(cp, "outBuffer", buffer);
    for (auto c : buffer)
        outBuffer.push_back(c);

    UNSERIALIZE_CONTAINER(inBuffer);
}

void
PS2Device::hostRegDataAvailable(const std::function<void()> &c)
{
    fatal_if(dataAvailableCallback,
             "A data pending callback has already been associated with this "
             "PS/2 device.\n");

    dataAvailableCallback = c;
}

uint8_t
PS2Device::hostRead()
{
    uint8_t data = outBuffer.front();
    outBuffer.pop_front();
    return data;
}

void
PS2Device::hostWrite(uint8_t c)
{
    DPRINTF(PS2, "PS2: Host -> device: %#x\n", c);
    inBuffer.push_back(c);
    if (recv(inBuffer))
        inBuffer.clear();
}

void
PS2Device::send(const uint8_t *data, size_t size)
{
    assert(data || size == 0);
    while (size) {
        DPRINTF(PS2, "PS2: Device -> host: %#x\n", *data);
        outBuffer.push_back(*(data++));
        size--;
    }

    // Registering a callback is optional.
    if (dataAvailableCallback)
        dataAvailableCallback();
}

void
PS2Device::sendAck()
{
    send(Ps2::Ack);
}
