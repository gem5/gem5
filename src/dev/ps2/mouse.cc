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

#include "dev/ps2/mouse.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PS2.hh"
#include "dev/ps2/types.hh"
#include "params/PS2Mouse.hh"
#include "sim/serialize.hh"

namespace gem5
{

namespace ps2
{

PS2Mouse::PS2Mouse(const PS2MouseParams &p)
    : Device(p),
      status(0), resolution(4), sampleRate(100)
{
}

bool
PS2Mouse::recv(const std::vector<uint8_t> &data)
{
    switch (data[0]) {
      case ReadID:
        DPRINTF(PS2, "Mouse ID requested.\n");
        sendAck();
        send(mouse::ID);
        return true;
      case Disable:
        DPRINTF(PS2, "Disabling data reporting.\n");
        status.enabled = 0;
        sendAck();
        return true;
      case Enable:
        DPRINTF(PS2, "Enabling data reporting.\n");
        status.enabled = 1;
        sendAck();
        return true;
      case Resend:
        panic("Mouse resend unimplemented.\n");
      case Reset:
        DPRINTF(PS2, "Resetting the mouse.\n");
        sampleRate = 100;
        resolution = 4;
        status.twoToOne = 0;
        status.enabled = 0;
        sendAck();
        send(SelfTestPass);
        send(mouse::ID);
        return true;

      case mouse::Scale1to1:
        DPRINTF(PS2, "Setting mouse scale to 1:1.\n");
        status.twoToOne = 0;
        sendAck();
        return true;
      case mouse::Scale2to1:
        DPRINTF(PS2, "Setting mouse scale to 2:1.\n");
        status.twoToOne = 1;
        sendAck();
        return true;
      case mouse::SetResolution:
        if (data.size() == 1) {
            DPRINTF(PS2, "Setting mouse resolution.\n");
            sendAck();
            return false;
        } else {
            DPRINTF(PS2, "Mouse resolution set to %d.\n", data[1]);
            resolution = data[1];
            sendAck();
            return true;
        }
      case mouse::GetStatus:
        DPRINTF(PS2, "Getting mouse status.\n");
        sendAck();
        send((uint8_t *)&(status), 1);
        send(&resolution, sizeof(resolution));
        send(&sampleRate, sizeof(sampleRate));
        return true;
      case mouse::ReadData:
        panic("Reading mouse data unimplemented.\n");
      case mouse::ResetWrapMode:
        DPRINTF(PS2, "Resetting Wrap Mode\n");
        sendAck();
        return true;
      case mouse::WrapMode:
        panic("Setting mouse wrap mode unimplemented.\n");
      case mouse::RemoteMode:
        panic("Setting mouse remote mode unimplemented.\n");
      case mouse::SampleRate:
        if (data.size() == 1) {
            DPRINTF(PS2, "Setting mouse sample rate.\n");
            sendAck();
            return false;
        } else {
            DPRINTF(PS2, "Mouse sample rate %d samples "
                    "per second.\n", data[1]);
            sampleRate = data[1];
            sendAck();
            return true;
        }
      case DefaultsAndDisable:
        DPRINTF(PS2, "Disabling and resetting mouse.\n");
        sampleRate = 100;
        resolution = 4;
        status.twoToOne = 0;
        status.enabled = 0;
        sendAck();
        return true;
      default:
        warn("Unknown mouse command %#02x.\n", data[0]);
        send(Resend);
        return true;
    }
}

void
PS2Mouse::serialize(CheckpointOut &cp) const
{
    Device::serialize(cp);

    SERIALIZE_SCALAR(status);
    SERIALIZE_SCALAR(resolution);
    SERIALIZE_SCALAR(sampleRate);
}

void
PS2Mouse::unserialize(CheckpointIn &cp)
{
    Device::unserialize(cp);

    UNSERIALIZE_SCALAR(status);
    UNSERIALIZE_SCALAR(resolution);
    UNSERIALIZE_SCALAR(sampleRate);
}

} // namespace ps2
} // namespace gem5
