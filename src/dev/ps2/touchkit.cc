/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
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

#include "dev/ps2/touchkit.hh"

#include <cstdint>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PS2.hh"
#include "dev/ps2/mouse.hh"
#include "dev/ps2/types.hh"
#include "params/PS2TouchKit.hh"

namespace gem5
{

namespace ps2
{

TouchKit::TouchKit(const PS2TouchKitParams &p)
    : Device(p), vnc(p.vnc), enabled(false), touchKitEnabled(false)
{
    if (vnc)
        vnc->setMouse(this);
}

void
TouchKit::serialize(CheckpointOut &cp) const
{
    Device::serialize(cp);

    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(touchKitEnabled);
}

void
TouchKit::unserialize(CheckpointIn &cp)
{
    Device::unserialize(cp);

    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(touchKitEnabled);
}

bool
TouchKit::recv(const std::vector<uint8_t> &data)
{
    switch (data[0]) {
    case Reset:
        DPRINTF(PS2, "Resetting device.\n");
        enabled = false;
        touchKitEnabled = false;
        sendAck();
        send(SelfTestPass);
        return true;

    case ReadID:
        sendAck();
        send(mouse::ID);
        return true;

    case Disable:
        DPRINTF(PS2, "Disabling device.\n");
        enabled = false;
        sendAck();
        return true;

    case Enable:
        DPRINTF(PS2, "Enabling device.\n");
        enabled = true;
        sendAck();
        return true;

    case DefaultsAndDisable:
        DPRINTF(PS2, "Setting defaults and disabling device.\n");
        enabled = false;
        sendAck();
        return true;

    case mouse::Scale1to1:
    case mouse::Scale2to1:
        sendAck();
        return true;

    case mouse::SetResolution:
    case mouse::SampleRate:
        sendAck();
        return data.size() == 2;

    case mouse::GetStatus:
        sendAck();
        send(0);
        send(2);   // default resolution
        send(100); // default sample rate
        return true;

    case TpReadId:
        // We're not a trackpoint device, this should make the probe
        // go away
        sendAck();
        send(0);
        send(0);
        sendAck();
        return true;

    case TouchKitDiag:
        return recvTouchKit(data);

    default:
        panic("Unknown byte received: %#x\n", data[0]);
    }
}

bool
TouchKit::recvTouchKit(const std::vector<uint8_t> &data)
{
    // Ack all incoming bytes
    sendAck();

    // Packet format is: 0x0A SIZE CMD DATA
    assert(data[0] == TouchKitDiag);
    if (data.size() < 3 || data.size() - 2 < data[1])
        return false;

    const uint8_t len = data[1];
    const uint8_t cmd = data[2];

    // We have received at least one TouchKit diagnostic
    // command. Enabled TouchKit reports.
    touchKitEnabled = true;

    switch (cmd) {
    case TouchKitActive:
        warn_if(len != 1, "Unexpected activate packet length: %u\n", len);
        sendTouchKit('A');
        return true;

    default:
        panic("Unimplemented touchscreen command: %#x\n", cmd);
    }
}

void
TouchKit::sendTouchKit(const uint8_t *data, size_t size)
{
    send(TouchKitDiag);
    send(size);
    for (int i = 0; i < size; ++i)
        send(data[i]);
}

void
TouchKit::mouseAt(uint16_t x, uint16_t y, uint8_t buttons)
{
    // If the driver hasn't initialized the device yet, no need to try and send
    // it anything. Similarly we can get vnc mouse events orders of magnitude
    // faster than m5 can process them. Only queue up two sets mouse movements
    // and don't add more until those are processed.
    if (!enabled || !touchKitEnabled || sendPending() > 10)
        return;

    // Convert screen coordinates to touchpad coordinates
    const uint16_t _x = (2047.0 / vnc->videoWidth()) * x;
    const uint16_t _y = (2047.0 / vnc->videoHeight()) * y;

    const uint8_t resp[] = {
        buttons,
        (uint8_t)(_x >> 7),
        (uint8_t)(_x & 0x7f),
        (uint8_t)(_y >> 7),
        (uint8_t)(_y & 0x7f),
    };

    send(resp, sizeof(resp));
}

} // namespace ps2
} // namespace gem5
