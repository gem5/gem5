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
 *
 * Authors: Ali Saidi
 *          William Wang
 *          Andreas Sandberg
 */

#include "dev/ps2/touchkit.hh"

#include "base/logging.hh"
#include "debug/PS2.hh"
#include "dev/ps2.hh"
#include "params/PS2TouchKit.hh"

const uint8_t PS2TouchKit::ID[] = {0x00};

PS2TouchKit::PS2TouchKit(const PS2TouchKitParams *p)
    : PS2Device(p),
      vnc(p->vnc),
      driverInitialized(false)
{
    if (vnc)
        vnc->setMouse(this);
}

void
PS2TouchKit::serialize(CheckpointOut &cp) const
{
    PS2Device::serialize(cp);

    SERIALIZE_SCALAR(driverInitialized);
}

void
PS2TouchKit::unserialize(CheckpointIn &cp)
{
    PS2Device::unserialize(cp);

    UNSERIALIZE_SCALAR(driverInitialized);
}

bool
PS2TouchKit::recv(const std::vector<uint8_t> &data)
{
    switch (data[0]) {
      case Ps2::Ps2Reset:
        sendAck();
        send(Ps2::SelfTestPass);
        return true;

      case Ps2::SetResolution:
      case Ps2::SetRate:
      case Ps2::SetStatusLed:
        sendAck();
        return data.size() == 2;

      case Ps2::ReadId:
        sendAck();
        send((const uint8_t *)&ID, sizeof(ID));
        return true;

      case Ps2::TpReadId:
        // We're not a trackpoint device, this should make the probe
        // go away
        sendAck();
        send(0);
        send(0);
        sendAck();
        return true;

      case Ps2::SetScaling1_1:
      case Ps2::SetScaling1_2:
      case Ps2::Disable:
      case Ps2::Enable:
      case Ps2::SetDefaults:
        sendAck();
        return true;

      case Ps2::StatusRequest:
        sendAck();
        send(0);
        send(2); // default resolution
        send(100); // default sample rate
        return true;

      case Ps2::TouchKitId:
        sendAck();
        if (data.size() == 1) {
            send(Ps2::TouchKitId);
            send(1);
            send('A');

            return false;
        } else if (data.size() == 3) {
            driverInitialized = true;
            return true;
        } else {
            return false;
        }

      default:
        panic("Unknown byte received: %d\n", data[0]);
    }
}

void
PS2TouchKit::mouseAt(uint16_t x, uint16_t y, uint8_t buttons)
{
    // If the driver hasn't initialized the device yet, no need to try and send
    // it anything. Similarly we can get vnc mouse events orders of magnitude
    // faster than m5 can process them. Only queue up two sets mouse movements
    // and don't add more until those are processed.
    if (!driverInitialized || sendPending() > 10)
        return;

    // Convert screen coordinates to touchpad coordinates
    const uint16_t _x = (2047.0 / vnc->videoWidth()) * x;
    const uint16_t _y = (2047.0 / vnc->videoHeight()) * y;

    const uint8_t resp[] = {
        buttons,
        (uint8_t)(_x >> 7), (uint8_t)(_x & 0x7f),
        (uint8_t)(_y >> 7), (uint8_t)(_y & 0x7f),
    };

    send(resp, sizeof(resp));
}

PS2TouchKit *
PS2TouchKitParams::create()
{
    return new PS2TouchKit(this);
}
