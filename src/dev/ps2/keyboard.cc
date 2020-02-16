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

#include "dev/ps2/keyboard.hh"

#include "base/logging.hh"
#include "debug/PS2.hh"
#include "dev/ps2/types.hh"
#include "params/PS2Keyboard.hh"

PS2Keyboard::PS2Keyboard(const PS2KeyboardParams *p)
    : PS2Device(p),
      shiftDown(false),
      enabled(false)
{
    if (p->vnc)
        p->vnc->setKeyboard(this);
}

void
PS2Keyboard::serialize(CheckpointOut &cp) const
{
    PS2Device::serialize(cp);
    SERIALIZE_SCALAR(shiftDown);
    SERIALIZE_SCALAR(enabled);
}

void
PS2Keyboard::unserialize(CheckpointIn &cp)
{
    PS2Device::unserialize(cp);
    UNSERIALIZE_SCALAR(shiftDown);
    UNSERIALIZE_SCALAR(enabled);
}

bool
PS2Keyboard::recv(const std::vector<uint8_t> &data)
{
    switch (data[0]) {
      case Ps2::ReadID:
        DPRINTF(PS2, "Got keyboard read ID command.\n");
        sendAck();
        send(Ps2::Keyboard::ID);
        return true;
      case Ps2::Enable:
        DPRINTF(PS2, "Enabling the keyboard.\n");
        enabled = true;
        sendAck();
        return true;
      case Ps2::Disable:
        DPRINTF(PS2, "Disabling the keyboard.\n");
        enabled = false;
        sendAck();
        return true;
      case Ps2::DefaultsAndDisable:
        DPRINTF(PS2, "Disabling and resetting the keyboard.\n");
        enabled = false;
        sendAck();
        return true;
      case Ps2::Reset:
        DPRINTF(PS2, "Resetting keyboard.\n");
        enabled = true;
        sendAck();
        send(Ps2::SelfTestPass);
        return true;
      case Ps2::Resend:
        panic("Keyboard resend unimplemented.\n");

      case Ps2::Keyboard::LEDWrite:
        if (data.size() == 1) {
            DPRINTF(PS2, "Got LED write command.\n");
            sendAck();
            return false;
        } else {
            DPRINTF(PS2, "Setting LEDs: "
                    "caps lock %s, num lock %s, scroll lock %s\n",
                    bits(data[1], 2) ? "on" : "off",
                    bits(data[1], 1) ? "on" : "off",
                    bits(data[1], 0) ? "on" : "off");
            sendAck();
            return true;
        }
      case Ps2::Keyboard::DiagnosticEcho:
        panic("Keyboard diagnostic echo unimplemented.\n");
      case Ps2::Keyboard::AlternateScanCodes:
        panic("Accessing alternate scan codes unimplemented.\n");
      case Ps2::Keyboard::TypematicInfo:
        if (data.size() == 1) {
            DPRINTF(PS2, "Setting typematic info.\n");
            sendAck();
            return false;
        } else {
            DPRINTF(PS2, "Setting typematic info to %#02x.\n", data[1]);
            sendAck();
            return true;
        }
      case Ps2::Keyboard::AllKeysToTypematic:
        panic("Setting all keys to typemantic unimplemented.\n");
      case Ps2::Keyboard::AllKeysToMakeRelease:
        panic("Setting all keys to make/release unimplemented.\n");
      case Ps2::Keyboard::AllKeysToMake:
        panic("Setting all keys to make unimplemented.\n");
      case Ps2::Keyboard::AllKeysToTypematicMakeRelease:
        panic("Setting all keys to "
                "typematic/make/release unimplemented.\n");
      case Ps2::Keyboard::KeyToTypematic:
        panic("Setting a key to typematic unimplemented.\n");
      case Ps2::Keyboard::KeyToMakeRelease:
        panic("Setting a key to make/release unimplemented.\n");
      case Ps2::Keyboard::KeyToMakeOnly:
        panic("Setting key to make only unimplemented.\n");
      default:
        panic("Unknown keyboard command %#02x.\n", data[0]);
    }
}

void
PS2Keyboard::keyPress(uint32_t key, bool down)
{
    std::list<uint8_t> keys;

    // convert the X11 keysym into ps2 codes and update the shift
    // state (shiftDown)
    Ps2::keySymToPs2(key, down, shiftDown, keys);

    // Drop key presses if the keyboard hasn't been enabled by the
    // host. We do that after translating the key code to ensure that
    // we keep track of the shift state.
    if (!enabled)
        return;

    // Insert into our queue of characters
    for (uint8_t c : keys)
        send(c);
}


PS2Keyboard *
PS2KeyboardParams::create()
{
    return new PS2Keyboard(this);
}
