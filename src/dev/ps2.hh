/*
 * Copyright (c) 2011 ARM Limited
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
 *
 * Authors: Ali Saidi
 */

#ifndef __DEV_PS2_HH__
#define __DEV_PS2_HH__

#include <stdint.h>
#include <list>

#include "base/bitunion.hh"

/** @file misc functions and constants required to interface with or emulate ps2
 * devices
 */

namespace Ps2 {
enum {
    Ps2Reset        = 0xff,
    SelfTestPass    = 0xAA,
    SetStatusLed    = 0xed,
    SetResolution   = 0xe8,
    StatusRequest   = 0xe9,
    SetScaling1_2   = 0xe7,
    SetScaling1_1   = 0xe6,
    ReadId          = 0xf2,
    TpReadId        = 0xe1,
    Ack             = 0xfa,
    SetRate         = 0xf3,
    Enable          = 0xf4,
    Disable         = 0xf5,
    SetDefaults     = 0xf6,
    KeyboardId      = 0xab,
    TouchKitId      = 0x0a,
    MouseId         = 0x00,
};

/** A bitfield that represents the first byte of a mouse movement packet
 */
BitUnion8(Ps2MouseMovement)
    Bitfield<0> leftButton;
    Bitfield<1> rightButton;
    Bitfield<2> middleButton;
    Bitfield<3> one;
    Bitfield<4> xSign;
    Bitfield<5> ySign;
    Bitfield<6> xOverflow;
    Bitfield<7> yOverflow;
EndBitUnion(Ps2MouseMovement)

/** Convert an x11 key symbol into a set of ps2 charecters.
 * @param key x11 key symbol
 * @param down if the key is being pressed or released
 * @param cur_shift if device has already sent a shift
 * @param keys list of keys command to send to emulate the x11 key symbol
 */
void keySymToPs2(uint32_t key, bool down, bool &cur_shift,
        std::list<uint8_t> &keys);

} /* namespace Ps2 */
#endif // __DEV_PS2_HH__
