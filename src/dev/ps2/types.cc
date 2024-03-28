/*
 * Copyright (c) 2011, 2018 ARM Limited
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

#include "dev/ps2/types.hh"

#include <list>

#include "base/logging.hh"
#include "x11keysym/keysym.h"

namespace gem5
{

namespace ps2
{

const std::vector<uint8_t> keyboard::ID{0xAB, 0x83};
const std::vector<uint8_t> mouse::ID{0x00};

/** Table to convert simple key symbols (0x00XX) into ps2 bytes. Lower byte
 * is the scan code to send and upper byte is if a modifier is required to
 * generate it. The table generates us keyboard codes, (e.g. the guest is
 * supposed to recognize the keyboard as en_US). A new table would be required
 * for another locale.
 */

/* clang-format off */
static const uint16_t keySymToPs2Byte[128] = {
// 0 / 8   1 / 9   2 / A   3 / B   4 / C   5 / D   6 / E   7 / F
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 0x00-0x07
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 0x08-0x0f
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 0x10-0x17
   0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // 0x18-0x1f
   0x0029, 0x0116, 0x0152, 0x0126, 0x0125, 0x012e, 0x013d, 0x0052, // 0x20-0x27
   0x0146, 0x0145, 0x013e, 0x0155, 0x0041, 0x004e, 0x0049, 0x004a, // 0x28-0x2f
   0x0045, 0x0016, 0x001e, 0x0026, 0x0025, 0x002e, 0x0036, 0x003d, // 0x30-0x37
   0x003e, 0x0046, 0x014c, 0x004c, 0x0141, 0x0055, 0x0149, 0x014a, // 0x38-0x3f
   0x011e, 0x011c, 0x0132, 0x0121, 0x0123, 0x0124, 0x012b, 0x0134, // 0x40-0x47
   0x0133, 0x0143, 0x013b, 0x0142, 0x014b, 0x013a, 0x0131, 0x0144, // 0x48-0x4f
   0x014d, 0x0115, 0x012d, 0x011b, 0x012c, 0x013c, 0x012a, 0x011d, // 0x50-0x57
   0x0122, 0x0135, 0x011a, 0x0054, 0x005d, 0x005b, 0x0136, 0x014e, // 0x58-0x5f
   0x000e, 0x001c, 0x0032, 0x0021, 0x0023, 0x0024, 0x002b, 0x0034, // 0x60-0x67
   0x0033, 0x0043, 0x003b, 0x0042, 0x004b, 0x003a, 0x0031, 0x0044, // 0x68-0x6f
   0x004d, 0x0015, 0x002d, 0x001b, 0x002c, 0x003c, 0x002a, 0x001d, // 0x70-0x77
   0x0022, 0x0035, 0x001a, 0x0154, 0x015d, 0x015b, 0x010e, 0x0000  // 0x78-0x7f
};
/* clang-format on */

const uint8_t ShiftKey = 0x12;
const uint8_t BreakKey = 0xf0;
const uint8_t ExtendedKey = 0xe0;
const uint32_t UpperKeys = 0xff00;

void
keySymToPs2(uint32_t key, bool down, bool &cur_shift,
        std::list<uint8_t> &keys)
{
    if (key <= XK_asciitilde) {
        uint16_t tmp = keySymToPs2Byte[key];
        uint8_t code = tmp & 0xff;
        bool shift = tmp >> 8;

        if (down) {
            if (!cur_shift && shift) {
                keys.push_back(ShiftKey);
                cur_shift = true;
            }
            keys.push_back(code);
        } else {
            if (cur_shift && !shift) {
                keys.push_back(BreakKey);
                keys.push_back(ShiftKey);
                cur_shift = false;
            }
            keys.push_back(BreakKey);
            keys.push_back(code);
        }
    } else {
        if ((key & UpperKeys) == UpperKeys) {
            bool extended = false;
            switch (key) {
              case XK_BackSpace:
                keys.push_back(0x66);
                break;
              case XK_Tab:
                keys.push_back(0x0d);
                break;
              case XK_Return:
                keys.push_back(0x5a);
                break;
             case XK_Escape:
                keys.push_back(0x76);
                break;
             case XK_Delete:
                extended = true;
                keys.push_back(0x71);
                break;
             case XK_Home:
                extended = true;
                keys.push_back(0x6c);
                break;
             case XK_Left:
                extended = true;
                keys.push_back(0x6b);
                break;
             case XK_Right:
                extended = true;
                keys.push_back(0x74);
                break;
             case XK_Down:
                extended = true;
                keys.push_back(0x72);
                break;
             case XK_Up:
                extended = true;
                keys.push_back(0x75);
                break;
             case XK_Page_Up:
                extended = true;
                keys.push_back(0x7d);
                break;
             case XK_Page_Down:
                extended = true;
                keys.push_back(0x7a);
                break;
             case XK_End:
                extended = true;
                keys.push_back(0x69);
                break;
             case XK_Shift_L:
                keys.push_back(0x12);
                if (down)
                    cur_shift = true;
                else
                    cur_shift = false;
                break;
             case XK_Shift_R:
                keys.push_back(0x59);
                if (down)
                    cur_shift = true;
                else
                    cur_shift = false;
                break;
             case XK_Control_L:
                keys.push_back(0x14);
                break;
             case XK_Control_R:
                extended = true;
                keys.push_back(0x14);
                break;
             case XK_Alt_L:
                keys.push_back(0x11);
                break;
             case XK_Alt_R:
                extended = true;
                keys.push_back(0x11);
                break;
             default:
               warn("Unknown extended key %#x\n", key);
               return;
            }

            if (extended) {
                if (down) {
                    keys.push_front(ExtendedKey);
                } else {
                    keys.push_front(BreakKey);
                    keys.push_front(ExtendedKey);
                }
            } else {
                if (!down)
                    keys.push_front(BreakKey);
            }
        } // upper keys
    } // extended keys
    return;
}

} // namespace ps2
} // namespace gem5
