/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "base/bitfield.hh"
#include "dev/x86/i8259.hh"

Tick
X86ISA::I8259::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - pioAddr)
    {
      case 0x0:
        if (readIRR) {
            DPRINTF(I8259, "Reading IRR as %#x.\n", IRR);
            pkt->set(IRR);
        } else {
            DPRINTF(I8259, "Reading ISR as %#x.\n", ISR);
            pkt->set(ISR);
        }
        break;
      case 0x1:
        DPRINTF(I8259, "Reading IMR as %#x.\n", IMR);
        pkt->set(IMR);
        break;
    }
    return latency;
}

Tick
X86ISA::I8259::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    uint8_t val = pkt->get<uint8_t>();
    switch (pkt->getAddr() - pioAddr) {
      case 0x0:
        if (bits(val, 4)) {
            DPRINTF(I8259, "Received initialization command word 1.\n");
            IMR = 0;
            edgeTriggered = bits(val, 3);
            DPRINTF(I8259, "%s triggered mode.\n",
                    edgeTriggered ? "Edge" : "Level");
            cascadeMode = !bits(val, 1);
            DPRINTF(I8259, "%s mode.\n",
                    cascadeMode ? "Cascade" : "Single");
            expectICW4 = bits(val, 0);
            initControlWord = 1;
            DPRINTF(I8259, "Expecting %d more bytes.\n", expectICW4 ? 3 : 2);
        } else if (bits(val, 4, 3) == 0) {
            DPRINTF(I8259, "Received operation command word 2.\n");
            switch (bits(val, 7, 5)) {
              case 0x0:
                DPRINTF(I8259,
                        "Subcommand: Rotate in auto-EOI mode (clear).\n");
                break;
              case 0x1:
                DPRINTF(I8259, "Subcommand: Nonspecific EOI.\n");
                break;
              case 0x2:
                DPRINTF(I8259, "Subcommand: No operation.\n");
                break;
              case 0x3:
                DPRINTF(I8259, "Subcommand: Specific EIO.");
                DPRINTF(I8259, "Reset In-Service bit %d.\n", bits(val, 2, 0));
                break;
              case 0x4:
                DPRINTF(I8259, "Subcommand: Rotate in auto-EOI mode (set).\n");
                break;
              case 0x5:
                DPRINTF(I8259, "Subcommand: Rotate on nonspecific EOI.\n");
                break;
              case 0x6:
                DPRINTF(I8259, "Subcommand: Set priority command.\n");
                DPRINTF(I8259, "Lowest: IRQ%d   Highest IRQ%d.\n",
                        bits(val, 2, 0), (bits(val, 2, 0) + 1) % 8);
                break;
              case 0x7:
                DPRINTF(I8259, "Subcommand: Rotate on specific EOI.\n");
                DPRINTF(I8259, "Lowest: IRQ%d   Highest IRQ%d.\n",
                        bits(val, 2, 0), (bits(val, 2, 0) + 1) % 8);
                break;
            }
        } else if (bits(val, 4, 3) == 1) {
            DPRINTF(I8259, "Received operation command word 3.\n");
            if (bits(val, 7)) {
                DPRINTF(I8259, "%s special mask mode.\n",
                        bits(val, 6) ? "Set" : "Clear");
            }
            if (bits(val, 1)) {
                readIRR = bits(val, 0);
                DPRINTF(I8259, "Read %s.\n", readIRR ? "IRR" : "ISR");
            }
        }
        break;
      case 0x1:
        switch (initControlWord) {
          case 0x0:
            DPRINTF(I8259, "Received operation command word 1.\n");
            DPRINTF(I8259, "Wrote IMR value %#x.\n", val);
            IMR = val;
            break;
          case 0x1:
            DPRINTF(I8259, "Received initialization command word 2.\n");
            DPRINTF(I8259, "Responsible for vectors %#x-%#x.\n",
                    val & ~mask(3), val | mask(3));
            if (cascadeMode) {
                initControlWord++;
            } else {
                cascadeBits = 0;
                initControlWord = 0;
            }
            break;
          case 0x2:
            DPRINTF(I8259, "Received initialization command word 3.\n");
            if (mode == Enums::I8259Master) {
                DPRINTF(I8259, "Slaves attached to IRQs:%s%s%s%s%s%s%s%s\n",
                        bits(val, 0) ? " 0" : "",
                        bits(val, 1) ? " 1" : "",
                        bits(val, 2) ? " 2" : "",
                        bits(val, 3) ? " 3" : "",
                        bits(val, 4) ? " 4" : "",
                        bits(val, 5) ? " 5" : "",
                        bits(val, 6) ? " 6" : "",
                        bits(val, 7) ? " 7" : "");
                cascadeBits = val;
            } else {
                DPRINTF(I8259, "Slave ID is %d.\n", val & mask(3));
                cascadeBits = val & mask(3);
            }
            if (expectICW4)
                initControlWord++;
            else
                initControlWord = 0;
            break;
          case 0x3:
            DPRINTF(I8259, "Received initialization command word 4.\n");
            if (bits(val, 4)) {
                DPRINTF(I8259, "Special fully nested mode.\n");
            } else {
                DPRINTF(I8259, "Not special fully nested mode.\n");
            }
            if (bits(val, 3) == 0) {
                DPRINTF(I8259, "Nonbuffered.\n");
            } else if (bits(val, 2) == 0) {
                DPRINTF(I8259, "Buffered.\n");
            } else {
                DPRINTF(I8259, "Unrecognized buffer mode.\n");
            }
            DPRINTF(I8259, "%s End Of Interrupt.\n",
                    bits(val, 1) ? "Automatic" : "Normal");
            DPRINTF(I8259, "%s mode.\n", bits(val, 0) ? "80x86" : "MCX-80/85");
            initControlWord = 0;
            break;
        }
        break;
    }
    return latency;
}

void
X86ISA::I8259::signalInterrupt(int line)
{
    DPRINTF(I8259, "Interrupt raised on line %d.\n", line);
    if (line > 7)
        fatal("Line number %d doesn't exist. The max is 7.\n");
    if (bits(IMR, line)) {
        DPRINTF(I8259, "Interrupt %d was masked.\n", line);
    } else {
        if (output) {
            DPRINTF(I8259, "Propogating interrupt.\n");
            output->signalInterrupt();
        } else {
            warn("Received interrupt but didn't have "
                    "anyone to tell about it.\n");
        }
    }
}

X86ISA::I8259 *
I8259Params::create()
{
    return new X86ISA::I8259(this);
}
