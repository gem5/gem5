/*
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

/** @file
 * Implementation of PC platform.
 */

#include "dev/x86/pc.hh"

#include "arch/x86/intmessage.hh"
#include "arch/x86/x86_traits.hh"
#include "dev/x86/i82094aa.hh"
#include "dev/x86/i8254.hh"
#include "dev/x86/i8259.hh"
#include "dev/x86/south_bridge.hh"
#include "sim/system.hh"

namespace gem5
{

Pc::Pc(const Params &p) : Platform(p), southBridge(p.south_bridge)
{}

void
Pc::init()
{
    assert(southBridge);

    /*
     * Initialize the timer.
     */
    auto &timer = *southBridge->pit;
    //Timer 0, mode 2, no bcd, 16 bit count
    timer.writeControl(0x34);
    //Timer 0, latch command
    timer.writeControl(0x00);
    //Write a 16 bit count of 0
    timer.writeCounter(0, 0);
    timer.writeCounter(0, 0);

    /*
     * Initialize the I/O APIC.
     */
    X86ISA::I82094AA &ioApic = *southBridge->ioApic;
    X86ISA::I82094AA::RedirTableEntry entry = 0;
    entry.mask = 1;
    entry.deliveryMode = X86ISA::delivery_mode::ExtInt;
    entry.vector = 0x20;
    ioApic.writeReg(0x10, entry.bottomDW);
    ioApic.writeReg(0x11, entry.topDW);
    entry.deliveryMode = X86ISA::delivery_mode::Fixed;
    entry.vector = 0x24;
    ioApic.writeReg(0x18, entry.bottomDW);
    ioApic.writeReg(0x19, entry.topDW);
    entry.vector = 0x21;
    ioApic.writeReg(0x12, entry.bottomDW);
    ioApic.writeReg(0x13, entry.topDW);
    entry.vector = 0x20;
    ioApic.writeReg(0x14, entry.bottomDW);
    ioApic.writeReg(0x15, entry.topDW);
    entry.vector = 0x28;
    ioApic.writeReg(0x20, entry.bottomDW);
    ioApic.writeReg(0x21, entry.topDW);
    entry.vector = 0x2C;
    ioApic.writeReg(0x28, entry.bottomDW);
    ioApic.writeReg(0x29, entry.topDW);
    entry.vector = 0x2E;
    ioApic.writeReg(0x2C, entry.bottomDW);
    ioApic.writeReg(0x2D, entry.topDW);
    entry.vector = 0x30;
    ioApic.writeReg(0x30, entry.bottomDW);
    ioApic.writeReg(0x31, entry.topDW);

    /*
     * Mask the PICs. I'm presuming the BIOS/bootloader would have cleared
     * these out and masked them before passing control to the OS.
     */
    southBridge->pic1->maskAll();
    southBridge->pic2->maskAll();
}

void
Pc::postConsoleInt()
{
    southBridge->ioApic->requestInterrupt(4);
    southBridge->pic1->signalInterrupt(4);
}

void
Pc::clearConsoleInt()
{
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}

void
Pc::postPciInt(int line)
{
    southBridge->ioApic->requestInterrupt(line);
}

void
Pc::clearPciInt(int line)
{
    warn_once("Tried to clear PCI interrupt %d\n", line);
}

} // namespace gem5
