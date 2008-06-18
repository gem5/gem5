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
 *
 * Authors: Gabe Black
 */

/** @file
 * Implementation of PC platform.
 */

#include <deque>
#include <string>
#include <vector>

#include "arch/x86/x86_traits.hh"
#include "dev/intel_8254_timer.hh"
#include "cpu/intr_control.hh"
#include "dev/terminal.hh"
#include "dev/x86/pc.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

PC::PC(const Params *p)
    : Platform(p), system(p->system)
{
    southBridge = NULL;
    // set the back pointer from the system to myself
    system->platform = this;
}

void
PC::init()
{
    assert(southBridge);
    Intel8254Timer & timer = southBridge->pit.pit;
    //Timer 0, mode 2, no bcd, 16 bit count
    timer.writeControl(0x34);
    //Timer 0, latch command
    timer.writeControl(0x00);
    //Write a 16 bit count of 0
    timer.counter0.write(0);
    timer.counter0.write(0);
}

Tick
PC::intrFrequency()
{
    panic("Need implementation\n");
    M5_DUMMY_RETURN
}

void
PC::postConsoleInt()
{
    warn_once("Don't know what interrupt to post for console.\n");
    //panic("Need implementation\n");
}

void
PC::clearConsoleInt()
{
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}

void
PC::postPciInt(int line)
{
    panic("Need implementation\n");
}

void
PC::clearPciInt(int line)
{
    panic("Need implementation\n");
}

Addr
PC::pciToDma(Addr pciAddr) const
{
    panic("Need implementation\n");
    M5_DUMMY_RETURN
}


Addr
PC::calcConfigAddr(int bus, int dev, int func)
{
    assert(func < 8);
    assert(dev < 32);
    assert(bus == 0);
    return (PhysAddrPrefixPciConfig | (func << 8) | (dev << 11));
}

PC *
PCParams::create()
{
    return new PC(this);
}
