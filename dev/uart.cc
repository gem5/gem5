/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Implements a 8250 UART
 */

#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/str.hh"        // for to_number
#include "base/trace.hh"
#include "dev/simconsole.hh"
#include "dev/uart.hh"
#include "dev/platform.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"

using namespace std;

Uart::Uart(const string &name, SimConsole *c, MemoryController *mmu, Addr a,
           Addr s, HierParams *hier, Bus *bus, Tick pio_latency, Platform *p)
    : PioDevice(name, p), addr(a), size(s), cons(c)
{
    mmu->add_child(this, RangeSize(addr, size));


    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &Uart::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * bus->clockRate;
    }

    status = 0;

    // set back pointers
    cons->uart = this;
    platform->uart = this;
}

Tick
Uart::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}

DEFINE_SIM_OBJECT_CLASS_NAME("Uart", Uart)

