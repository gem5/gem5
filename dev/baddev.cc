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
 */

/** @file
 * BadDevice implemenation
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/baddev.hh"
#include "dev/platform.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

BadDevice::BadDevice(Params *p)
    : BasicPioDevice(p), devname(p->devic_ename)
{
    pioSize = 0xf;
}

Tick
BadDevice::read(Packet &pkt)
{
    panic("Device %s not imlpmented\n", devname);
}

Tick
BadDevice::write(Packet &pkt)
{
    panic("Device %s not imlpmented\n", devname);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

    Param<string> devicename;
    Param<Addr> pio_addr;
    SimObjectParam<AlphaSystem *> system;
    SimObjectParam<Platform *> platform;
    Param<Tick> pio_latency;

END_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

BEGIN_INIT_SIM_OBJECT_PARAMS(BadDevice)

    INIT_PARAM(devicename, "Name of device to error on"),
    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency", 1000)

END_INIT_SIM_OBJECT_PARAMS(BadDevice)

CREATE_SIM_OBJECT(BadDevice)
{
    BadDevice::Params *p = new BadDevice::Params;
    p->name =getInstanceName();
    p->platform = platform;
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->system = system;
    p->devicename = devicename;
    return new BadDevice(p);
}

REGISTER_SIM_OBJECT("BadDevice", BadDevice)
