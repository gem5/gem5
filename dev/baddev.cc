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

/* @file
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

BadDevice::BadDevice(const string &name, Addr a, MemoryController *mmu,
                     HierParams *hier, Bus *bus, const string &devicename)
    : PioDevice(name, NULL), addr(a), devname(devicename)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &BadDevice::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
    }

}

Fault
BadDevice::read(MemReqPtr &req, uint8_t *data)
{

    panic("Device %s not imlpmented\n", devname);
    return No_Fault;
}

Fault
BadDevice::write(MemReqPtr &req, const uint8_t *data)
{
    panic("Device %s not imlpmented\n", devname);
    return No_Fault;
}

Tick
BadDevice::cacheAccess(MemReqPtr &req)
{
    return curTick;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

    SimObjectParam<Platform *> platform;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<HierParams *> hier;
    SimObjectParam<Bus*> io_bus;
    Param<Tick> pio_latency;
    Param<string> devicename;

END_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

BEGIN_INIT_SIM_OBJECT_PARAMS(BadDevice)

    INIT_PARAM(platform, "Platform"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency", 1000),
    INIT_PARAM(devicename, "Name of device to error on")

END_INIT_SIM_OBJECT_PARAMS(BadDevice)

CREATE_SIM_OBJECT(BadDevice)
{
    return new BadDevice(getInstanceName(), addr, mmu, hier, io_bus,
                         devicename);
}

REGISTER_SIM_OBJECT("BadDevice", BadDevice)
