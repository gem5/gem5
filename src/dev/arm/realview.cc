/*
 * Copyright (c) 2009, 2014 ARM Limited
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
 * Authors: Ali Saidi
 */

/** @file
 * Implementation of RealView platform.
 */

#include <deque>
#include <string>
#include <vector>

#include "config/the_isa.hh"
#include "cpu/intr_control.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/realview.hh"
#include "dev/terminal.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

RealView::RealView(const Params *p)
    : Platform(p), system(p->system), gic(nullptr)
{}

void
RealView::initState()
{
    Addr junk;
    bool has_gen_pci_host;
    has_gen_pci_host = system->kernelSymtab->findAddress("gen_pci_setup", junk);

    if (has_gen_pci_host && !params()->pci_cfg_gen_offsets)
        warn("Kernel supports generic PCI host but PCI Config offsets "
                "configured for legacy. Set pci_cfg_gen_offsets to True");
    if (has_gen_pci_host && !params()->pci_io_base)
        warn("Kernel supports generic PCI host but PCI IO base is set "
                "to 0. Set pci_io_base to the start of PCI IO space");
}

void
RealView::postConsoleInt()
{
    warn_once("Don't know what interrupt to post for console.\n");
    //panic("Need implementation\n");
}

void
RealView::clearConsoleInt()
{
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}

void
RealView::postPciInt(int line)
{
    gic->sendInt(line);
}

void
RealView::clearPciInt(int line)
{
    gic->clearInt(line);
}

Addr
RealView::pciToDma(Addr pciAddr) const
{
    return pciAddr;
}


Addr
RealView::calcPciConfigAddr(int bus, int dev, int func)
{
    if (bus != 0)
        return ULL(-1);

    Addr cfg_offset = 0;
    if (params()->pci_cfg_gen_offsets)
        cfg_offset |= ((func & 7) << 12) | ((dev & 0x1f) << 15);
    else
        cfg_offset |= ((func & 7) << 16) | ((dev & 0x1f) << 19);
    return params()->pci_cfg_base | cfg_offset;
}

Addr
RealView::calcPciIOAddr(Addr addr)
{
    return params()->pci_io_base + addr;
}

Addr
RealView::calcPciMemAddr(Addr addr)
{
    return addr;
}

RealView *
RealViewParams::create()
{
    return new RealView(this);
}
