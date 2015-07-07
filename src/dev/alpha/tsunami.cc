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
 * Authors: Ali Saidi
 */

/** @file
 * Implementation of Tsunami platform.
 */

#include <deque>
#include <string>
#include <vector>

#include "arch/alpha/system.hh"
#include "config/the_isa.hh"
#include "cpu/intr_control.hh"
#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunami_cchip.hh"
#include "dev/alpha/tsunami_io.hh"
#include "dev/alpha/tsunami_pchip.hh"
#include "dev/terminal.hh"

using namespace std;
//Should this be AlphaISA?
using namespace TheISA;

Tsunami::Tsunami(const Params *p)
    : Platform(p), system(p->system)
{
    for (int i = 0; i < Tsunami::Max_CPUs; i++)
        intr_sum_type[i] = 0;
}

void
Tsunami::init()
{
    AlphaSystem *alphaSystem = dynamic_cast<AlphaSystem *>(system);
    assert(alphaSystem);
    alphaSystem->setIntrFreq(io->frequency());
}

void
Tsunami::postConsoleInt()
{
    io->postPIC(0x10);
}

void
Tsunami::clearConsoleInt()
{
    io->clearPIC(0x10);
}

void
Tsunami::postPciInt(int line)
{
    cchip->postDRIR(line);
}

void
Tsunami::clearPciInt(int line)
{
    cchip->clearDRIR(line);
}

Addr
Tsunami::pciToDma(Addr pciAddr) const
{
    return pchip->translatePciToDma(pciAddr);
}


Addr
Tsunami::calcPciConfigAddr(int bus, int dev, int func)
{
   return pchip->calcConfigAddr(bus, dev, func);
}

Addr
Tsunami::calcPciIOAddr(Addr addr)
{
   return pchip->calcIOAddr(addr);
}

Addr
Tsunami::calcPciMemAddr(Addr addr)
{
   return pchip->calcMemAddr(addr);
}

void
Tsunami::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(intr_sum_type, Tsunami::Max_CPUs);
}

void
Tsunami::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(intr_sum_type, Tsunami::Max_CPUs);
}

Tsunami *
TsunamiParams::create()
{
    return new Tsunami(this);
}
