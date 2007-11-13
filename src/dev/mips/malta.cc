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
 *          Rick Strong
 */

/** @file
 * Implementation of Malta platform.
 */

#include <deque>
#include <string>
#include <vector>

#include "cpu/intr_control.hh"
#include "dev/simconsole.hh"
#include "dev/mips/malta_cchip.hh"
#include "dev/mips/malta_pchip.hh"
#include "dev/mips/malta_io.hh"
#include "dev/mips/malta.hh"
#include "params/Malta.hh"
#include "sim/system.hh"


using namespace std;
using namespace TheISA;

Malta::Malta(const Params *p)
    : Platform(p), system(p->system)
{
    // set the back pointer from the system to myself
    system->platform = this;

    for (int i = 0; i < Malta::Max_CPUs; i++)
        intr_sum_type[i] = 0;
}

Tick
Malta::intrFrequency()
{
    return io->frequency();
}

void
Malta::postConsoleInt()
{
     //panic("Malta::postConsoleInt() has not been implemented.");
    io->postIntr(0x10/*HW4*/);//see {Linux-src}/arch/mips/mips-boards/sim/sim_setup.c
}

void
Malta::clearConsoleInt()
{
        //FIXME: implement clearConsoleInt()
        //warn("Malta::clearConsoleInt() has not been implemented.");
    io->clearIntr(0x10/*HW4*/);
}

void
Malta::postPciInt(int line)
{
                panic("Malta::postPciInt() has not been implemented.");
    //cchip->postDRIR(line);
}

void
Malta::clearPciInt(int line)
{
                panic("Malta::clearPciInt() has not been implemented.");
    //cchip->clearDRIR(line);
}

Addr
Malta::pciToDma(Addr pciAddr) const
{
                panic("Malta::pciToDma() has not been implemented.");
    return pchip->translatePciToDma(pciAddr);
}


Addr
Malta::calcConfigAddr(int bus, int dev, int func)
{
        panic("Malta::calcConfigAddr() has not been implemented.");
   return pchip->calcConfigAddr(bus, dev, func);
}

void
Malta::serialize(std::ostream &os)
{

    SERIALIZE_ARRAY(intr_sum_type, Malta::Max_CPUs);
}

void
Malta::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(intr_sum_type, Malta::Max_CPUs);
}

Malta *
MaltaParams::create()
{
    return new Malta(this);
}
