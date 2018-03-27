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

#include "dev/mips/malta.hh"

#include <deque>
#include <string>
#include <vector>

#include "cpu/intr_control.hh"
#include "debug/Malta.hh"
#include "dev/mips/malta_cchip.hh"
#include "dev/mips/malta_io.hh"
#include "params/Malta.hh"
#include "sim/system.hh"

using namespace std;

Malta::Malta(const Params *p)
    : Platform(p), system(p->system)
{
    for (int i = 0; i < Malta::Max_CPUs; i++)
        intr_sum_type[i] = 0;
}

void
Malta::postConsoleInt()
{
    //see {Linux-src}/arch/mips/mips-boards/sim/sim_setup.c
    io->postIntr(0x10/*HW4*/);
}

void
Malta::clearConsoleInt()
{
    //FIXME: implement clearConsoleInt()
    io->clearIntr(0x10/*HW4*/);
}

void
Malta::postPciInt(int line)
{
    panic("Malta::postPciInt() has not been implemented.");
}

void
Malta::clearPciInt(int line)
{
    panic("Malta::clearPciInt() has not been implemented.");
}

Addr
Malta::pciToDma(Addr pciAddr) const
{
    panic("Malta::pciToDma() has not been implemented.");
}

void
Malta::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(intr_sum_type, Malta::Max_CPUs);
}

void
Malta::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(intr_sum_type, Malta::Max_CPUs);
}

Malta *
MaltaParams::create()
{
    return new Malta(this);
}
