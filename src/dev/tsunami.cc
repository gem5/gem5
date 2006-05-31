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

#include "cpu/intr_control.hh"
#include "dev/simconsole.hh"
#include "dev/tsunami_cchip.hh"
#include "dev/tsunami_pchip.hh"
#include "dev/tsunami_io.hh"
#include "dev/tsunami.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
//Should this be AlphaISA?
using namespace TheISA;

Tsunami::Tsunami(const string &name, System *s, IntrControl *ic)
    : Platform(name, ic), system(s)
{
    // set the back pointer from the system to myself
    system->platform = this;

    for (int i = 0; i < Tsunami::Max_CPUs; i++)
        intr_sum_type[i] = 0;
}

Tick
Tsunami::intrFrequency()
{
    return io->frequency();
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

void
Tsunami::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(intr_sum_type, Tsunami::Max_CPUs);
}

void
Tsunami::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(intr_sum_type, Tsunami::Max_CPUs);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Tsunami)

    SimObjectParam<System *> system;
    SimObjectParam<IntrControl *> intrctrl;

END_DECLARE_SIM_OBJECT_PARAMS(Tsunami)

BEGIN_INIT_SIM_OBJECT_PARAMS(Tsunami)

    INIT_PARAM(system, "system"),
    INIT_PARAM(intrctrl, "interrupt controller")

END_INIT_SIM_OBJECT_PARAMS(Tsunami)

CREATE_SIM_OBJECT(Tsunami)
{
    return new Tsunami(getInstanceName(), system, intrctrl);
}

REGISTER_SIM_OBJECT("Tsunami", Tsunami)
