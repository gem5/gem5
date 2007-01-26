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
 * Implementation of T1000 platform.
 */

#include <deque>
#include <string>
#include <vector>

#include "cpu/intr_control.hh"
#include "dev/simconsole.hh"
#include "dev/sparc/t1000.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
//Should this be AlphaISA?
using namespace TheISA;

T1000::T1000(const string &name, System *s, IntrControl *ic)
    : Platform(name, ic), system(s)
{
    // set the back pointer from the system to myself
    system->platform = this;
}

Tick
T1000::intrFrequency()
{
    panic("Need implementation\n");
    M5_DUMMY_RETURN
}

void
T1000::postConsoleInt()
{
    warn_once("Don't know what interrupt to post for console.\n");
    //panic("Need implementation\n");
}

void
T1000::clearConsoleInt()
{
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}

void
T1000::postPciInt(int line)
{
    panic("Need implementation\n");
}

void
T1000::clearPciInt(int line)
{
    panic("Need implementation\n");
}

Addr
T1000::pciToDma(Addr pciAddr) const
{
    panic("Need implementation\n");
    M5_DUMMY_RETURN
}


Addr
T1000::calcConfigAddr(int bus, int dev, int func)
{
    panic("Need implementation\n");
    M5_DUMMY_RETURN
}

void
T1000::serialize(std::ostream &os)
{
    panic("Need implementation\n");
}

void
T1000::unserialize(Checkpoint *cp, const std::string &section)
{
    panic("Need implementation\n");
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(T1000)

    SimObjectParam<System *> system;
    SimObjectParam<IntrControl *> intrctrl;

END_DECLARE_SIM_OBJECT_PARAMS(T1000)

BEGIN_INIT_SIM_OBJECT_PARAMS(T1000)

    INIT_PARAM(system, "system"),
    INIT_PARAM(intrctrl, "interrupt controller")

END_INIT_SIM_OBJECT_PARAMS(T1000)

CREATE_SIM_OBJECT(T1000)
{
    return new T1000(getInstanceName(), system, intrctrl);
}

REGISTER_SIM_OBJECT("T1000", T1000)
