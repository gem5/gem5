/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include <cstring>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/output.hh"
#include "sim/builder.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"
#include "sim/universe.hh"

using namespace std;

Tick curTick = 0;
Tick ticksPerSecond;
double __ticksPerMS;
double __ticksPerUS;
double __ticksPerNS;
double __ticksPerPS;

bool fullSystem;
ostream *outputStream;
ostream *configStream;

// Dummy Object
class Root : public SimObject
{
  public:
    Root(const std::string &name) : SimObject(name) {}
};

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Root)

    Param<bool> full_system;
    Param<Tick> frequency;
    Param<string> output_file;

END_DECLARE_SIM_OBJECT_PARAMS(Root)

BEGIN_INIT_SIM_OBJECT_PARAMS(Root)

    INIT_PARAM(full_system, "full system simulation"),
    INIT_PARAM(frequency, "tick frequency"),
    INIT_PARAM(output_file, "file to dump simulator output to")

END_INIT_SIM_OBJECT_PARAMS(Root)

CREATE_SIM_OBJECT(Root)
{
    static bool created = false;
    if (created)
        panic("only one root object allowed!");

    created = true;
    fullSystem = full_system;

#ifdef FULL_SYSTEM
    if (!fullSystem)
        panic("FULL_SYSTEM compiled and configuration not full_system");
#else
    if (fullSystem)
        panic("FULL_SYSTEM not compiled but configuration is full_system");
#endif

    ticksPerSecond = frequency;
    double freq = double(ticksPerSecond);
    __ticksPerMS = freq / 1.0e3;
    __ticksPerUS = freq / 1.0e6;
    __ticksPerNS = freq / 1.0e9;
    __ticksPerPS = freq / 1.0e12;

    outputStream = simout.find(output_file);

    return new Root(getInstanceName());
}

REGISTER_SIM_OBJECT("Root", Root)

