/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#include <cstring>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"

// Dummy Object
struct Root : public SimObject
{
    Root(const std::string &name) : SimObject(name) {}
};

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Root)

    Param<int> dummy; // needed below

END_DECLARE_SIM_OBJECT_PARAMS(Root)

BEGIN_INIT_SIM_OBJECT_PARAMS(Root)

    INIT_PARAM(dummy, "")  // All SimObjects must have params

END_INIT_SIM_OBJECT_PARAMS(Root)

CREATE_SIM_OBJECT(Root)
{
    static bool created = false;
    if (created)
        panic("only one root object allowed!");

    created = true;

    return  new Root(getInstanceName());
}

REGISTER_SIM_OBJECT("Root", Root)
