/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 */

#include <Python.h>

#include "base/types.hh"
#include "dev/etherint.hh"
#include "sim/serialize.hh"
#include "sim/sim_object.hh"

extern "C" SimObject *convertSwigSimObjectPtr(PyObject *);

/** Resolve a SimObject name using the Python configuration */
class PythonSimObjectResolver : public SimObjectResolver
{
    SimObject *resolveSimObject(const std::string &name);
};

EtherInt * lookupEthPort(SimObject *so, const std::string &name, int i);

/**
 * Connect the described MemObject ports.  Called from Python via SWIG.
 */
int connectPorts(SimObject *o1, const std::string &name1, int i1,
    SimObject *o2, const std::string &name2, int i2);


inline void
serializeAll(const std::string &cpt_dir)
{
    Serializable::serializeAll(cpt_dir);
}

CheckpointIn *
getCheckpoint(const std::string &cpt_dir);

inline void
unserializeGlobals(CheckpointIn &cp)
{
    Serializable::unserializeGlobals(cp);
}
