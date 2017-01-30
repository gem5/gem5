/*
 * Copyright (c) 2017 ARM Limited
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

#include "pybind11/pybind11.h"

#include <string>

#include "config/the_isa.hh"

#if THE_ISA != NULL_ISA
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherobject.hh"

#endif

#include "mem/mem_object.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "sim/full_system.hh"

namespace py = pybind11;

#if THE_ISA != NULL_ISA
static EtherInt *
lookupEthPort(SimObject *so, const std::string &name, int i)
{
    EtherObject *eo = dynamic_cast<EtherObject *>(so);
    EtherDevice *ed = dynamic_cast<EtherDevice *>(so);
    if (eo == NULL && ed == NULL) {
        warn("error casting SimObject %s", so->name());
        return NULL;
    }

    EtherInt *p = NULL;
    if (eo)
        p = eo->getEthPort(name, i);
    else
        p = ed->getEthPort(name, i);
    return p;
}
#endif

/**
 * Connect the described MemObject ports.  Called from Python.
 * The indices i1 & i2 will be -1 for regular ports, >= 0 for vector ports.
 * SimObject1 is the master, and SimObject2 is the slave
 */
static int
connectPorts(SimObject *o1, const std::string &name1, int i1,
             SimObject *o2, const std::string &name2, int i2)
{
#if THE_ISA != NULL_ISA
    EtherObject *eo1, *eo2;
    EtherDevice *ed1, *ed2;
    eo1 = dynamic_cast<EtherObject*>(o1);
    ed1 = dynamic_cast<EtherDevice*>(o1);
    eo2 = dynamic_cast<EtherObject*>(o2);
    ed2 = dynamic_cast<EtherDevice*>(o2);

    if ((eo1 || ed1) && (eo2 || ed2)) {
        EtherInt *p1 = lookupEthPort(o1, name1, i1);
        EtherInt *p2 = lookupEthPort(o2, name2, i2);

        if (p1 != NULL &&  p2 != NULL) {

            p1->setPeer(p2);
            p2->setPeer(p1);

            return 1;
        }
    }
#endif

    // These could be MessageBuffers from the ruby memory system. If so, they
    // need not be connected to anything currently.
    MessageBuffer *mb1, *mb2;
    mb1 = dynamic_cast<MessageBuffer*>(o1);
    mb2 = dynamic_cast<MessageBuffer*>(o2);

    if (mb1 || mb2) {
        // No need to connect anything here currently. MessageBuffer
        // connections in Python only serve to print the connections in
        // the config output.
        // TODO: Add real ports to MessageBuffers and use MemObject connect
        // code below to bind MessageBuffer senders and receivers
        return 1;
    }

    MemObject *mo1, *mo2;
    mo1 = dynamic_cast<MemObject*>(o1);
    mo2 = dynamic_cast<MemObject*>(o2);

    if (mo1 == NULL || mo2 == NULL) {
        panic ("Error casting SimObjects %s and %s to MemObject", o1->name(),
               o2->name());
    }

    // generic master/slave port connection
    BaseMasterPort& masterPort = mo1->getMasterPort(name1, i1);
    BaseSlavePort& slavePort   = mo2->getSlavePort(name2, i2);

    masterPort.bind(slavePort);

    return 1;
}

void
pybind_init_pyobject(py::module &m_native)
{
    py::module m = m_native.def_submodule("pyobject");

    m.def("connectPorts", &connectPorts);
}
