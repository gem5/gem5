/*
 * Copyright 2018 Google, Inc.
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

#include "systemc/core/port.hh"

#include "base/logging.hh"
#include "systemc/core/process.hh"
#include "systemc/core/sensitivity.hh"
#include "systemc/ext/channel/messages.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"

namespace sc_gem5
{

void
Port::finalizePort(StaticSensitivityPort *port)
{
    for (int i = 0; i < size(); i++)
        port->addEvent(&getInterface(i)->default_event());
}

void
Port::finalizeFinder(StaticSensitivityFinder *finder)
{
    for (int i = 0; i < size(); i++)
        finder->addEvent(&finder->find(getInterface(i)));
}

void
Port::finalizeReset(Reset *reset)
{
    assert(size() <= 1);
    if (size()) {
        auto iface =
            dynamic_cast<sc_core::sc_signal_in_if<bool> *>(getInterface(0));
        assert(iface);
        if (!reset->install(iface))
            delete reset;
    }
}

void
Port::sensitive(StaticSensitivityPort *port)
{
    if (finalized)
        finalizePort(port);
    else
        sensitivities.push_back(new Sensitivity(port));
}

void
Port::sensitive(StaticSensitivityFinder *finder)
{
    if (finalized)
        finalizeFinder(finder);
    else
        sensitivities.push_back(new Sensitivity(finder));
}

void
Port::addReset(Reset *reset)
{
    if (finalized)
        finalizeReset(reset);
    else
        resets.push_back(reset);
}

void
Port::finalize()
{
    if (finalized)
        return;
    finalized = true;

    for (auto &b : bindings) {
        if (b->interface) {
            addInterface(b->interface);
        } else {
            b->port->_gem5Port->finalize();
            addInterfaces(b->port);
        }
        delete b;
    }

    bindings.clear();

    for (auto &s : sensitivities) {
        if (s->port)
            finalizePort(s->port);
        else
            finalizeFinder(s->finder);
        delete s;
    }

    sensitivities.clear();

    for (auto &r : resets)
        finalizeReset(r);

    resets.clear();

    if (size() > maxSize()) {
        std::ostringstream ss;
        ss << size() << " binds exceeds maximum of " << maxSize()
           << " allowed";
        portBase->report_error(sc_core::SC_ID_COMPLETE_BINDING_,
                               ss.str().c_str());
    }

    switch (portBase->_portPolicy()) {
    case sc_core::SC_ONE_OR_MORE_BOUND:
        if (size() == 0)
            portBase->report_error(sc_core::SC_ID_COMPLETE_BINDING_,
                                   "port not bound");
        break;
    case sc_core::SC_ALL_BOUND:
        if (size() < maxSize() || size() < 1) {
            std::stringstream ss;
            ss << size() << " actual binds is less than required "
               << maxSize();
            portBase->report_error(sc_core::SC_ID_COMPLETE_BINDING_,
                                   ss.str().c_str());
        }
        break;
    case sc_core::SC_ZERO_OR_MORE_BOUND:
        break;
    default:
        panic("Unrecognized port policy %d.", portBase->_portPolicy());
    }
}

void
Port::regPort()
{
    if (!regPortNeeded)
        return;

    for (int i = 0; i < size(); i++)
        getInterface(i)->register_port(*portBase, portBase->_ifTypeName());
}

std::list<Port *> allPorts;

} // namespace sc_gem5
