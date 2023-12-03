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

#ifndef __SYSTEMC_CORE_PORT_HH__
#define __SYSTEMC_CORE_PORT_HH__

#include <list>
#include <typeinfo>
#include <vector>

#include "base/cprintf.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_port.hh"

namespace sc_gem5
{

class StaticSensitivityPort;
class StaticSensitivityFinder;
class Reset;

class Port;

extern std::list<Port *> allPorts;

class Port
{
  private:
    ::sc_core::sc_port_base *portBase;

    bool finalized;
    int _maxSize;
    int _size;

    bool regPortNeeded;

    void finalizePort(StaticSensitivityPort *port);
    void finalizeFinder(StaticSensitivityFinder *finder);
    void finalizeReset(Reset *reset);

    void
    addInterface(::sc_core::sc_interface *iface)
    {
        portBase->_gem5AddInterface(iface);
        _size++;
    }

    void
    addInterfaces(::sc_core::sc_port_base *pb)
    {
        // Only the ports farthest from the interfaces call register_port.
        pb->_gem5Port->regPortNeeded = false;
        for (int i = 0; i < pb->size(); i++)
            addInterface(pb->_gem5Interface(i));
    }

    ::sc_core::sc_interface *
    getInterface(int i)
    {
        return portBase->_gem5Interface(i);
    }

    struct Binding
    {
        explicit Binding(::sc_core::sc_interface *interface)
            : interface(interface), port(nullptr)
        {}

        explicit Binding(::sc_core::sc_port_base *port)
            : interface(nullptr), port(port)
        {}

        ::sc_core::sc_interface *interface;
        ::sc_core::sc_port_base *port;
    };

    struct Sensitivity
    {
        Sensitivity(StaticSensitivityPort *port) : port(port), finder(nullptr)
        {}

        Sensitivity(StaticSensitivityFinder *finder)
            : port(nullptr), finder(finder)
        {}

        StaticSensitivityPort *port;
        StaticSensitivityFinder *finder;
    };

    std::vector<Binding *> bindings;
    std::vector<Sensitivity *> sensitivities;
    std::vector<Reset *> resets;

  public:
    static Port *
    fromPort(const ::sc_core::sc_port_base *pb)
    {
        return pb->_gem5Port;
    }

    ::sc_core::sc_port_base *
    sc_port_base()
    {
        return portBase;
    }

    Port(::sc_core::sc_port_base *port_base, int max)
        : portBase(port_base),
          finalized(false),
          _maxSize(max),
          _size(0),
          regPortNeeded(true)
    {
        allPorts.push_front(this);
    }

    ~Port() { allPorts.remove(this); }

    void
    bind(::sc_core::sc_interface *interface)
    {
        if (bindings.empty())
            addInterface(interface);
        else
            bindings.push_back(new Binding(interface));
    }

    void
    bind(::sc_core::sc_port_base *port)
    {
        bindings.push_back(new Binding(port));
    }

    void sensitive(StaticSensitivityPort *port);
    void sensitive(StaticSensitivityFinder *finder);
    void addReset(Reset *reset);

    void finalize();
    void regPort();

    int
    size()
    {
        return _size;
    }

    int
    maxSize()
    {
        return _maxSize ? _maxSize : _size;
    }
};

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_PORT_HH__
