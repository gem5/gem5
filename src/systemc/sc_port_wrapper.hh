/*
 * Copyright 2019 Google LLC.
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

#ifndef __SYSTEMC_SC_PORT_WRAPPER_HH__
#define __SYSTEMC_SC_PORT_WRAPPER_HH__

#include <string>
#include <type_traits>

#include "base/logging.hh"
#include "sim/port.hh"
#include "systemc/ext/core/sc_export.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_port.hh"

namespace sc_gem5
{

// Forward declaration
template <typename IF>
class ScPortWrapper;
template <typename IF>
class ScInterfaceWrapper;
template <typename IF>
class ScExportWrapper;

template <typename IF>
class ScPortWrapper : public gem5::Port
{
  public:
    using ScPort = sc_core::sc_port_b<IF>;

    ScPortWrapper(ScPort& p, const std::string& name, gem5::PortID id)
        : gem5::Port(name, id), port_(p)
    {}

    ScPort&
    port()
    {
        return port_;
    }

    void
    unbind() override
    {
        using namespace gem5;

        panic("sc_port can't be unbound.");
    }

    void
    bind(gem5::Port& peer) override
    {
        using namespace gem5;

        // Try ScPortWrapper or ScInterfaceWrapper
        if (auto* beer = dynamic_cast<ScPortWrapper<IF>*>(&peer)) {
            port_.bind(beer->port());
        } else if (auto* iface =
                       dynamic_cast<ScInterfaceWrapper<IF>*>(&peer)) {
            port_.bind(iface->interface());
        } else {
            fatal("Attempt to bind sc_port %s to incompatible port %s.",
                  name(), peer.name());
        }
        gem5::Port::bind(peer);
    }

  private:
    ScPort& port_;
};

template <typename IF>
class ScInterfaceWrapper : public gem5::Port
{
  public:
    ScInterfaceWrapper(IF& i, const std::string name, gem5::PortID id)
        : gem5::Port(name, id), iface_(i)
    {}

    IF&
    interface()
    {
        return iface_;
    }

    void
    unbind() override
    {
        using namespace gem5;

        panic("sc_interface can't be unbound.");
    }

    void
    bind(gem5::Port& peer) override
    {
        using namespace gem5;

        // fatal error if peer is neither ScPortWrapper nor ScExportWrapper
        fatal_if(!dynamic_cast<ScPortWrapper<IF>*>(&peer) &&
                     !dynamic_cast<ScExportWrapper<IF>*>(&peer),
                 "Attempt to bind sc_interface %s to incompatible port %s.",
                 name(), peer.name());

        // Don't bind to peer otherwise we may have error messages saying that
        // this interface has already be bound since the peer may already did
        // that. Just let sc_port or sc_export do the binding
        gem5::Port::bind(peer);
    }

  private:
    IF& iface_;
};

template <typename IF>
class ScExportWrapper : public gem5::Port
{
  public:
    using ScExport = sc_core::sc_export<IF>;

    ScExportWrapper(ScExport& p, const std::string& name, gem5::PortID id)
        : gem5::Port(name, id), port_(p)
    {}

    ScExport&
    port()
    {
        return port_;
    }

    void
    unbind() override
    {
        using namespace gem5;

        panic("sc_export cannot be unbound.");
    }

    void
    bind(gem5::Port& peer) override
    {
        using namespace gem5;

        auto* iface = dynamic_cast<ScInterfaceWrapper<IF>*>(&peer);
        fatal_if(!iface,
                 "Attempt to bind sc_export %s to incompatible port %s.",
                 name(), peer.name());

        port_.bind(iface->interface());
        gem5::Port::bind(peer);
    }

  private:
    ScExport& port_;
};

}  // namespace sc_gem5

#endif  // __SYSTEMC_SC_PORT_WRAPPER_HH__
