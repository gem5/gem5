/*
 * Copyright (c) 2011-2012,2015,2017 ARM Limited
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
 */

/**
 * @file
 * Port Object Declaration.
 */

#ifndef __SIM_PORT_HH__
#define __SIM_PORT_HH__

#include <cassert>
#include <ostream>
#include <string>

#include "base/types.hh"

namespace gem5
{

/**
 * Ports are used to interface objects to each other.
 */
class Port
{

  private:

    /** Descriptive name (for DPRINTF output) */
    const std::string portName;

  protected:

    class UnboundPortException {};

    [[noreturn]] void reportUnbound() const;

    /**
     * A numeric identifier to distinguish ports in a vector, and set
     * to InvalidPortID in case this port is not part of a vector.
     */
    const PortID id;

    /**
     * A pointer to this port's peer.
     */
    Port *_peer;


    /**
     * Whether this port is currently connected to a peer port.
     */
    bool _connected;

    /**
     * Abstract base class for ports
     *
     * @param _name Port name including the owners name
     * @param _id A port identifier for vector ports
     */
    Port(const std::string& _name, PortID _id);

  public:

    /**
     * Virtual destructor due to inheritance.
     */
    virtual ~Port();

    /** Return a reference to this port's peer. */
    Port &getPeer() { return *_peer; }

    /** Return port name (for DPRINTF). */
    const std::string name() const { return portName; }

    /** Get the port id. */
    PortID getId() const { return id; }

    /** Attach to a peer port. */
    virtual void
    bind(Port &peer)
    {
        _peer = &peer;
        _connected = true;
    }

    /** Dettach from a peer port. */
    virtual void
    unbind()
    {
        _peer = nullptr;
        _connected = false;
    }

    /** Is this port currently connected to a peer? */
    bool isConnected() const { return _connected; }

    /** A utility function to make it easier to swap out ports. */
    void
    takeOverFrom(Port *old)
    {
        assert(old);
        assert(old->isConnected());
        assert(!isConnected());
        Port &peer = old->getPeer();
        assert(peer.isConnected());

        // Disconnect the original binding.
        old->unbind();
        peer.unbind();

        // Connect the new binding.
        peer.bind(*this);
        bind(peer);
    }
};

static inline std::ostream &
operator << (std::ostream &os, const Port &port)
{
    os << port.name();
    return os;
}

} // namespace gem5

#endif //__SIM_PORT_HH__
