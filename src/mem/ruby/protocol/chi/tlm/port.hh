/*
 * Copyright (c) 2025 ARM Limited
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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_PORT_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_PORT_HH__

#include <type_traits>

#include "base/trace.hh"
#include "debug/TLMPort.hh"
#include "mem/ruby/protocol/chi/tlm/utils.hh"
#include "sim/signal.hh"

namespace gem5
{

namespace tlm::chi
{

using TlmData = std::pair<ARM::CHI::Payload *, ARM::CHI::Phase *>;

class SinkPortBase : public SignalSinkPort<TlmData>
{
  public:
    template <class Device>
    SinkPortBase(const std::string &_name, PortID _id, Device *dev)
        : SignalSinkPort(_name, _id)
    {}

    /**
     * We override the SignalSinkPort::set method as the base implementation
     * checks if the new state is equal to the old state and blocks the message
     * if that is true.
     * We don't want to define a complex equality operator which checks on both
     * Payload and Phae fields to see if there is any difference. Instead we
     * always forward the TlmData to the receiver.
     */
    void
    set(const TlmData &data, const bool bypass_on_change = false) override
    {
        _state = data;
        DPRINTF(TLMPort, "recv %s\n",
                transactionToString(*_state.first, *_state.second));
        if (_onChange) {
            _onChange(_state);
        }
    }
};

template <class Compat> using SinkPort = SinkPortBase;

class SourcePortBase : public SignalSourcePort<TlmData>
{
  public:
    template <class Device>
    SourcePortBase(const std::string &_name, PortID _id, Device *owner)
        : SignalSourcePort(_name, _id)
    {}

    void
    send(TlmData &data)
    {
        DPRINTF(TLMPort, "send %s\n",
                transactionToString(*data.first, *data.second));
        set(data);
    }
};

template <class Compat> using SourcePort = SourcePortBase;

} // namespace tlm::chi

} // namespace gem5

#endif //__MEM_RUBY_PROTOCOL_CHI_TLM_PORT_HH__
