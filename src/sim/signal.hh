/*
 * Copyright 2022 Google, Inc.
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

#ifndef __SIM_SIGNAL_HH__
#define __SIM_SIGNAL_HH__

#include <functional>

#include "base/logging.hh"
#include "sim/port.hh"

namespace gem5
{

template <typename State>
class SignalSourcePort;

template <typename State>
class SignalSinkPort : public Port
{
  public:
    using OnChangeFunc = std::function<void(const State &new_val)>;

  private:
    friend SignalSourcePort<State>;

    SignalSourcePort<State> *_source = nullptr;

    State _state = {};
    OnChangeFunc _onChange;

  protected:
    void
    set(const State &new_state)
    {
        if (new_state == _state)
            return;

        _state = new_state;
        if (_onChange)
            _onChange(_state);
    }

  public:
    SignalSinkPort(const std::string &_name, PortID _id=InvalidPortID) :
        Port(_name, _id)
    {}

    const State &state() const { return _state; }
    void onChange(OnChangeFunc func) { _onChange = std::move(func); }

    void
    bind(Port &peer) override
    {
        _source = dynamic_cast<SignalSourcePort<State> *>(&peer);
        fatal_if(!_source, "Attempt to bind signal pin %s to "
                "incompatible pin %s", name(), peer.name());
        Port::bind(peer);
    }
    void
    unbind() override
    {
        _source = nullptr;
        Port::unbind();
    }
};

template <typename State>
class SignalSourcePort : public Port
{
  private:
    SignalSinkPort<State> *sink = nullptr;
    State _state = {};

  public:
    SignalSourcePort(const std::string &_name, PortID _id=InvalidPortID) :
        Port(_name, _id)
    {}

    void
    set(const State &new_state)
    {
        _state = new_state;
        sink->set(new_state);
    }

    const State &state() const { return _state; }

    void
    bind(Port &peer) override
    {
        sink = dynamic_cast<SignalSinkPort<State> *>(&peer);
        fatal_if(!sink, "Attempt to bind signal pin %s to "
                "incompatible pin %s", name(), peer.name());
        Port::bind(peer);
    }
    void
    unbind() override
    {
        sink = nullptr;
        Port::unbind();
    }
};

} // namespace gem5

#endif //__SIM_SIGNAL_HH__
