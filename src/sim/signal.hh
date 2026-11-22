/*
 * Copyright (c) 2025 Arm Limited
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

#include <cstdint>
#include <functional>
#include <variant>

#include "base/logging.hh"
#include "sim/port.hh"

namespace gem5
{

template <typename State>
class SignalSourcePort;

template <typename State, typename Enable = void>
class SignalSinkPort : public Port
{
  public:
    using OnChangeFunc = std::function<void(const State &new_val)>;

  protected:
    friend SignalSourcePort<State>;

    SignalSourcePort<State> *_source = nullptr;

    State _state = {};

  protected:
    // if bypass_on_change is specified true, it will not call the _onChange
    // function. Only _state will be updated if needed.
    virtual void
    set(const State &new_state, const bool bypass_on_change = false)
    {
        if (new_state == _state)
            return;

        _state = new_state;
        if (!bypass_on_change && _onChange)
            _onChange(_state);
    }

    OnChangeFunc _onChange;

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
        // The state of sink has to match the state of source.
        _state = _source->state();
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
class SignalSinkPort<State, std::enable_if_t<std::is_integral_v<State>>>
    : public Port {
  public:
    using OnChangeFunc = std::function<void(const State &new_val)>;

  private:
    friend SignalSourcePort<State>;
    friend SignalSourcePort<int64_t>;

    SignalSourcePort<State> *_source = nullptr;
    SignalSourcePort<int64_t> *_number_source = nullptr;

    State _state = {};

  protected:
    // if bypass_on_change is specified true, it will not call the _onChange
    // function. Only _state will be updated if needed.
    virtual void
    set(const State &new_state, const bool bypass_on_change = false)
    {
        if (new_state == _state)
            return;

        _state = new_state;
        if (!bypass_on_change && _onChange)
            _onChange(_state);
    }

    OnChangeFunc _onChange;

  public:
    SignalSinkPort(const std::string &_name, PortID _id=InvalidPortID) :
        Port(_name, _id)
    {}

    const State &state() const { return _state; }
    void onChange(OnChangeFunc func) { _onChange = std::move(func); }

    void bind(Port &peer) override;

    void
    unbind() override
    {
        _source = nullptr;
        Port::unbind();
    }
};

template <>
class SignalSourcePort<int64_t> : public Port
{
  private:
    std::variant<
        std::monostate,
        SignalSinkPort<bool>*,
        SignalSinkPort<uint32_t>*> numeric_sink_ptr;
    int64_t _state;

  public:
    SignalSourcePort(const std::string &_name, PortID _id = InvalidPortID)
        : Port(_name, _id)
    {
        _state = {};
    }

    // Give an initial value to the _state instead of using a default value.
    SignalSourcePort(const std::string &_name, PortID _id,
                     const int64_t &init_state)
        : SignalSourcePort(_name, _id)
    {
        _state = init_state;
    }

    // if bypass_on_change is specified true, it will not call the _onChange
    // function. Only _state will be updated if needed.
    void
    set(const int64_t &new_state, const bool bypass_on_change = false)
    {
        _state = new_state;
        std::visit([this, new_state, bypass_on_change](auto &&sink_ptr){
            using SinkPtrType = std::decay_t<decltype(sink_ptr)>;
            if constexpr (std::is_same_v<SinkPtrType, SignalSinkPort<bool>*>) {
                warn_if(new_state > 1 || new_state < 0,
                    "Value %lld from %s is out of range for bool signal %s.",
                    new_state, name().c_str(), sink_ptr->name().c_str());
                sink_ptr->set(new_state != 0, bypass_on_change);
            }
            else if constexpr (std::is_same_v<SinkPtrType,
                SignalSinkPort<uint32_t>*>) {
                warn_if(new_state < 0 ||
                    new_state > std::numeric_limits<uint32_t>::max(),
                    "Value %lld from %s is out of range for uint32 signal %s.",
                    new_state, name().c_str(), sink_ptr->name().c_str());
                sink_ptr->set(static_cast<uint32_t>(new_state),
                              bypass_on_change);
            }
        }, numeric_sink_ptr);
    }

    const int64_t &state() const { return _state; }

    void
    bind(Port &peer) override
    {
        numeric_sink_ptr = std::monostate{};

        if (auto *bool_s = dynamic_cast<SignalSinkPort<bool> *>(&peer)) {
            numeric_sink_ptr = bool_s;
        }
        if (auto *uint32_s = dynamic_cast<SignalSinkPort<uint32_t> *>(&peer)) {
            numeric_sink_ptr = uint32_s;
        }

        fatal_if(std::holds_alternative<std::monostate>(numeric_sink_ptr),
             "Attempt to bind numeric signal pin %s to incompatible pin %s",
             name(), peer.name());
        Port::bind(peer);
    }
    void
    unbind() override
    {
        numeric_sink_ptr = std::monostate{};
        Port::unbind();
    }
};

template <typename State>
class SignalSourcePort : public Port
{
  private:
    SignalSinkPort<State> *sink = nullptr;
    State _state;

  public:
    SignalSourcePort(const std::string &_name, PortID _id = InvalidPortID)
        : Port(_name, _id)
    {
        _state = {};
    }

    // Give an initial value to the _state instead of using a default value.
    SignalSourcePort(const std::string &_name, PortID _id,
                     const State &init_state)
        : SignalSourcePort(_name, _id)
    {
        _state = init_state;
    }

    // if bypass_on_change is specified true, it will not call the _onChange
    // function. Only _state will be updated if needed.
    void
    set(const State &new_state, const bool bypass_on_change = false)
    {
        _state = new_state;
        sink->set(new_state, bypass_on_change);
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

template <typename State>
void SignalSinkPort<State, std::enable_if_t<std::is_integral_v<State>>>::bind(
    Port &peer) {
    if (_source = dynamic_cast<SignalSourcePort<State> *>(&peer);
        _source != nullptr) {
        _state = _source->state();
    }
    if (_number_source = dynamic_cast<SignalSourcePort<int64_t> *>(&peer);
        _number_source != nullptr) {
        _state = _number_source->state();
    }
    Port::bind(peer);
    // The state of sink has to match the state of source.
    fatal_if(!_source && !_number_source, "Attempt to bind signal pin %s to "
            "incompatible pin %s", name(), peer.name());
}

}  // namespace gem5

#endif  //__SIM_SIGNAL_HH__
