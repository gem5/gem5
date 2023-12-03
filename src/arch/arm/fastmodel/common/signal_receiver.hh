/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_COMMON_SIGNAL_RECEIVER_HH__
#define __ARCH_ARM_FASTMODEL_COMMON_SIGNAL_RECEIVER_HH__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <amba_pv.h>
#pragma GCC diagnostic pop

#include <functional>
#include <vector>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/types.hh"
#include "dev/intpin.hh"

namespace gem5
{

namespace fastmodel
{

class SignalReceiver : public amba_pv::signal_slave_base<bool>
{
  public:
    typedef std::function<void(bool)> OnChangeFunc;

  private:
    bool _state;
    OnChangeFunc _onChange;

  public:
    amba_pv::signal_slave_export<bool> signal_in;

    SignalReceiver(const std::string &name, OnChangeFunc on_change = nullptr)
        : SignalReceiver(name.c_str(), on_change)
    {}

    SignalReceiver(const char *name, OnChangeFunc on_change = nullptr)
        : amba_pv::signal_slave_base<bool>(name),
          _state(false),
          _onChange(on_change)
    {
        signal_in.bind(*this);
    }

    void
    onChange(OnChangeFunc func)
    {
        _onChange = func;
    }

    void
    set_state(int export_id, const bool &new_state) override
    {
        if (new_state == _state)
            return;

        _state = new_state;
        _onChange(_state);
    }
};

class SignalReceiverInt : public SignalReceiver
{
  public:
    using IntPin = SignalSourcePort<bool>;

    explicit SignalReceiverInt(const std::string &name) : SignalReceiver(name)
    {
        onChange([this](bool status) {
            for (auto &signal : signalOut) {
                if (signal && signal->isConnected())
                    signal->set(status);
            }
        });
    }

    IntPin &
    getSignalOut(int idx)
    {
        if (signalOut.size() <= idx) {
            signalOut.resize(idx + 1);
        }
        if (!signalOut[idx]) {
            signalOut[idx] = std::make_unique<IntPin>(
                csprintf("%s.signalOut[%d]", get_name(), idx), idx);
        }
        return *signalOut[idx];
    }

  private:
    std::vector<std::unique_ptr<IntPin>> signalOut;
};

} // namespace fastmodel
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_COMMON_SIGNAL_RECEIVER_HH__
