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

#ifndef __DEV_INTPIN_HH__
#define __DEV_INTPIN_HH__

#include <type_traits>

#include "sim/signal.hh"

namespace gem5
{

class IntSinkPinBase : public SignalSinkPort<bool>
{
  private:
    const int _number = 0;

  public:

    template <class Device>
    IntSinkPinBase(const std::string &_name, PortID _id, Device *dev,
            int num) :
        SignalSinkPort(_name, _id), _number(num)
    {
        onChange([dev, num](const bool &new_val) {
            if (new_val)
                dev->raiseInterruptPin(num);
            else
                dev->lowerInterruptPin(num);
        });
    }

    template <class Device>
    IntSinkPinBase(const std::string &_name, PortID _id, Device *dev) :
        IntSinkPinBase(_name, _id, dev, _id)
    {}

    IntSinkPinBase(const std::string &_name, PortID _id, int num) :
        SignalSinkPort(_name, _id), _number(num)
    {}

    IntSinkPinBase(const std::string &_name, PortID _id) :
        IntSinkPinBase(_name, _id, _id)
    {}

    int number() { return _number; }
};

template <class Compat>
using IntSinkPin = IntSinkPinBase;

class IntSourcePinBase : public SignalSourcePort<bool>
{
  public:
    template <class Device>
    IntSourcePinBase(const std::string &_name, PortID _id, Device *owner) :
        SignalSourcePort(_name, _id)
    {}

    IntSourcePinBase(const std::string &_name, PortID _id) :
        SignalSourcePort(_name, _id)
    {}

    void raise() { set(true); }
    void lower() { set(false); }
};

template <class Compat>
using IntSourcePin = IntSourcePinBase;

} // namespace gem5

#endif //__DEV_INTPIN_HH__
