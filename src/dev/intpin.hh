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

#include "sim/port.hh"

namespace gem5
{

class IntSourcePinBase;

class IntSinkPinBase : public Port
{
  protected:
    friend IntSourcePinBase;

    IntSourcePinBase *source = nullptr;

    int _number = 0;
    bool _state = false;

    IntSinkPinBase(const std::string &_name, PortID _id, int num) :
        Port(_name, _id), _number(num)
    {}

    virtual void raiseOnDevice() = 0;
    virtual void lowerOnDevice() = 0;

    void
    raise()
    {
        _state = true;
        raiseOnDevice();
    }

    void
    lower()
    {
        _state = false;
        lowerOnDevice();
    }

  public:
    int number() { return _number; }
    bool state() { return _state; }

    void bind(Port &peer) override;
    void unbind() override;
};

template <class Device>
class IntSinkPin : public IntSinkPinBase
{
  private:
    Device *device = nullptr;

    void raiseOnDevice() override { device->raiseInterruptPin(number()); }
    void lowerOnDevice() override { device->lowerInterruptPin(number()); }

  public:
    IntSinkPin(const std::string &_name, PortID _id, Device *dev, int num) :
        IntSinkPinBase(_name, _id, num), device(dev) {}
    IntSinkPin(const std::string &_name, PortID _id, Device *dev) :
        IntSinkPin(_name, _id, dev, _id) {}
};

class IntSourcePinBase : public Port
{
  private:
    IntSinkPinBase *sink = nullptr;

  public:
    IntSourcePinBase(const std::string &_name, PortID _id):
        Port(_name, _id)
    {}

    void raise() { sink->raise(); }
    void lower() { sink->lower(); }

    void bind(Port &peer) override;
    void unbind() override;
};

template <class Device>
class IntSourcePin : public IntSourcePinBase
{
  public:
    IntSourcePin(const std::string &_name, PortID _id, Device *owner) :
        IntSourcePinBase(_name, _id)
    {}
};

} // namespace gem5

#endif //__DEV_INTPIN_HH__
