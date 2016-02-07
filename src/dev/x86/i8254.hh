/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 *
 * Authors: Gabe Black
 */

#ifndef __DEV_X86_I8254_HH__
#define __DEV_X86_I8254_HH__

#include "dev/intel_8254_timer.hh"
#include "dev/io_device.hh"
#include "params/I8254.hh"

namespace X86ISA
{

class IntSourcePin;

class I8254 : public BasicPioDevice
{
  protected:
    Tick latency;
    class X86Intel8254Timer : public Intel8254Timer
    {
      protected:
        I8254 * parent;

        void
        counterInterrupt(unsigned int num)
        {
            parent->counterInterrupt(num);
        }

      public:
        X86Intel8254Timer(const std::string &name, I8254 * _parent) :
            Intel8254Timer(_parent, name), parent(_parent)
        {}
    };


    X86Intel8254Timer pit;

    IntSourcePin *intPin;

    void counterInterrupt(unsigned int num);

  public:
    typedef I8254Params Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    I8254(Params *p) : BasicPioDevice(p, 4), latency(p->pio_latency),
            pit(p->name, this), intPin(p->int_pin)
    {
    }
    Tick read(PacketPtr pkt) override;

    Tick write(PacketPtr pkt) override;

    bool
    outputHigh(unsigned int num)
    {
        return pit.outputHigh(num);
    }

    uint8_t
    readCounter(unsigned int num)
    {
        return pit.readCounter(num);
    }

    void
    writeCounter(unsigned int num, const uint8_t data)
    {
        pit.writeCounter(num, data);
    }

    void
    writeControl(uint8_t val)
    {
        pit.writeControl(val);
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void startup() override;

};

} // namespace X86ISA

#endif //__DEV_X86_SOUTH_BRIDGE_I8254_HH__
