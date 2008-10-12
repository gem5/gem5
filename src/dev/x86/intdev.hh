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

#ifndef __DEV_X86_INTDEV_HH__
#define __DEV_X86_INTDEV_HH__

#include <assert.h>
#include <string>

#include "arch/x86/x86_traits.hh"
#include "mem/mem_object.hh"
#include "mem/mport.hh"
#include "sim/sim_object.hh"
#include "params/X86IntPin.hh"

namespace X86ISA {

class IntDev
{
  protected:
    class IntPort : public MessagePort
    {
        IntDev * device;
        Tick latency;
        Addr intAddr;
      public:
        IntPort(const std::string &_name, MemObject * _parent,
                IntDev *dev, Tick _latency) :
            MessagePort(_name, _parent), device(dev), latency(_latency)
        {
        }

        void getDeviceAddressRanges(AddrRangeList &resp, bool &snoop)
        {
            snoop = false;
            device->getIntAddrRange(resp);
        }

        Tick recvMessage(PacketPtr pkt)
        {
            return device->recvMessage(pkt);
        }

        void recvStatusChange(Status status)
        {
            if (status == RangeChange) {
                sendStatusChange(Port::RangeChange);
            }
        }

    };

    IntPort * intPort;

  public:
    IntDev(MemObject * parent, Tick latency = 0)
    {
        if (parent != NULL) {
            intPort = new IntPort(parent->name() + ".int_port",
                    parent, this, latency);
        } else {
            intPort = NULL;
        }
    }

    virtual ~IntDev()
    {}

    virtual void
    signalInterrupt(int line)
    {
        panic("signalInterrupt not implemented.\n");
    }

    virtual Tick
    recvMessage(PacketPtr pkt)
    {
        panic("recvMessage not implemented.\n");
        return 0;
    }

    virtual void
    getIntAddrRange(AddrRangeList &range_list)
    {
        panic("intAddrRange not implemented.\n");
    }
};

class IntPin : public SimObject
{
  protected:
    IntDev * device;
    int line;

  public:
    typedef X86IntPinParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    IntPin(Params *p) : SimObject(p),
            device(dynamic_cast<IntDev *>(p->device)), line(p->line)
    {
        assert(device);
    }

    void
    signalInterrupt()
    {
        device->signalInterrupt(line);
    }
};

}; // namespace X86ISA

#endif //__DEV_X86_INTDEV_HH__
