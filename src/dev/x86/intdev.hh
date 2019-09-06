/*
 * Copyright (c) 2012 ARM Limited
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

#include <cassert>
#include <list>
#include <string>

#include "arch/x86/intmessage.hh"
#include "arch/x86/x86_traits.hh"
#include "mem/mport.hh"
#include "sim/sim_object.hh"

namespace X86ISA {

typedef std::list<int> ApicList;

class IntDevice
{
  protected:
    class IntSlavePort : public MessageSlavePort
    {
        IntDevice * device;

      public:
        IntSlavePort(const std::string& _name, SimObject* _parent,
                     IntDevice* dev) :
            MessageSlavePort(_name, _parent), device(dev)
        {
        }

        AddrRangeList getAddrRanges() const
        {
            return device->getIntAddrRange();
        }

        Tick recvMessage(PacketPtr pkt)
        {
            // @todo someone should pay for this
            pkt->headerDelay = pkt->payloadDelay = 0;
            return device->recvMessage(pkt);
        }
    };

    class IntMasterPort : public MessageMasterPort
    {
        IntDevice* device;
        Tick latency;
      public:
        IntMasterPort(const std::string& _name, SimObject* _parent,
                      IntDevice* dev, Tick _latency) :
            MessageMasterPort(_name, _parent), device(dev), latency(_latency)
        {
        }

        Tick recvResponse(PacketPtr pkt)
        {
            return device->recvResponse(pkt);
        }

        // This is x86 focused, so if this class becomes generic, this would
        // need to be moved into a subclass.
        void sendMessage(ApicList apics,
                TriggerIntMessage message, bool timing);
    };

    IntMasterPort intMasterPort;

  public:
    IntDevice(SimObject * parent, Tick latency = 0) :
        intMasterPort(parent->name() + ".int_master", parent, this, latency)
    {
    }

    virtual ~IntDevice()
    {}

    virtual void init();

    virtual Tick
    recvMessage(PacketPtr pkt)
    {
        panic("recvMessage not implemented.\n");
        return 0;
    }

    virtual Tick
    recvResponse(PacketPtr pkt)
    {
        panic("recvResponse not implemented.\n");
        return 0;
    }

    virtual AddrRangeList
    getIntAddrRange() const
    {
        panic("intAddrRange not implemented.\n");
    }
};

} // namespace X86ISA

#endif //__DEV_X86_INTDEV_HH__
