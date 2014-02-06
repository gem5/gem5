/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2012 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_SYSTEM_ABSTRACT_MEMORY_CONTROL_HH__
#define __MEM_RUBY_SYSTEM_ABSTRACT_MEMORY_CONTROL_HH__

#include <iostream>
#include <list>
#include <string>

#include "mem/protocol/MemoryControlRequestType.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/system/MemoryNode.hh"
#include "params/MemoryControl.hh"
#include "sim/clocked_object.hh"

//////////////////////////////////////////////////////////////////////////////

class MemoryControl : public ClockedObject, public Consumer
{
  public:
    typedef MemoryControlParams Params;
    const Params *params() const
    { return dynamic_cast<const Params *>(_params); }

    MemoryControl(const Params *p);
    virtual void init() = 0;
    virtual void reset() = 0;

    ~MemoryControl();

    virtual void wakeup() = 0;

    virtual void setConsumer(Consumer* consumer_ptr) = 0;
    virtual Consumer* getConsumer() = 0;
    virtual void setClockObj(ClockedObject* consumer_ptr) {}

    virtual void setDescription(const std::string& name) = 0;
    virtual std::string getDescription() = 0;

    // Called from the directory:
    virtual void enqueue(const MsgPtr& message, Cycles latency) = 0;
    virtual void enqueueMemRef(MemoryNode *memRef) = 0;
    virtual void dequeue() = 0;
    virtual const Message* peek() = 0;
    virtual MemoryNode *peekNode() = 0;
    virtual bool isReady() = 0;
    virtual bool areNSlotsAvailable(int n) = 0;  // infinite queue length

    virtual void print(std::ostream& out) const = 0;
    virtual void regStats() {};

    virtual const int getChannel(const physical_address_t addr) const = 0;
    virtual const int getBank(const physical_address_t addr) const = 0;
    virtual const int getRank(const physical_address_t addr) const = 0;
    virtual const int getRow(const physical_address_t addr) const = 0;

    //added by SS
    virtual int getBanksPerRank() = 0;
    virtual int getRanksPerDimm() = 0;
    virtual int getDimmsPerChannel() = 0;

    virtual void recordRequestType(MemoryControlRequestType requestType);

    virtual bool functionalReadBuffers(Packet *pkt)
    { fatal("Functional read access not implemented!");}
    virtual uint32_t functionalWriteBuffers(Packet *pkt)
    { fatal("Functional read access not implemented!");}

protected:
    class MemCntrlEvent : public Event
    {
      public:
        MemCntrlEvent(MemoryControl* _mem_cntrl)
        {
            mem_cntrl = _mem_cntrl;
        }
      private:
        void process() { mem_cntrl->wakeup(); }

        MemoryControl* mem_cntrl;
    };

    MemCntrlEvent m_event;
};

#endif //   __MEM_RUBY_SYSTEM_ABSTRACT_MEMORY_CONTROL_HH__
