/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_MESSAGE_HH__
#define __MEM_RUBY_SLICC_INTERFACE_MESSAGE_HH__

#include <iostream>

#include "base/refcnt.hh"
#include "mem/packet.hh"

class Message;
typedef RefCountingPtr<Message> MsgPtr;

class Message : public RefCounted
{
  public:
    Message(Cycles curTime)
        : m_time(curTime),
          m_LastEnqueueTime(curTime),
          m_DelayedCycles(0)
    { }

    Message(const Message &other)
        : m_time(other.m_time),
          m_LastEnqueueTime(other.m_LastEnqueueTime),
          m_DelayedCycles(other.m_DelayedCycles)
    { }

    virtual ~Message() { }

    virtual Message* clone() const = 0;
    virtual void print(std::ostream& out) const = 0;
    virtual void setIncomingLink(int) {}
    virtual void setVnet(int) {}

    /**
     * The two functions below are used for reading / writing the message
     * functionally. The methods return true if the address in the packet
     * matches the address / address range in the message. Each message
     * class that can be potentially searched for the address needs to
     * implement these methods.
     */
    virtual bool functionalRead(Packet *pkt) = 0;
    //{ fatal("Read functional access not implemented!"); }
    virtual bool functionalWrite(Packet *pkt) = 0;
    //{ fatal("Write functional access not implemented!"); }

    void setDelayedCycles(const Cycles cycles) { m_DelayedCycles = cycles; }
    const Cycles getDelayedCycles() const {return m_DelayedCycles;}

    void setLastEnqueueTime(const Cycles& time) { m_LastEnqueueTime = time; }
    const Cycles getLastEnqueueTime() const {return m_LastEnqueueTime;}

    const Cycles& getTime() const { return m_time; }
    void setTime(const Cycles& new_time) { m_time = new_time; }

  private:
    Cycles m_time;
    Cycles m_LastEnqueueTime; // my last enqueue time
    Cycles m_DelayedCycles; // my delayed cycles
};

inline std::ostream&
operator<<(std::ostream& out, const Message& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SLICC_INTERFACE_MESSAGE_HH__
