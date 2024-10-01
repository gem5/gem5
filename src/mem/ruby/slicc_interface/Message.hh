/*
 * Copyright (c) 2021 ARM Limited
 * All rights reserved.
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
#include <memory>
#include <stack>

#include "mem/packet.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/common/WriteMask.hh"
#include "mem/ruby/protocol/MessageSizeType.hh"

namespace gem5
{

namespace ruby
{

class Message;
typedef std::shared_ptr<Message> MsgPtr;

class Message
{
  public:
    Message(Tick curTime, int block_size)
        : m_block_size(block_size),
          m_time(curTime),
          m_LastEnqueueTime(curTime),
          m_DelayedTicks(0), m_msg_counter(0)
    { }

    Message(const Message &other) = default;

    virtual ~Message() { }

    virtual MsgPtr clone() const = 0;
    virtual void print(std::ostream& out) const = 0;

    virtual const MessageSizeType& getMessageSize() const
    { panic("MessageSizeType() called on wrong message!"); }
    virtual MessageSizeType& getMessageSize()
    { panic("MessageSizeType() called on wrong message!"); }

    /**
     * The two functions below are used for reading / writing the message
     * functionally. The methods return true if the address in the packet
     * matches the address / address range in the message. Each message
     * class that can be potentially searched for the address needs to
     * implement these methods.
     */
    virtual bool functionalRead(Packet *pkt)
    { panic("functionalRead(Packet) not implemented"); }
    virtual bool functionalRead(Packet *pkt, WriteMask &mask)
    { panic("functionalRead(Packet,WriteMask) not implemented"); }
    virtual bool functionalWrite(Packet *pkt)
    { panic("functionalWrite(Packet) not implemented"); }

    //! Update the delay this message has experienced so far.
    void updateDelayedTicks(Tick curTime)
    {
        assert(m_LastEnqueueTime <= curTime);
        Tick delta = curTime - m_LastEnqueueTime;
        m_DelayedTicks += delta;
    }
    Tick getDelayedTicks() const {return m_DelayedTicks;}

    void setLastEnqueueTime(const Tick& time) { m_LastEnqueueTime = time; }
    Tick getLastEnqueueTime() const {return m_LastEnqueueTime;}

    Tick getTime() const { return m_time; }
    void setMsgCounter(uint64_t c) { m_msg_counter = c; }
    uint64_t getMsgCounter() const { return m_msg_counter; }

    // Functions related to network traversal
    virtual const NetDest& getDestination() const
    { panic("getDestination() called on wrong message!"); }
    virtual NetDest& getDestination()
    { panic("getDestination() called on wrong message!"); }

    int getIncomingLink() const { return incoming_link; }
    void setIncomingLink(int link) { incoming_link = link; }
    int getVnet() const { return vnet; }
    void setVnet(int net) { vnet = net; }

  protected:
    int m_block_size = 0;

  private:
    Tick m_time;
    Tick m_LastEnqueueTime; // my last enqueue time
    Tick m_DelayedTicks; // my delayed cycles
    uint64_t m_msg_counter; // FIXME, should this be a 64-bit value?

    // Variables for required network traversal
    int incoming_link;
    int vnet;
};

inline bool
operator>(const MsgPtr &lhs, const MsgPtr &rhs)
{
    const Message *l = lhs.get();
    const Message *r = rhs.get();

    if (l->getLastEnqueueTime() == r->getLastEnqueueTime()) {
        return l->getMsgCounter() > r->getMsgCounter();
    }
    return l->getLastEnqueueTime() > r->getLastEnqueueTime();
}

inline std::ostream&
operator<<(std::ostream& out, const Message& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SLICC_INTERFACE_MESSAGE_HH__
