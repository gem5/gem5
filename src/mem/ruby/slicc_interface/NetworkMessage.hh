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

#ifndef __MEM_RUBY_SLICC_INTERFACE_NETWORKMESSAGE_HH__
#define __MEM_RUBY_SLICC_INTERFACE_NETWORKMESSAGE_HH__

#include <iostream>

#include "mem/protocol/MessageSizeType.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/slicc_interface/Message.hh"

class NetworkMessage;
typedef RefCountingPtr<NetworkMessage> NetMsgPtr;

class NetworkMessage : public Message
{
  public:
    NetworkMessage(Tick curTime)
        : Message(curTime), m_internal_dest_valid(false)
    { }

    NetworkMessage(const NetworkMessage &other)
        : Message(other), m_internal_dest(other.m_internal_dest),
          m_internal_dest_valid(other.m_internal_dest_valid)
    { }

    virtual ~NetworkMessage() { }

    virtual const NetDest& getDestination() const = 0;
    virtual NetDest& getDestination() = 0;
    virtual const MessageSizeType& getMessageSize() const = 0;
    virtual MessageSizeType& getMessageSize() = 0;

    const NetDest&
    getInternalDestination() const
    {
        if (m_internal_dest_valid == false)
            return getDestination();

        return m_internal_dest;
    }

    NetDest&
    getInternalDestination()
    {
        if (m_internal_dest_valid == false) {
            m_internal_dest = getDestination();
            m_internal_dest_valid = true;
        }
        return m_internal_dest;
    }

    virtual void print(std::ostream& out) const = 0;

    int getIncomingLink() const { return incoming_link; }
    void setIncomingLink(int link) { incoming_link = link; }
    int getVnet() const { return vnet; }
    void setVnet(int net) { vnet = net; }

  private:
    NetDest m_internal_dest;
    bool m_internal_dest_valid;
    int incoming_link;
    int vnet;
};

inline std::ostream&
operator<<(std::ostream& out, const NetworkMessage& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SLICC_INTERFACE_NETWORKMESSAGE_HH__
