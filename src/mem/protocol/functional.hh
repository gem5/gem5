/*
 * Copyright (c) 2011-2012,2015,2017 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __MEM_GEM5_PROTOCOL_FUNCTIONAL_HH__
#define __MEM_GEM5_PROTOCOL_FUNCTIONAL_HH__

#include "mem/packet.hh"

namespace gem5
{

class FunctionalResponseProtocol;

class FunctionalRequestProtocol
{
    friend class FunctionalResponseProtocol;

  protected:
    /**
     * Send a functional request packet, where the data is instantly
     * updated everywhere in the memory system, without affecting the
     * current state of any block or moving the block.
     *
     * @param pkt Packet to send.
     */
    void send(FunctionalResponseProtocol *peer, PacketPtr pkt) const;

    /**
     * Receive a functional snoop request packet from the peer.
     */
    virtual void recvFunctionalSnoop(PacketPtr pkt) = 0;
};

class FunctionalResponseProtocol
{
    friend class FunctionalRequestProtocol;

  protected:
    /**
     * Send a functional snoop request packet, where the data is
     * instantly updated everywhere in the memory system, without
     * affecting the current state of any block or moving the block.
     *
     * @param pkt Snoop packet to send.
     */
    void sendSnoop(FunctionalRequestProtocol *peer, PacketPtr pkt) const;

    /**
     * Receive a functional request packet from the peer.
     */
    virtual void recvFunctional(PacketPtr pkt) = 0;
};

} // namespace gem5

#endif //__MEM_GEM5_PROTOCOL_FUNCTIONAL_HH__
