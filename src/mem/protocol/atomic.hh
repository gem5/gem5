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
 *
 * Authors: Ron Dreslinski
 *          Andreas Hansson
 *          William Wang
 */

#ifndef __MEM_GEM5_PROTOCOL_ATOMIC_HH__
#define __MEM_GEM5_PROTOCOL_ATOMIC_HH__

#include "mem/backdoor.hh"
#include "mem/packet.hh"

class AtomicResponseProtocol;

class AtomicRequestProtocol
{
    friend class AtomicResponseProtocol;

  protected:
    /**
     * Send an atomic request packet, where the data is moved and the
     * state is updated in zero time, without interleaving with other
     * memory accesses.
     *
     * @param peer Peer to send packet to.
     * @param pkt Packet to send.
     *
     * @return Estimated latency of access.
     */
    Tick send(AtomicResponseProtocol *peer, PacketPtr pkt);

    /**
     * Send an atomic request packet like above, but also request a backdoor
     * to the data being accessed.
     *
     * @param peer Peer to send packet to.
     * @param pkt Packet to send.
     * @param backdoor Can be set to a back door pointer by the target to let
     *        caller have direct access to the requested data.
     *
     * @return Estimated latency of access.
     */
    Tick sendBackdoor(AtomicResponseProtocol *peer, PacketPtr pkt,
                      MemBackdoorPtr &backdoor);

    /**
     * Receive an atomic snoop request packet from our peer.
     */
    virtual Tick recvAtomicSnoop(PacketPtr pkt) = 0;
};

class AtomicResponseProtocol
{
    friend class AtomicRequestProtocol;

  protected:
    /**
     * Send an atomic snoop request packet, where the data is moved
     * and the state is updated in zero time, without interleaving
     * with other memory accesses.
     *
     * @param peer Peer to send packet to.
     * @param pkt Snoop packet to send.
     *
     * @return Estimated latency of access.
     */
    Tick sendSnoop(AtomicRequestProtocol *peer, PacketPtr pkt);

    /**
     * Receive an atomic request packet from the peer.
     */
    virtual Tick recvAtomic(PacketPtr pkt) = 0;

    /**
     * Receive an atomic request packet from the peer, and optionally
     * provide a backdoor to the data being accessed.
     */
    virtual Tick recvAtomicBackdoor(
            PacketPtr pkt, MemBackdoorPtr &backdoor) = 0;
};

#endif //__MEM_GEM5_PROTOCOL_ATOMIC_HH__
