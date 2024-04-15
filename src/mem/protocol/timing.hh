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

#ifndef __MEM_GEM5_PROTOCOL_TIMING_HH__
#define __MEM_GEM5_PROTOCOL_TIMING_HH__

#include "mem/packet.hh"

namespace gem5
{

class TimingResponseProtocol;

class TimingRequestProtocol
{
    friend class TimingResponseProtocol;

  protected:
    /**
     * Attempt to send a timing request to the peer by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvReqRetry at which point it can re-issue a
     * sendTimingReq.
     *
     * @param peer Peer to send packet to.
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
     */
    bool sendReq(TimingResponseProtocol *peer, PacketPtr pkt);

    /**
     * Check if the peer can handle a timing request.
     *
     * If the send cannot be handled at the moment, as indicated by
     * the return value, then the sender will receive a recvReqRetry
     * at which point it can re-issue a sendTimingReq.
     *
     * @param peer Peer to send packet to.
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
     */
    bool trySend(TimingResponseProtocol *peer, PacketPtr pkt) const;

    /**
     * Attempt to send a timing snoop response packet to it's peer
     * by calling its corresponding receive function. If the send
     * does not succeed, as indicated by the return value, then the
     * sender must wait for a recvRetrySnoop at which point it can
     * re-issue a sendTimingSnoopResp.
     *
     * @param pkt Packet to send.
     */
    bool sendSnoopResp(TimingResponseProtocol *peer, PacketPtr pkt);

    /**
     * Send a retry to the peer that previously attempted a
     * sendTimingResp to this protocol and failed.
     */
    void sendRetryResp(TimingResponseProtocol *peer);

    /**
     * Receive a timing response from the peer.
     */
    virtual bool recvTimingResp(PacketPtr pkt) = 0;

    /**
     * Receive a timing snoop request from the peer.
     */
    virtual void recvTimingSnoopReq(PacketPtr pkt) = 0;

    /**
     * Called by the peer if sendTimingReq was called on this peer (causing
     * recvTimingReq to be called on the peer) and was unsuccessful.
     */
    virtual void recvReqRetry() = 0;

    /**
     * Called by the peer if sendTimingSnoopResp was called on this
     * protocol (causing recvTimingSnoopResp to be called on the peer)
     * and was unsuccessful.
     */
    virtual void recvRetrySnoopResp() = 0;
};

class TimingResponseProtocol
{
    friend class TimingRequestProtocol;

  protected:
    /**
     * Attempt to send a timing response to the peer by calling
     * its corresponding receive function. If the send does not
     * succeed, as indicated by the return value, then the sender must
     * wait for a recvRespRetry at which point it can re-issue a
     * sendTimingResp.
     *
     * @param peer Peer to send the packet to.
     * @param pkt Packet to send.
     *
     * @return If the send was succesful or not.
     */
    bool sendResp(TimingRequestProtocol *peer, PacketPtr pkt);

    /**
     * Attempt to send a timing snoop request packet to the peer
     * by calling its corresponding receive function. Snoop requests
     * always succeed and hence no return value is needed.
     *
     * @param peer Peer to send the packet to.
     * @param pkt Packet to send.
     */
    void sendSnoopReq(TimingRequestProtocol *peer, PacketPtr pkt);

    /**
     * Send a retry to the peer that previously attempted a
     * sendTimingReq to this protocol and failed.
     */
    void sendRetryReq(TimingRequestProtocol *peer);

    /**
     * Send a retry to the peer that previously attempted a
     * sendTimingSnoopResp to this peer and failed.
     */
    void sendRetrySnoopResp(TimingRequestProtocol *peer);

    /**
     * Receive a timing request from the peer.
     */
    virtual bool recvTimingReq(PacketPtr pkt) = 0;

    /**
     * Availability request from the peer.
     */
    virtual bool tryTiming(PacketPtr pkt) = 0;

    /**
     * Receive a timing snoop response from the peer.
     */
    virtual bool recvTimingSnoopResp(PacketPtr pkt) = 0;

    /**
     * Called by the peer if sendTimingResp was called on this
     * protocol (causing recvTimingResp to be called on the peer)
     * and was unsuccessful.
     */
    virtual void recvRespRetry() = 0;
};

} // namespace gem5

#endif //__MEM_GEM5_PROTOCOL_TIMING_HH__
