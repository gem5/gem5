/*
 * Copyright (c) 2012,2015,2017 ARM Limited
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
 * Authors: Steve Reinhardt
 *          Andreas Hansson
 *          William Wang
 */

#include "mem/protocol/timing.hh"

/* The request protocol. */

bool
TimingRequestProtocol::sendReq(TimingResponseProtocol *peer, PacketPtr pkt)
{
    assert(pkt->isRequest());
    return peer->recvTimingReq(pkt);
}

bool
TimingRequestProtocol::trySend(
        TimingResponseProtocol *peer, PacketPtr pkt) const
{
  assert(pkt->isRequest());
  return peer->tryTiming(pkt);
}

bool
TimingRequestProtocol::sendSnoopResp(
        TimingResponseProtocol *peer, PacketPtr pkt)
{
    assert(pkt->isResponse());
    return peer->recvTimingSnoopResp(pkt);
}

void
TimingRequestProtocol::sendRetryResp(TimingResponseProtocol *peer)
{
    peer->recvRespRetry();
}

/* The response protocol. */

bool
TimingResponseProtocol::sendResp(TimingRequestProtocol *peer, PacketPtr pkt)
{
    assert(pkt->isResponse());
    return peer->recvTimingResp(pkt);
}

void
TimingResponseProtocol::sendSnoopReq(
        TimingRequestProtocol *peer, PacketPtr pkt)
{
    assert(pkt->isRequest());
    peer->recvTimingSnoopReq(pkt);
}

void
TimingResponseProtocol::sendRetryReq(TimingRequestProtocol *peer)
{
    peer->recvReqRetry();
}

void
TimingResponseProtocol::sendRetrySnoopResp(TimingRequestProtocol *peer)
{
    peer->recvRetrySnoopResp();
}
