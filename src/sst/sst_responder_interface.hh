// Copyright (c) 2021 The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef __SST_RESPONDER_INTERFACE_HH__
#define __SST_RESPONDER_INTERFACE_HH__

#include <string>

#include "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaamem/port.hh"

/**
 *  SSTResponderInterface provides an interface specified gem5's expectations
 * on the functionality of an SST Responder. This interfaces expects
 * SST Responder to be able to handle gem5 packet on recvTimingReq(),
 * recvRespRetry(), and recvFunctional().
 */

namespace gem5
{

class SSTResponderInterface
{
  public:

    SSTResponderInterface();
    virtual ~SSTResponderInterface() {};

    // This function is called when OutgoingRequestBridge wants to forward
    // a gem5 request to SST, i.e. when OutgoingRequestPort::recvTimingReq()
    // is called.
    virtual bool handleRecvTimingReq(PacketPtr pkt) = 0;

    // This function is called when OutogingRequestPort::recvRespRetry() is
    // called.
    virtual void handleRecvRespRetry() = 0;

    // This function is called when OutgoingRequestBridge wants to send a
    // non-timing request. This function should only be called during the
    // SST construction phrase, i.e. not at simulation time.
    virtual void handleRecvFunctional(PacketPtr pkt) = 0;
};

} // namespace gem5

#endif // __SST_RESPONDER_INTERFACE_HH__
