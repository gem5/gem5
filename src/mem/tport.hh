/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __MEM_TPORT_HH__
#define __MEM_TPORT_HH__

/**
 * @file
 *
 * Declaration of SimpleTimingPort.
 */

#include "mem/qport.hh"

namespace gem5
{

class SimObject;

/**
 * The simple timing port uses a queued port to implement
 * recvFunctional and recvTimingReq through recvAtomic. It is always a
 * response port.
 */
class SimpleTimingPort : public QueuedResponsePort
{
  private:
    /**
     * The packet queue used to store outgoing responses. Note that
     * the queue is made private and that we avoid overloading the
     * name used in the QueuedResponsePort. Access is provided through
     * the queue reference in the base class.
     */
    RespPacketQueue queueImpl;

  protected:
    /** Implemented using recvAtomic(). */
    void recvFunctional(PacketPtr pkt);

    /** Implemented using recvAtomic(). */
    bool recvTimingReq(PacketPtr pkt);

    virtual Tick recvAtomic(PacketPtr pkt) = 0;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

  public:
    /**
     * Create a new SimpleTimingPort that relies on a packet queue to
     * hold responses, and implements recvTimingReq and recvFunctional
     * through calls to recvAtomic. Once a request arrives, it is
     * passed to recvAtomic, and in the case of a timing access any
     * response is scheduled to be sent after the delay of the atomic
     * operation.
     *
     * @param name port name
     * @param owner structural owner
     */
    SimpleTimingPort(const std::string &name, SimObject *owner);

    virtual ~SimpleTimingPort() {}
};

} // namespace gem5

#endif // __MEM_TPORT_HH__
