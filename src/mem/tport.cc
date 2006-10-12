/*
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
 *
 * Authors: Ali Saidi
 */

#include "mem/tport.hh"

void
SimpleTimingPort::recvFunctional(Packet *pkt)
{
    //First check queued events
    std::list<Packet *>::iterator i = transmitList.begin();
    std::list<Packet *>::iterator end = transmitList.end();
    for (; i != end; ++i) {
        Packet * target = *i;
        // If the target contains data, and it overlaps the
        // probed request, need to update data
        if (target->intersect(pkt)) {
            uint8_t* pkt_data;
            uint8_t* write_data;
            int data_size;
            if (target->getAddr() < pkt->getAddr()) {
                int offset = pkt->getAddr() - target->getAddr();
                            pkt_data = pkt->getPtr<uint8_t>();
                            write_data = target->getPtr<uint8_t>() + offset;
                            data_size = target->getSize() - offset;
                            assert(data_size > 0);
                            if (data_size > pkt->getSize())
                                data_size = pkt->getSize();
            } else {
                int offset = target->getAddr() - pkt->getAddr();
                pkt_data = pkt->getPtr<uint8_t>() + offset;
                write_data = target->getPtr<uint8_t>();
                data_size = pkt->getSize() - offset;
                assert(data_size > pkt->getSize());
                if (data_size > target->getSize())
                    data_size = target->getSize();
            }

            if (pkt->isWrite()) {
                memcpy(pkt_data, write_data, data_size);
            } else {
                memcpy(write_data, pkt_data, data_size);
            }
        }
    }
    //Then just do an atomic access and throw away the returned latency
    recvAtomic(pkt);
}

bool
SimpleTimingPort::recvTiming(Packet *pkt)
{
    // If the device is only a slave, it should only be sending
    // responses, which should never get nacked.  There used to be
    // code to hanldle nacks here, but I'm pretty sure it didn't work
    // correctly with the drain code, so that would need to be fixed
    // if we ever added it back.
    assert(pkt->result != Packet::Nacked);
    Tick latency = recvAtomic(pkt);
    // turn packet around to go back to requester if response expected
    if (pkt->needsResponse()) {
        pkt->makeTimingResponse();
        sendTimingLater(pkt, latency);
    }
    return true;
}

void
SimpleTimingPort::recvRetry()
{
    assert(outTiming > 0);
    assert(!transmitList.empty());
    if (sendTiming(transmitList.front())) {
        transmitList.pop_front();
        outTiming--;
        DPRINTF(Bus, "No Longer waiting on retry\n");
        if (!transmitList.empty())
            sendTimingLater(transmitList.front(), 1);
    }

    if (transmitList.empty() && drainEvent) {
        drainEvent->process();
        drainEvent = NULL;
    }
}

void
SimpleTimingPort::SendEvent::process()
{
    assert(port->outTiming > 0);
    if (!port->transmitList.empty() && port->transmitList.front() != packet) {
        //We are not the head of the list
        port->transmitList.push_back(packet);
    } else if (port->sendTiming(packet)) {
        // send successful
        if (port->transmitList.size()) {
            port->transmitList.pop_front();
            port->outTiming--;
           if (!port->transmitList.empty())
                port->sendTimingLater(port->transmitList.front(), 1);
        }
        if (port->transmitList.empty() && port->drainEvent) {
            port->drainEvent->process();
            port->drainEvent = NULL;
        }
    } else {
        // send unsuccessful (due to flow control).  Will get retry
        // callback later; save for then if not already
        DPRINTF(Bus, "Waiting on retry\n");
        if (!(port->transmitList.front() == packet))
            port->transmitList.push_back(packet);
    }
}


unsigned int
SimpleTimingPort::drain(Event *de)
{
    if (outTiming == 0 && transmitList.size() == 0)
        return 0;
    drainEvent = de;
    return 1;
}
