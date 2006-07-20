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
SimpleTimingPort::recvRetry()
{
    bool result = true;
    while (result && transmitList.size()) {
        result = Port::sendTiming(transmitList.front());
        if (result)
            transmitList.pop_front();
    }
   if (transmitList.size() == 0 && drainEvent) {
       drainEvent->process();
       drainEvent = NULL;
   }
}

void
SimpleTimingPort::SendEvent::process()
{
    port->outTiming--;
    assert(port->outTiming >= 0);
    if (port->Port::sendTiming(packet))
       if (port->transmitList.size() == 0 && port->drainEvent) {
           port->drainEvent->process();
           port->drainEvent = NULL;
       }
       return;

    port->transmitList.push_back(packet);
}

void
SimpleTimingPort::resendNacked(Packet *pkt) {
    pkt->reinitNacked();
    if (transmitList.size()) {
         transmitList.push_front(pkt);
    } else {
        if (!Port::sendTiming(pkt))
            transmitList.push_front(pkt);
    }
};


unsigned int
SimpleTimingPort::drain(Event *de)
{
    if (outTiming == 0 && transmitList.size() == 0)
        return 0;
    drainEvent = de;
    return 1;
}
