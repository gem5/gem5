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
 */

#include "dev/io_device.hh"
#include "sim/builder.hh"


PioPort::PioPort(PioDevice *dev, Platform *p)
        : device(dev), platform(p)
{ }


Tick
PioPort::recvAtomic(Packet &pkt)
{
    return device->recvAtomic(pkt);
}

void
PioPort::recvFunctional(Packet &pkt)
{
    device->recvAtomic(pkt);
}

void
PioPort::getDeviceAddressRanges(AddrRangeList &resp, AddrRangeList &snoop)
{
    snoop.clear();
    device->addressRanges(resp);
}


Packet *
PioPort::recvRetry()
{
    Packet* pkt = transmitList.front();
    transmitList.pop_front();
    return pkt;
}


void
PioPort::SendEvent::process()
{
    if (port->Port::sendTiming(packet) == Success)
        return;

    port->transmitList.push_back(&packet);
}


bool
PioPort::recvTiming(Packet &pkt)
{
    device->recvAtomic(pkt);
    sendTiming(pkt, pkt.time-pkt.req->getTime());
    return Success;
}

PioDevice::~PioDevice()
{
    if (pioPort)
        delete pioPort;
}

void
PioDevice::init()
{
    if (!pioPort)
        panic("Pio port not connected to anything!");
    pioPort->sendStatusChange(Port::RangeChange);
}

void
BasicPioDevice::addressRanges(AddrRangeList &range_list)
{
    assert(pioSize != 0);
    range_list.clear();
    range_list.push_back(RangeSize(pioAddr, pioSize));
}


DmaPort::DmaPort(DmaDevice *dev, Platform *p)
        : device(dev), platform(p), pendingCount(0)
{ }

bool
DmaPort::recvTiming(Packet &pkt)
{
    if (pkt.senderState) {
        DmaReqState *state;
        state = (DmaReqState*)pkt.senderState;
        state->completionEvent->schedule(pkt.time - pkt.req->getTime());
        delete pkt.req;
        delete &pkt;
    }  else {
        delete pkt.req;
        delete &pkt;
    }

    return Success;
}

DmaDevice::DmaDevice(Params *p)
    : PioDevice(p), dmaPort(NULL)
{ }

void
DmaPort::SendEvent::process()
{
    if (port->Port::sendTiming(packet) == Success)
        return;

    port->transmitList.push_back(&packet);
}

Packet *
DmaPort::recvRetry()
{
    Packet* pkt = transmitList.front();
    transmitList.pop_front();
    return pkt;
}
void
DmaPort::dmaAction(Command cmd, Addr addr, int size, Event *event,
        uint8_t *data)
{

    assert(event);

    int prevSize = 0;
    Packet basePkt;
    Request baseReq(false);

    basePkt.flags = 0;
    basePkt.coherence = NULL;
    basePkt.senderState = NULL;
    basePkt.dest = Packet::Broadcast;
    basePkt.cmd = cmd;
    basePkt.result = Unknown;
    basePkt.req = NULL;
//    baseReq.nicReq = true;
    baseReq.setTime(curTick);

    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
            Packet *pkt = new Packet(basePkt);
            Request *req = new Request(baseReq);
            pkt->addr = gen.addr();
            pkt->size = gen.size();
            pkt->req = req;
            pkt->req->setPaddr(pkt->addr);
            pkt->req->setSize(pkt->size);
            // Increment the data pointer on a write
            if (data)
                pkt->dataStatic(data + prevSize) ;
            prevSize += pkt->size;
            // Set the last bit of the dma as the final packet for this dma
            // and set it's completion event.
            if (prevSize == size) {
                DmaReqState *state = new DmaReqState(event, true);

                pkt->senderState = (void*)state;
            }
            assert(pendingCount >= 0);
            pendingCount++;
            sendDma(pkt);
    }
    // since this isn't getting used and we want a check to make sure that all
    // packets had data in them at some point.
    basePkt.dataStatic((uint8_t*)NULL);
}


void
DmaPort::sendDma(Packet *pkt)
{
   // some kind of selction between access methods
   // more work is going to have to be done to make
   // switching actually work
  /* MemState state = device->platform->system->memState;

   if (state == Timing) {
       if (sendTiming(pkt) == Failure)
           transmitList.push_back(&packet);
    } else if (state == Atomic) {*/
       sendAtomic(*pkt);
       if (pkt->senderState) {
           DmaReqState *state = (DmaReqState*)pkt->senderState;
           state->completionEvent->schedule(curTick + (pkt->time - pkt->req->getTime()) +1);
       }
       pendingCount--;
       assert(pendingCount >= 0);
       delete pkt->req;
       delete pkt;

/*   } else if (state == Functional) {
       sendFunctional(pkt);
       // Is this correct???
       completionEvent->schedule(pkt.req->responseTime - pkt.req->requestTime);
       completionEvent == NULL;
   } else
       panic("Unknown memory command state.");
  */
}

DmaDevice::~DmaDevice()
{
    if (dmaPort)
        delete dmaPort;
}


