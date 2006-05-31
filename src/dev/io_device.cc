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

#include "base/trace.hh"
#include "dev/io_device.hh"
#include "sim/builder.hh"


PioPort::PioPort(PioDevice *dev, Platform *p)
    : Port(dev->name() + "-pioport"), device(dev), platform(p)
{ }


Tick
PioPort::recvAtomic(Packet *pkt)
{
    return device->recvAtomic(pkt);
}

void
PioPort::recvFunctional(Packet *pkt)
{
    device->recvAtomic(pkt);
}

void
PioPort::getDeviceAddressRanges(AddrRangeList &resp, AddrRangeList &snoop)
{
    snoop.clear();
    device->addressRanges(resp);
}


void
PioPort::recvRetry()
{
    Packet* pkt = transmitList.front();
    if (Port::sendTiming(pkt)) {
        transmitList.pop_front();
    }
}


void
PioPort::SendEvent::process()
{
    if (port->Port::sendTiming(packet))
        return;

    port->transmitList.push_back(packet);
}



bool
PioPort::recvTiming(Packet *pkt)
{
    Tick latency = device->recvAtomic(pkt);
    // turn packet around to go back to requester
    pkt->makeTimingResponse();
    sendTiming(pkt, latency);
    return true;
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
    : Port(dev->name() + "-dmaport"), device(dev), platform(p), pendingCount(0)
{ }

bool
DmaPort::recvTiming(Packet *pkt)
{
    if (pkt->senderState) {
        DmaReqState *state;
        DPRINTF(DMA, "Received response Packet %#x with senderState: %#x\n",
               pkt, pkt->senderState);
        state = dynamic_cast<DmaReqState*>(pkt->senderState);
        assert(state);
        state->completionEvent->process();
        delete pkt->req;
        delete pkt;
    }  else {
        DPRINTF(DMA, "Received response Packet %#x with no senderState\n", pkt);
        delete pkt->req;
        delete pkt;
    }

    return true;
}

DmaDevice::DmaDevice(Params *p)
    : PioDevice(p), dmaPort(NULL)
{ }

void
DmaPort::recvRetry()
{
    Packet* pkt = transmitList.front();
    bool result = true;
    while (result && transmitList.size()) {
        DPRINTF(DMA, "Retry on  Packet %#x with senderState: %#x\n",
                   pkt, pkt->senderState);
        result = sendTiming(pkt);
        if (result) {
            DPRINTF(DMA, "-- Done\n");
            transmitList.pop_front();
            pendingCount--;
            assert(pendingCount >= 0);
        } else {
            DPRINTF(DMA, "-- Failed, queued\n");
        }
    }
}


void
DmaPort::dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data)
{
    assert(event);

    int prevSize = 0;

    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
            Request *req = new Request(gen.addr(), gen.size(), 0);
            Packet *pkt = new Packet(req, cmd, Packet::Broadcast);

            // Increment the data pointer on a write
            if (data)
                pkt->dataStatic(data + prevSize);

            prevSize += gen.size();

            // Set the last bit of the dma as the final packet for this dma
            // and set it's completion event.
            if (prevSize == size) {
                pkt->senderState = new DmaReqState(event, true);
            }
            assert(pendingCount >= 0);
            pendingCount++;
            sendDma(pkt);
    }
}


void
DmaPort::sendDma(Packet *pkt)
{
   // some kind of selction between access methods
   // more work is going to have to be done to make
   // switching actually work
  /* MemState state = device->platform->system->memState;

   if (state == Timing) {  */
       DPRINTF(DMA, "Attempting to send Packet %#x with senderState: %#x\n",
               pkt, pkt->senderState);
       if (transmitList.size() || !sendTiming(pkt)) {
           transmitList.push_back(pkt);
           DPRINTF(DMA, "-- Failed: queued\n");
       } else {
           DPRINTF(DMA, "-- Done\n");
           pendingCount--;
           assert(pendingCount >= 0);
       }
  /*  } else if (state == Atomic) {
       sendAtomic(pkt);
       if (pkt->senderState) {
           DmaReqState *state = dynamic_cast<DmaReqState*>(pkt->senderState);
           assert(state);
           state->completionEvent->schedule(curTick + (pkt->time - pkt->req->getTime()) +1);
       }
       pendingCount--;
       assert(pendingCount >= 0);
       delete pkt->req;
       delete pkt;

   } else if (state == Functional) {
       sendFunctional(pkt);
       // Is this correct???
       completionEvent->schedule(pkt->req->responseTime - pkt->req->requestTime);
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


