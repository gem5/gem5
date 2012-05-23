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
 *
 * Authors: Ali Saidi
 *          Nathan Binkert
 */

#include "base/chunk_generator.hh"
#include "debug/DMA.hh"
#include "dev/dma_device.hh"
#include "sim/system.hh"

DmaPort::DmaPort(MemObject *dev, System *s, Tick min_backoff, Tick max_backoff)
    : MasterPort(dev->name() + "-dma", dev), device(dev), sys(s),
      masterId(s->getMasterId(dev->name())),
      pendingCount(0), actionInProgress(0), drainEvent(NULL),
      backoffTime(0), minBackoffDelay(min_backoff),
      maxBackoffDelay(max_backoff), inRetry(false),
      backoffEvent(this)
{ }

bool
DmaPort::recvTimingResp(PacketPtr pkt)
{
    if (pkt->wasNacked()) {
        DPRINTF(DMA, "Received nacked %s addr %#x\n",
                pkt->cmdString(), pkt->getAddr());

        if (backoffTime < minBackoffDelay)
            backoffTime = minBackoffDelay;
        else if (backoffTime < maxBackoffDelay)
            backoffTime <<= 1;

        device->reschedule(backoffEvent, curTick() + backoffTime, true);

        DPRINTF(DMA, "Backoff time set to %d ticks\n", backoffTime);

        pkt->reinitNacked();
        queueDma(pkt, true);
    } else if (pkt->senderState) {
        DmaReqState *state;
        backoffTime >>= 2;

        DPRINTF(DMA, "Received response %s addr %#x size %#x\n",
                pkt->cmdString(), pkt->getAddr(), pkt->req->getSize());
        state = dynamic_cast<DmaReqState*>(pkt->senderState);
        pendingCount--;

        assert(pendingCount >= 0);
        assert(state);

        // We shouldn't ever get a block in ownership state
        assert(!(pkt->memInhibitAsserted() && !pkt->sharedAsserted()));

        state->numBytes += pkt->req->getSize();
        assert(state->totBytes >= state->numBytes);
        if (state->totBytes == state->numBytes) {
            if (state->completionEvent) {
                if (state->delay)
                    device->schedule(state->completionEvent,
                                     curTick() + state->delay);
                else
                    state->completionEvent->process();
            }
            delete state;
        }
        delete pkt->req;
        delete pkt;

        if (pendingCount == 0 && drainEvent) {
            drainEvent->process();
            drainEvent = NULL;
        }
    }  else {
        panic("Got packet without sender state... huh?\n");
    }

    return true;
}

DmaDevice::DmaDevice(const Params *p)
    : PioDevice(p), dmaPort(this, sys, params()->min_backoff_delay,
                            params()->max_backoff_delay)
{ }

void
DmaDevice::init()
{
    if (!dmaPort.isConnected())
        panic("DMA port of %s not connected to anything!", name());
    PioDevice::init();
}

unsigned int
DmaDevice::drain(Event *de)
{
    unsigned int count;
    count = pioPort.drain(de) + dmaPort.drain(de);
    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}

unsigned int
DmaPort::drain(Event *de)
{
    if (pendingCount == 0)
        return 0;
    drainEvent = de;
    return 1;
}

void
DmaPort::recvRetry()
{
    assert(transmitList.size());
    bool result = true;
    do {
        PacketPtr pkt = transmitList.front();
        DPRINTF(DMA, "Retry on %s addr %#x\n",
                pkt->cmdString(), pkt->getAddr());
        result = sendTimingReq(pkt);
        if (result) {
            DPRINTF(DMA, "-- Done\n");
            transmitList.pop_front();
            inRetry = false;
        } else {
            inRetry = true;
            DPRINTF(DMA, "-- Failed, queued\n");
        }
    } while (!backoffTime &&  result && transmitList.size());

    if (transmitList.size() && backoffTime && !inRetry) {
        DPRINTF(DMA, "Scheduling backoff for %d\n", curTick()+backoffTime);
        if (!backoffEvent.scheduled())
            device->schedule(backoffEvent, backoffTime + curTick());
    }
    DPRINTF(DMA, "TransmitList: %d, backoffTime: %d inRetry: %d es: %d\n",
            transmitList.size(), backoffTime, inRetry,
            backoffEvent.scheduled());
}

void
DmaPort::dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data, Tick delay, Request::Flags flag)
{
    assert(device->getState() == SimObject::Running);

    DmaReqState *reqState = new DmaReqState(event, size, delay);


    DPRINTF(DMA, "Starting DMA for addr: %#x size: %d sched: %d\n", addr, size,
            event ? event->scheduled() : -1 );
    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
            Request *req = new Request(gen.addr(), gen.size(), flag, masterId);
            PacketPtr pkt = new Packet(req, cmd);

            // Increment the data pointer on a write
            if (data)
                pkt->dataStatic(data + gen.complete());

            pkt->senderState = reqState;

            assert(pendingCount >= 0);
            pendingCount++;
            DPRINTF(DMA, "--Queuing DMA for addr: %#x size: %d\n", gen.addr(),
                    gen.size());
            queueDma(pkt);
    }

}

void
DmaPort::queueDma(PacketPtr pkt, bool front)
{

    if (front)
        transmitList.push_front(pkt);
    else
        transmitList.push_back(pkt);
    sendDma();
}

void
DmaPort::sendDma()
{
    // some kind of selction between access methods
    // more work is going to have to be done to make
    // switching actually work
    assert(transmitList.size());
    PacketPtr pkt = transmitList.front();

    Enums::MemoryMode state = sys->getMemoryMode();
    if (state == Enums::timing) {
        if (backoffEvent.scheduled() || inRetry) {
            DPRINTF(DMA, "Can't send immediately, waiting for retry or backoff timer\n");
            return;
        }

        DPRINTF(DMA, "Attempting to send %s addr %#x\n",
                pkt->cmdString(), pkt->getAddr());

        bool result;
        do {
            result = sendTimingReq(pkt);
            if (result) {
                transmitList.pop_front();
                DPRINTF(DMA, "-- Done\n");
            } else {
                inRetry = true;
                DPRINTF(DMA, "-- Failed: queued\n");
            }
        } while (result && !backoffTime && transmitList.size());

        if (transmitList.size() && backoffTime && !inRetry &&
                !backoffEvent.scheduled()) {
            DPRINTF(DMA, "-- Scheduling backoff timer for %d\n",
                    backoffTime+curTick());
            device->schedule(backoffEvent, backoffTime + curTick());
        }
    } else if (state == Enums::atomic) {
        transmitList.pop_front();

        Tick lat;
        DPRINTF(DMA, "--Sending  DMA for addr: %#x size: %d\n",
                pkt->req->getPaddr(), pkt->req->getSize());
        lat = sendAtomic(pkt);
        assert(pkt->senderState);
        DmaReqState *state = dynamic_cast<DmaReqState*>(pkt->senderState);
        assert(state);
        state->numBytes += pkt->req->getSize();

        DPRINTF(DMA, "--Received response for  DMA for addr: %#x size: %d nb: %d, tot: %d sched %d\n",
                pkt->req->getPaddr(), pkt->req->getSize(), state->numBytes,
                state->totBytes,
                state->completionEvent ? state->completionEvent->scheduled() : 0 );

        if (state->totBytes == state->numBytes) {
            if (state->completionEvent) {
                assert(!state->completionEvent->scheduled());
                device->schedule(state->completionEvent,
                                 curTick() + lat + state->delay);
            }
            delete state;
            delete pkt->req;
        }
        pendingCount--;
        assert(pendingCount >= 0);
        delete pkt;

        if (pendingCount == 0 && drainEvent) {
            drainEvent->process();
            drainEvent = NULL;
        }

   } else
       panic("Unknown memory command state.");
}

DmaDevice::~DmaDevice()
{
}

MasterPort &
DmaDevice::getMasterPort(const std::string &if_name, int idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return PioDevice::getMasterPort(if_name, idx);
}
