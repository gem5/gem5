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
 *          Andreas Hansson
 */

#include "base/chunk_generator.hh"
#include "debug/DMA.hh"
#include "debug/Drain.hh"
#include "dev/dma_device.hh"
#include "sim/system.hh"

DmaPort::DmaPort(MemObject *dev, System *s)
    : MasterPort(dev->name() + ".dma", dev), device(dev), sys(s),
      masterId(s->getMasterId(dev->name())),
      pendingCount(0), drainEvent(NULL),
      inRetry(false)
{ }

void
DmaPort::handleResp(PacketPtr pkt, Tick delay)
{
    // should always see a response with a sender state
    assert(pkt->isResponse());

    // get the DMA sender state
    DmaReqState *state = dynamic_cast<DmaReqState*>(pkt->senderState);
    assert(state);

    DPRINTF(DMA, "Received response %s for addr: %#x size: %d nb: %d,"  \
            " tot: %d sched %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->req->getSize(),
            state->numBytes, state->totBytes,
            state->completionEvent ?
            state->completionEvent->scheduled() : 0);

    assert(pendingCount != 0);
    pendingCount--;

    // update the number of bytes received based on the request rather
    // than the packet as the latter could be rounded up to line sizes
    state->numBytes += pkt->req->getSize();
    assert(state->totBytes >= state->numBytes);

    // if we have reached the total number of bytes for this DMA
    // request, then signal the completion and delete the sate
    if (state->totBytes == state->numBytes) {
        if (state->completionEvent) {
            delay += state->delay;
            if (delay)
                device->schedule(state->completionEvent, curTick() + delay);
            else
                state->completionEvent->process();
        }
        delete state;
    }

    // delete the request that we created and also the packet
    delete pkt->req;
    delete pkt;

    // we might be drained at this point, if so signal the drain event
    if (pendingCount == 0 && drainEvent) {
        drainEvent->process();
        drainEvent = NULL;
    }
}

bool
DmaPort::recvTimingResp(PacketPtr pkt)
{
    // We shouldn't ever get a block in ownership state
    assert(!(pkt->memInhibitAsserted() && !pkt->sharedAsserted()));

    handleResp(pkt);

    return true;
}

DmaDevice::DmaDevice(const Params *p)
    : PioDevice(p), dmaPort(this, sys)
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
    unsigned int count = pioPort.drain(de) + dmaPort.drain(de);
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
    DPRINTF(Drain, "DmaPort not drained\n");
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
    } while (result && transmitList.size());

    DPRINTF(DMA, "TransmitList: %d, inRetry: %d\n",
            transmitList.size(), inRetry);
}

void
DmaPort::dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data, Tick delay, Request::Flags flag)
{
    // one DMA request sender state for every action, that is then
    // split into many requests and packets based on the block size,
    // i.e. cache line size
    DmaReqState *reqState = new DmaReqState(event, size, delay);

    DPRINTF(DMA, "Starting DMA for addr: %#x size: %d sched: %d\n", addr, size,
            event ? event->scheduled() : -1);
    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
        Request *req = new Request(gen.addr(), gen.size(), flag, masterId);
        PacketPtr pkt = new Packet(req, cmd);

        // Increment the data pointer on a write
        if (data)
            pkt->dataStatic(data + gen.complete());

        pkt->senderState = reqState;

        DPRINTF(DMA, "--Queuing DMA for addr: %#x size: %d\n", gen.addr(),
                gen.size());
        queueDma(pkt);
    }
}

void
DmaPort::queueDma(PacketPtr pkt)
{
    transmitList.push_back(pkt);

    // remember that we have another packet pending, this will only be
    // decremented once a response comes back
    pendingCount++;

    sendDma();
}

void
DmaPort::sendDma()
{
    // some kind of selcetion between access methods
    // more work is going to have to be done to make
    // switching actually work
    assert(transmitList.size());
    PacketPtr pkt = transmitList.front();

    Enums::MemoryMode state = sys->getMemoryMode();
    if (state == Enums::timing) {
        if (inRetry) {
            DPRINTF(DMA, "Can't send immediately, waiting for retry\n");
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
        } while (result && transmitList.size());
    } else if (state == Enums::atomic) {
        transmitList.pop_front();

        DPRINTF(DMA, "Sending  DMA for addr: %#x size: %d\n",
                pkt->req->getPaddr(), pkt->req->getSize());
        Tick lat = sendAtomic(pkt);

        handleResp(pkt, lat);
    } else
        panic("Unknown memory mode.");
}

MasterPort &
DmaDevice::getMasterPort(const std::string &if_name, int idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return PioDevice::getMasterPort(if_name, idx);
}
