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
    : MasterPort(dev->name() + ".dma", dev), device(dev), sendEvent(this),
      sys(s), masterId(s->getMasterId(dev->name())),
      pendingCount(0), drainManager(NULL),
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
            device->schedule(state->completionEvent, curTick() + delay);
        }
        delete state;
    }

    // delete the request that we created and also the packet
    delete pkt->req;
    delete pkt;

    // we might be drained at this point, if so signal the drain event
    if (pendingCount == 0 && drainManager) {
        drainManager->signalDrainDone();
        drainManager = NULL;
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
DmaDevice::drain(DrainManager *dm)
{
    unsigned int count = pioPort.drain(dm) + dmaPort.drain(dm);
    if (count)
        setDrainState(Drainable::Draining);
    else
        setDrainState(Drainable::Drained);
    return count;
}

unsigned int
DmaPort::drain(DrainManager *dm)
{
    if (pendingCount == 0)
        return 0;
    drainManager = dm;
    DPRINTF(Drain, "DmaPort not drained\n");
    return 1;
}

void
DmaPort::recvRetry()
{
    assert(transmitList.size());
    trySendTimingReq();
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
    for (ChunkGenerator gen(addr, size, sys->cacheLineSize());
         !gen.done(); gen.next()) {
        Request *req = new Request(gen.addr(), gen.size(), flag, masterId);
        req->taskId(ContextSwitchTaskId::DMA);
        PacketPtr pkt = new Packet(req, cmd);

        // Increment the data pointer on a write
        if (data)
            pkt->dataStatic(data + gen.complete());

        pkt->senderState = reqState;

        DPRINTF(DMA, "--Queuing DMA for addr: %#x size: %d\n", gen.addr(),
                gen.size());
        queueDma(pkt);
    }

    // in zero time also initiate the sending of the packets we have
    // just created, for atomic this involves actually completing all
    // the requests
    sendDma();
}

void
DmaPort::queueDma(PacketPtr pkt)
{
    transmitList.push_back(pkt);

    // remember that we have another packet pending, this will only be
    // decremented once a response comes back
    pendingCount++;
}

void
DmaPort::trySendTimingReq()
{
    // send the first packet on the transmit list and schedule the
    // following send if it is successful
    PacketPtr pkt = transmitList.front();

    DPRINTF(DMA, "Trying to send %s addr %#x\n", pkt->cmdString(),
            pkt->getAddr());

    inRetry = !sendTimingReq(pkt);
    if (!inRetry) {
        transmitList.pop_front();
        DPRINTF(DMA, "-- Done\n");
        // if there is more to do, then do so
        if (!transmitList.empty())
            // this should ultimately wait for as many cycles as the
            // device needs to send the packet, but currently the port
            // does not have any known width so simply wait a single
            // cycle
            device->schedule(sendEvent, device->clockEdge(Cycles(1)));
    } else {
        DPRINTF(DMA, "-- Failed, waiting for retry\n");
    }

    DPRINTF(DMA, "TransmitList: %d, inRetry: %d\n",
            transmitList.size(), inRetry);
}

void
DmaPort::sendDma()
{
    // some kind of selcetion between access methods
    // more work is going to have to be done to make
    // switching actually work
    assert(transmitList.size());

    if (sys->isTimingMode()) {
        // if we are either waiting for a retry or are still waiting
        // after sending the last packet, then do not proceed
        if (inRetry || sendEvent.scheduled()) {
            DPRINTF(DMA, "Can't send immediately, waiting to send\n");
            return;
        }

        trySendTimingReq();
    } else if (sys->isAtomicMode()) {
        // send everything there is to send in zero time
        while (!transmitList.empty()) {
            PacketPtr pkt = transmitList.front();
            transmitList.pop_front();

            DPRINTF(DMA, "Sending  DMA for addr: %#x size: %d\n",
                    pkt->req->getPaddr(), pkt->req->getSize());
            Tick lat = sendAtomic(pkt);

            handleResp(pkt, lat);
        }
    } else
        panic("Unknown memory mode.");
}

BaseMasterPort &
DmaDevice::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return PioDevice::getMasterPort(if_name, idx);
}
