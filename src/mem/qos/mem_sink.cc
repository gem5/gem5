/*
 * Copyright (c) 2018 ARM Limited
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
 * Author: Matteo Andreozzi
 */

#include "debug/Drain.hh"
#include "debug/QOS.hh"
#include "mem_sink.hh"
#include "sim/system.hh"

namespace QoS {

MemSinkCtrl::MemSinkCtrl(const QoSMemSinkCtrlParams* p)
  : MemCtrl(p), requestLatency(p->request_latency),
    responseLatency(p->response_latency),
    memoryPacketSize(p->memory_packet_size),
    readBufferSize(p->read_buffer_size),
    writeBufferSize(p->write_buffer_size), port(name() + ".port", *this),
    retryRdReq(false), retryWrReq(false), nextRequest(0), nextReqEvent(this)
{
    // Resize read and write queue to allocate space
    // for configured QoS priorities
    readQueue.resize(numPriorities());
    writeQueue.resize(numPriorities());
}

MemSinkCtrl::~MemSinkCtrl()
{}

void
MemSinkCtrl::init()
{
    MemCtrl::init();

    // Allow unconnected memories as this is used in several ruby
    // systems at the moment
    if (port.isConnected()) {
        port.sendRangeChange();
    }
}

bool
MemSinkCtrl::readQueueFull(const uint64_t packets) const
{
    return (totalReadQueueSize + packets > readBufferSize);
}

bool
MemSinkCtrl::writeQueueFull(const uint64_t packets) const
{
    return (totalWriteQueueSize + packets > writeBufferSize);
}

Tick
MemSinkCtrl::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(),
             "%s Should not see packets where cache is responding\n",
             __func__);

    access(pkt);
    return responseLatency;
}

void
MemSinkCtrl::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    pkt->popLabel();
}

Port &
MemSinkCtrl::getPort(const std::string &interface, PortID idx)
{
    if (interface != "port") {
        return MemCtrl::getPort(interface, idx);
    } else {
        return port;
    }
}

bool
MemSinkCtrl::recvTimingReq(PacketPtr pkt)
{
    // Request accepted
    bool req_accepted = true;

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "%s. Should only see "
             "read and writes at memory controller\n",
             __func__);

    panic_if(pkt->cacheResponding(),
             "%s. Should not see packets where cache is responding\n",
             __func__);

    DPRINTF(QOS,
            "%s: MASTER %s request %s addr %lld size %d\n",
            __func__,
            _system->getMasterName(pkt->req->masterId()),
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    uint64_t required_entries = divCeil(pkt->getSize(), memoryPacketSize);

    assert(required_entries);

    // Schedule packet
    uint8_t pkt_priority = qosSchedule({&readQueue, &writeQueue},
                                       memoryPacketSize, pkt);

    if (pkt->isRead()) {
        if (readQueueFull(required_entries)) {
            DPRINTF(QOS,
                    "%s Read queue full, not accepting\n", __func__);
            // Remember that we have to retry this port
            retryRdReq = true;
            numReadRetries++;
            req_accepted = false;
        } else {
            // Enqueue the incoming packet into corresponding
            // QoS priority queue
            readQueue.at(pkt_priority).push_back(pkt);
            queuePolicy->enqueuePacket(pkt);
        }
    } else {
        if (writeQueueFull(required_entries)) {
            DPRINTF(QOS,
                    "%s Write queue full, not accepting\n", __func__);
            // Remember that we have to retry this port
            retryWrReq = true;
            numWriteRetries++;
            req_accepted = false;
        } else {
            // Enqueue the incoming packet into corresponding QoS
            // priority queue
            writeQueue.at(pkt_priority).push_back(pkt);
            queuePolicy->enqueuePacket(pkt);
        }
    }

    if (req_accepted) {
        // The packet is accepted - log it
        logRequest(pkt->isRead()? READ : WRITE,
                   pkt->req->masterId(),
                   pkt->qosValue(),
                   pkt->getAddr(),
                   required_entries);
    }

    // Check if we have to process next request event
    if (!nextReqEvent.scheduled()) {
        DPRINTF(QOS,
                "%s scheduling next request at "
                "time %d (next is %d)\n", __func__,
                std::max(curTick(), nextRequest), nextRequest);
        schedule(nextReqEvent, std::max(curTick(), nextRequest));
    }
    return req_accepted;
}

void
MemSinkCtrl::processNextReqEvent()
{
    PacketPtr pkt = nullptr;

    // Evaluate bus direction
    busStateNext = selectNextBusState();

    // Record turnaround stats and update current state direction
    recordTurnaroundStats();

    // Set current bus state
    setCurrentBusState();

    // Access current direction buffer
    std::vector<PacketQueue>* queue_ptr = (busState == READ ? &readQueue :
                                                              &writeQueue);

    DPRINTF(QOS,
            "%s DUMPING %s queues status\n", __func__,
            (busState == WRITE ? "WRITE" : "READ"));

    if (DTRACE(QOS)) {
        for (uint8_t i = 0; i < numPriorities(); ++i) {
            std::string plist = "";
            for (auto& e : (busState == WRITE ? writeQueue[i]: readQueue[i])) {
                plist += (std::to_string(e->req->masterId())) + " ";
            }
            DPRINTF(QOS,
                    "%s priority Queue [%i] contains %i elements, "
                    "packets are: [%s]\n", __func__, i,
                    busState == WRITE ? writeQueueSizes[i] :
                                        readQueueSizes[i],
                    plist);
        }
    }

    uint8_t curr_prio = numPriorities();

    for (auto queue = (*queue_ptr).rbegin();
         queue != (*queue_ptr).rend(); ++queue) {

        curr_prio--;

        DPRINTF(QOS,
                "%s checking %s queue [%d] priority [%d packets]\n",
                __func__, (busState == READ? "READ" : "WRITE"),
                curr_prio, queue->size());

        if (!queue->empty()) {
            // Call the queue policy to select packet from priority queue
            auto p_it = queuePolicy->selectPacket(&(*queue));
            pkt = *p_it;
            queue->erase(p_it);

            DPRINTF(QOS,
                    "%s scheduling packet address %d for master %s from "
                    "priority queue %d\n", __func__, pkt->getAddr(),
                    _system->getMasterName(pkt->req->masterId()),
                    curr_prio);
            break;
        }
    }

    assert(pkt);

    // Setup next request service time - do it here as retry request
    // hands over control to the port
    nextRequest = curTick() + requestLatency;

    uint64_t removed_entries = divCeil(pkt->getSize(), memoryPacketSize);

    DPRINTF(QOS,
            "%s scheduled packet address %d for master %s size is %d, "
            "corresponds to %d memory packets\n", __func__, pkt->getAddr(),
            _system->getMasterName(pkt->req->masterId()),
            pkt->getSize(), removed_entries);

    // Schedule response
    panic_if(!pkt->needsResponse(),
        "%s response not required\n", __func__);

    // Do the actual memory access which also turns the packet
    // into a response
    access(pkt);

    // Log the response
    logResponse(pkt->isRead()? READ : WRITE,
                pkt->req->masterId(),
                pkt->qosValue(),
                pkt->getAddr(),
                removed_entries, responseLatency);

    // Schedule the response
    port.schedTimingResp(pkt, curTick() + responseLatency);
    DPRINTF(QOS,
            "%s response scheduled at time %d\n",
            __func__, curTick() + responseLatency);

    // Finally - handle retry requests - this handles control
    // to the port, so do it last
    if (busState == READ && retryRdReq) {
        retryRdReq = false;
        port.sendRetryReq();
    } else if (busState == WRITE && retryWrReq) {
        retryWrReq = false;
        port.sendRetryReq();
    }

    // Check if we have to schedule another request event
    if ((totalReadQueueSize || totalWriteQueueSize) &&
        !nextReqEvent.scheduled()) {

        schedule(nextReqEvent, curTick() + requestLatency);
        DPRINTF(QOS,
                "%s scheduling next request event at tick %d\n",
                __func__, curTick() + requestLatency);
    }
}

DrainState
MemSinkCtrl::drain()
{
    if (totalReadQueueSize || totalWriteQueueSize) {
        DPRINTF(Drain,
                "%s queues have requests, waiting to drain\n",
                __func__);
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
MemSinkCtrl::regStats()
{
    MemCtrl::regStats();

    // Initialize all the stats
    using namespace Stats;

    numReadRetries.name(name() + ".numReadRetries")
        .desc("Number of read retries");
    numWriteRetries.name(name() + ".numWriteRetries")
        .desc("Number of write retries");
}

MemSinkCtrl::MemoryPort::MemoryPort(const std::string& n,
                                    MemSinkCtrl& m)
  : QueuedSlavePort(n, &m, queue, true), memory(m), queue(memory, *this, true)
{}

AddrRangeList
MemSinkCtrl::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

Tick
MemSinkCtrl::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.recvAtomic(pkt);
}

void
MemSinkCtrl::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(memory.name());

    if (!queue.trySatisfyFunctional(pkt)) {
        // Default implementation of SimpleTimingPort::recvFunctional()
        // calls recvAtomic() and throws away the latency; we can save a
        // little here by just not calculating the latency.
        memory.recvFunctional(pkt);
    }

    pkt->popLabel();
}

bool
MemSinkCtrl::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    return memory.recvTimingReq(pkt);
}

} // namespace QoS

QoS::MemSinkCtrl*
QoSMemSinkCtrlParams::create()
{
    return new QoS::MemSinkCtrl(this);
}

