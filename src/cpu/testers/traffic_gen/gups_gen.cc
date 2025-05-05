/*
 * Copyright (c) 2021 The Regents of the University of California.
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

#include "cpu/testers/traffic_gen/gups_gen.hh"

#include <cstring>
#include <string>

#include "debug/GUPSGen.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

GUPSGen::GUPSGen(const GUPSGenParams& params):
    ClockedObject(params),
    nextCreateEvent([this]{ createNextReq(); }, name()),
    nextSendEvent([this]{ sendNextReq(); }, name()),
    system(params.system),
    requestorId(system->getRequestorId(this)),
    port(name() + ".port", this),
    startAddr(params.start_addr),
    memSize(params.mem_size),
    updateLimit(params.update_limit),
    elementSize(sizeof(uint64_t)), // every element in the table is a uint64_t
    reqQueueSize(params.request_queue_size),
    initMemory(params.init_memory),
    stats(this)
{}

Port&
GUPSGen::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return ClockedObject::getPort(if_name, idx);
    } else {
        return port;
    }
}

void
GUPSGen::init()
{
    doneReading = false;
    onTheFlyRequests = 0;
    readRequests = 0;

    tableSize = memSize / elementSize;
    numUpdates = 4 * tableSize;
}

void
GUPSGen::startup()
{
    int block_size = 64; // Write the initial values in 64 byte blocks.
    uint64_t stride_size = block_size / elementSize;
    auto write_data = std::make_unique<uint8_t[]>(block_size);
    if (initMemory) {
        for (uint64_t start_index = 0; start_index < tableSize;
                                    start_index += stride_size) {
            for (uint64_t offset = 0; offset < stride_size; offset++) {
                uint64_t value = start_index + offset;
                std::memcpy(write_data.get() + offset * elementSize,
                            &value, elementSize);
            }
            Addr addr = indexToAddr(start_index);
            PacketPtr pkt = getWritePacket(addr, block_size, write_data.get());
            port.sendFunctionalPacket(pkt);
            delete pkt;
        }
    }
    schedule(nextCreateEvent, nextCycle());
}

Addr
GUPSGen::indexToAddr (uint64_t index)
{
    Addr ret = index * elementSize + startAddr;
    return ret;
}

PacketPtr
GUPSGen::getReadPacket(Addr addr, unsigned int size)
{
    RequestPtr req = std::make_shared<Request>(addr, size, 0, requestorId);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)requestorId) << 2);

    // Embed it in a packet
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();

    return pkt;
}

PacketPtr
GUPSGen::getWritePacket(Addr addr, unsigned int size, uint8_t *data)
{
    RequestPtr req = std::make_shared<Request>(addr, size, 0,
                                               requestorId);
    // Dummy PC to have PC-based prefetchers latch on; get entropy into higher
    // bits
    req->setPC(((Addr)requestorId) << 2);

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(data);

    return pkt;
}

void
GUPSGen::handleResponse(PacketPtr pkt)
{
    onTheFlyRequests--;
    DPRINTF(GUPSGen, "%s: onTheFlyRequests: %d.\n",
                __func__, onTheFlyRequests);
    if (pkt->isWrite()) {
        DPRINTF(GUPSGen, "%s: received a write resp. pkt->addr_range: %s,"
                        " pkt->data: %d\n", __func__,
                        pkt->getAddrRange().to_string(),
                        *pkt->getPtr<uint64_t>());
        stats.totalUpdates++;
        stats.totalWrites++;
        stats.totalBytesWritten += elementSize;
        stats.totalWriteLat += curTick() - exitTimes[pkt->req];

        exitTimes.erase(pkt->req);
        delete pkt;
    } else {
        DPRINTF(GUPSGen, "%s: received a read resp. pkt->addr_range: %s\n",
                        __func__, pkt->getAddrRange().to_string());

        stats.totalReads++;
        stats.totalBytesRead += elementSize;
        stats.totalReadLat += curTick() - exitTimes[pkt->req];

        exitTimes.erase(pkt->req);

        responsePool.push(pkt);
    }
    if (doneReading && requestPool.empty() && onTheFlyRequests == 0) {
        exitSimLoop(name() + " is finished updating the memory.\n");
        return;
    }

    if ((requestPool.size() < reqQueueSize) &&
        (!doneReading || !responsePool.empty()) &&
        !nextCreateEvent.scheduled())
    {
        schedule(nextCreateEvent, nextCycle());
    }
}

void
GUPSGen::wakeUp()
{
    if (!nextSendEvent.scheduled() && !requestPool.empty()) {
        schedule(nextSendEvent, nextCycle());
    }
}

void
GUPSGen::createNextReq()
{
    // Prioritize pending writes over reads
    // Write as soon as the data is read
    if (!responsePool.empty()) {
        PacketPtr pkt = responsePool.front();
        responsePool.pop();

        uint64_t *updated_value = pkt->getPtr<uint64_t>();
        DPRINTF(GUPSGen, "%s: Read value %lu from address %s", __func__,
                            *updated_value, pkt->getAddrRange().to_string());
        *updated_value ^= updateTable[pkt->req];
        updateTable.erase(pkt->req);
        Addr addr = pkt->getAddr();
        PacketPtr new_pkt = getWritePacket(addr,
                            elementSize, (uint8_t*) updated_value);
        delete pkt;
        requestPool.push(new_pkt);
    } else if (!doneReading) {
        // If no writes then read
        // Check to make sure we're not reading more than we should.
        assert (readRequests < numUpdates);

        uint64_t value = readRequests;
        uint64_t index = rng->random<int64_t>((int64_t) 0, tableSize);
        Addr addr = indexToAddr(index);
        PacketPtr pkt = getReadPacket(addr, elementSize);
        updateTable[pkt->req] = value;
        requestPool.push(pkt);
        readRequests++;

        if (readRequests >= numUpdates) {
            DPRINTF(GUPSGen, "%s: Done creating reads.\n", __func__);
            doneReading = true;
        }
        else if (readRequests == updateLimit && updateLimit != 0) {
            DPRINTF(GUPSGen, "%s: Update limit reached.\n", __func__);
            doneReading = true;
        }
    }

    if (!nextCreateEvent.scheduled() &&
        (requestPool.size() < reqQueueSize) &&
        (!doneReading || !responsePool.empty()))
    {
        schedule(nextCreateEvent, nextCycle());
    }

    if (!nextSendEvent.scheduled() && !requestPool.empty()) {
        schedule(nextSendEvent, nextCycle());
    }
}

void
GUPSGen::sendNextReq()
{
    if (!port.blocked()) {
        PacketPtr pkt = requestPool.front();

        exitTimes[pkt->req] = curTick();
        if (pkt->isWrite()) {
            DPRINTF(GUPSGen, "%s: Sent write pkt, pkt->addr_range: "
                    "%s, pkt->data: %lu.\n", __func__,
                    pkt->getAddrRange().to_string(),
                    *pkt->getPtr<uint64_t>());
        } else {
            DPRINTF(GUPSGen, "%s: Sent read pkt, pkt->addr_range: %s.\n",
                    __func__, pkt->getAddrRange().to_string());
        }
        port.sendTimingPacket(pkt);
        onTheFlyRequests++;
        DPRINTF(GUPSGen, "%s: onTheFlyRequests: %d.\n",
                __func__, onTheFlyRequests);
        requestPool.pop();
    }

    if (!nextCreateEvent.scheduled() &&
        (requestPool.size() < reqQueueSize) &&
        (!doneReading || !responsePool.empty()))
    {
        schedule(nextCreateEvent, nextCycle());
    }

    if (!nextSendEvent.scheduled()) {
        if (!requestPool.empty()) {
            schedule(nextSendEvent, nextCycle());
        }
    }
}


void
GUPSGen::GenPort::sendTimingPacket(PacketPtr pkt)
{
    panic_if(_blocked, "Should never try to send if blocked MemSide!");

    // If we can't send the packet across the port, store it for later.
    if (!sendTimingReq(pkt)) {
        DPRINTF(GUPSGen, "GenPort::%s: Packet blocked\n", __func__);
        blockedPacket = pkt;
        _blocked = true;
    } else {
        DPRINTF(GUPSGen, "GenPort::%s: Packet sent\n", __func__);
    }
}

void
GUPSGen::GenPort::sendFunctionalPacket(PacketPtr pkt)
{
    sendFunctional(pkt);
}

void
GUPSGen::GenPort::recvReqRetry()
{
    // We should have a blocked packet if this function is called.
    DPRINTF(GUPSGen, "GenPort::%s: Received a retry.\n", __func__);

    assert(_blocked && (blockedPacket != nullptr));
    // Try to resend it. It's possible that it fails again.
    _blocked = false;
    sendTimingPacket(blockedPacket);
    if (!_blocked){
        blockedPacket = nullptr;
    }

    owner->wakeUp();
}

bool
GUPSGen::GenPort::recvTimingResp(PacketPtr pkt)
{
    owner->handleResponse(pkt);
    return true;
}

GUPSGen::GUPSGenStat::GUPSGenStat(GUPSGen* parent) :
    statistics::Group(parent),
    ADD_STAT(totalUpdates, statistics::units::Count::get(),
        "Total number of updates the generator made in the memory"),
    ADD_STAT(GUPS, statistics::units::Rate<statistics::units::Count,
                statistics::units::Second>::get(),
        "Rate of billion updates per second"),
    ADD_STAT(totalReads, statistics::units::Count::get(),
        "Total number of read requests"),
    ADD_STAT(totalBytesRead, statistics::units::Byte::get(),
        "Total number of bytes read"),
    ADD_STAT(avgReadBW, statistics::units::Rate<statistics::units::Byte,
                statistics::units::Second>::get(),
        "Average read bandwidth received from memory"),
    ADD_STAT(totalReadLat, statistics::units::Tick::get(),
        "Total latency of read requests."),
    ADD_STAT(avgReadLat, statistics::units::Tick::get(),
        "Average latency for read requests"),
    ADD_STAT(totalWrites, statistics::units::Count::get(),
        "Total number of write requests"),
    ADD_STAT(totalBytesWritten, statistics::units::Byte::get(),
        "Total number of bytes written"),
    ADD_STAT(avgWriteBW, statistics::units::Rate<statistics::units::Byte,
                statistics::units::Second>::get(),
        "Average write bandwidth received from memory"),
    ADD_STAT(totalWriteLat, statistics::units::Tick::get(),
        "Total latency of write requests."),
    ADD_STAT(avgWriteLat, statistics::units::Tick::get(),
        "Average latency for write requests")
{}

void
GUPSGen::GUPSGenStat::regStats()
{
    GUPS.precision(8);
    avgReadBW.precision(2);
    avgReadLat.precision(2);
    avgWriteBW.precision(2);
    avgWriteLat.precision(2);

    GUPS = (totalUpdates / 1e9) / simSeconds;

    avgReadBW = totalBytesRead / simSeconds;
    avgReadLat = (totalReadLat) / totalReads;

    avgWriteBW = totalBytesWritten / simSeconds;
    avgWriteLat = (totalWriteLat) / totalWrites;
}

}
