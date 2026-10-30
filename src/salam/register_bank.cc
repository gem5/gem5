/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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
 */

#include "salam/register_bank.hh"

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/user.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>

#include "base/trace.hh"
#include "debug/Drain.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/core.hh"
#include "sim/system.hh"

using namespace std;

#include "debug/MemoryAccess.hh"

RegisterBank::RegisterBank(const RegisterBankParams &p)
    : AbstractMemory(p),
      ppa(p.ppa),
      port(name() + ".reg_port", this),
      load(name() + ".load_port", this),
      accessCycles(p.access_cycles),
      writeCycles(p.write_cycles),
      retryResp(false),
      sendingResponse(false),
      dequeueEvent([this] { dequeue(); }, name()),
      // Publish the batch before either port's default-priority response
      // event, including when both delays are zero on a clock edge.
      deltaEvent([this] { delta(); }, name(), false, Event::Default_Pri - 1)
{
    // Setup of the delta memory container
    int shm_fd = -1;
    int map_flags = MAP_ANON | MAP_PRIVATE;

    deltaAddr = (uint8_t *)mmap(NULL, range.size(), PROT_READ | PROT_WRITE,
                                map_flags, shm_fd, 0);
    if (deltaAddr == (uint8_t *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap %d bytes for range %s!\n", range.size(),
              range.to_string());
    }
}

static inline void
tracePacket(System *sys, const char *label, PacketPtr pkt)
{
    int size = pkt->getSize();
    if (size == 1 || size == 2 || size == 4 || size == 8) {
        DPRINTF(MemoryAccess,
                "%s from %s of size %i on address %#x data "
                "%#x %c\n",
                label, sys->getRequestorName(pkt->req->requestorId()), size,
                pkt->getAddr(), pkt->getUintX(ByteOrder::little),
                pkt->req->isUncacheable() ? 'U' : 'C');
        return;
    }
    DPRINTF(MemoryAccess, "%s from %s of size %i on address %#x %c\n", label,
            sys->getRequestorName(pkt->req->requestorId()), size,
            pkt->getAddr(), pkt->req->isUncacheable() ? 'U' : 'C');
    DDUMP(MemoryAccess, pkt->getConstPtr<uint8_t>(), pkt->getSize());
}

#if TRACING_ON
#define TRACE_PACKET(A) tracePacket(system(), A, pkt)
#else
#define TRACE_PACKET(A)
#endif

void
RegisterBank::registerAccess(PacketPtr pkt)
{
    assert(AddrRange(pkt->getAddr(), pkt->getAddr() + (pkt->getSize()))
               .isSubset(range));

    if (pkt->isRead()) {
        assert(!pkt->isWrite());
        uint8_t *hostAddr = pmemAddr + pkt->getAddr() - range.start();
        if (pmemAddr) {
            pkt->setData(hostAddr);
        }
        ppa->accountStorageAccess(*pkt, range.start());
        stats.numReads[pkt->req->requestorId()]++;
        stats.bytesRead[pkt->req->requestorId()] += pkt->getSize();
    } else if (pkt->isWrite()) {
        uint8_t *hostAddr = deltaAddr + pkt->getAddr() - range.start();
        if (writeOK(pkt)) {
            if (deltaAddr) {
                // Start each pending batch from the visible backing store.
                // Functional initialization and restored memory need not have
                // passed through registerAccess().
                if (!deltaEvent.scheduled() && pmemAddr) {
                    std::memcpy(deltaAddr, pmemAddr, range.size());
                }
                pkt->writeData(hostAddr);
                DPRINTF(MemoryAccess, "%s wrote %i bytes to address %x\n",
                        __func__, pkt->getSize(), pkt->getAddr());
            }
            assert(!pkt->req->isInstFetch());
            TRACE_PACKET("Write");
            ppa->accountStorageAccess(*pkt, range.start());
            stats.numWrites[pkt->req->requestorId()]++;
            stats.bytesWritten[pkt->req->requestorId()] += pkt->getSize();
            if (!deltaEvent.scheduled()) {
                schedule(deltaEvent, clockEdge(writeCycles));
            }
        }
    } else {
        panic("Unexpected packet %s", pkt->print());
    }

    if (pkt->needsResponse()) {
        pkt->makeResponse();
    }
}

void
RegisterBank::init()
{
    // allow unconnected memories as this is used in several ruby
    // systems at the moment
    if (port.isConnected()) {
        port.sendRangeChange();
    }
    if (load.isConnected()) {
        load.sendRangeChange();
    }
    ppa->configureStorage(name(), range.size(),
                          port.isConnected() + load.isConnected());
    fatal_if(ppa->isEnabled() && !ppa->storagePhysical().empty() &&
                 ppa->storagePhysical() != "flip_flop",
             "%s: RegisterBank characterization must describe flip-flops",
             name());
    fatal_if(ppa->isEnabled() && ppa->maskModel() == "read_modify_write",
             "%s: flip-flop read-modify-write is unsupported", name());
    // Private reads respond immediately; load-port latency does not make
    // this service compatible with a positive physical access latency.
    if (ppa->isEnabled() && ppa->minimumAccessLatency() > 0) {
        ppa->markIncomplete("memory:immediate_RegisterBank_read_latency");
    }
    if (ppa->isEnabled() && ppa->minimumAccessInterval() > 0) {
        ppa->markIncomplete("memory:unbounded_RegisterBank_service_rate");
    }
}

Tick
RegisterBank::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");
    registerAccess(pkt);
    Tick latency = accessLatency();
    if (pkt->isWrite() && deltaEvent.scheduled()) {
        // Completion releases dependent loads. Acknowledgment must wait
        // for this write's batch to reach the visible backing store.
        latency = std::max(latency, deltaEvent.when() - curTick());
    }
    return latency;
}

Tick
RegisterBank::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    Tick latency = recvAtomic(pkt);

    if (backdoor.ptr()) {
        // Later direct accesses bypass packet service accounting.
        ppa->markIncomplete("memory:backdoor_activity_unobserved");
        _backdoor = &backdoor;
    }
    return latency;
}

void
RegisterBank::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    const bool write = pkt->isWrite();
    functionalAccess(pkt);

    if (pmemAddr && write) {
        // Functional writes are immediately visible and supersede matching
        // bytes in the pending delta, without starting a timing access.
        pkt->writeData(deltaAddr + pkt->getAddr() - range.start());
        for (auto &pending : packetQueue) {
            pkt->trySatisfyFunctional(pending.pkt);
        }
        load.trySatisfyFunctional(pkt);
    } else if (pmemAddr && pkt->isRead() && deltaEvent.scheduled()) {
        // The delta includes the latest accepted writes, even before those
        // writes become visible to ordinary timed register reads.
        pkt->setData(deltaAddr + pkt->getAddr() - range.start());
    }

    pkt->popLabel();
}

bool
RegisterBank::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller, "
             "saw %s to %#llx\n",
             pkt->cmdString(), pkt->getAddr());

    bool isRead = pkt->isRead();

    bool needsResponse = pkt->needsResponse();
    const Tick latency = recvAtomic(pkt);
    Tick responseTime = curTick() + (isRead ? 0 : latency);
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());
        const bool immediate =
            isRead && packetQueue.empty() && !retryResp && !sendingResponse;
        auto i = packetQueue.end();
        while (i != packetQueue.begin()) {
            auto previous = std::prev(i);
            if (responseTime >= previous->tick ||
                previous->pkt->matchAddr(pkt) ||
                (previous == packetQueue.begin() && retryResp)) {
                break;
            }
            i = previous;
        }
        packetQueue.emplace(i, pkt, responseTime);
        if (immediate) {
            dequeue();
        } else {
            scheduleNextResponse();
        }
    }
    return true;
}

void
RegisterBank::dequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();
    packetQueue.pop_front();

    // The requestor may delete the response and send its next request in
    // this call stack. Do not leave the outgoing packet in the queue or
    // allow a nested response to overwrite this send's retry state.
    sendingResponse = true;
    retryResp = !port.sendTimingResp(deferred_pkt.pkt);
    sendingResponse = false;
    if (retryResp) {
        packetQueue.push_front(deferred_pkt);
    } else if (!packetQueue.empty()) {
        scheduleNextResponse();
    } else if (drainState() == DrainState::Draining) {
        DPRINTF(Drain, "Draining of RegisterBank complete\n");
        signalDrainDone();
    }
}

void
RegisterBank::delta()
{
    std::memcpy(pmemAddr, deltaAddr, range.size());
}

void
RegisterBank::recvRespRetry()
{
    assert(retryResp);
    retryResp = false;
    scheduleNextResponse();
}

void
RegisterBank::scheduleNextResponse()
{
    if (packetQueue.empty() || retryResp || sendingResponse) {
        return;
    }
    const Tick ready = std::max(packetQueue.front().tick, curTick());
    if (!dequeueEvent.scheduled() || ready < dequeueEvent.when()) {
        reschedule(dequeueEvent, ready, true);
    }
}

Port &
RegisterBank::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "reg_port") {
        return port;
    } else if (if_name == "load_port") {
        return load;
    }
    return AbstractMemory::getPort(if_name, idx);
}

DrainState
RegisterBank::drain()
{
    if (!packetQueue.empty()) {
        DPRINTF(Drain, "ScratchpadMemory Queue has requests, "
                       "waiting to drain\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}
