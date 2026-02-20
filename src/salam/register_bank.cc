/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "sim/system.hh"

using namespace std;

#include "debug/MemoryAccess.hh"

RegisterBank::RegisterBank(const RegisterBankParams &p)
    : AbstractMemory(p),
      port(name() + ".reg_port", this),
      load(name() + ".load_port", this),
      deltaTime(p.delta_time),
      retryResp(false),
      dequeueEvent([this] { dequeue(); }, name()),
      deltaEvent([this] { delta(); }, name())
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
        stats.numReads[pkt->req->requestorId()]++;
        stats.bytesRead[pkt->req->requestorId()] += pkt->getSize();
    } else if (pkt->isWrite()) {
        uint8_t *hostAddr = deltaAddr + pkt->getAddr() - range.start();
        if (writeOK(pkt)) {
            if (deltaAddr) {
                pkt->writeData(hostAddr);
                DPRINTF(MemoryAccess, "%s wrote %i bytes to address %x\n",
                        __func__, pkt->getSize(), pkt->getAddr());
            }
            assert(!pkt->req->isInstFetch());
            TRACE_PACKET("Write");
            stats.numWrites[pkt->req->requestorId()]++;
            stats.bytesWritten[pkt->req->requestorId()] += pkt->getSize();
            if (!deltaEvent.scheduled()) {
                schedule(deltaEvent, curTick() + deltaTime);
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
}

Tick
RegisterBank::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");
    registerAccess(pkt);
    return deltaTime;
}

Tick
RegisterBank::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    Tick latency = recvAtomic(pkt);

    if (backdoor.ptr()) {
        _backdoor = &backdoor;
    }
    return latency;
}

void
RegisterBank::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    bool done = false;
    auto p = packetQueue.begin();
    // potentially update the packets in our packet queue as well
    while (!done && p != packetQueue.end()) {
        done = pkt->trySatisfyFunctional(p->pkt);
        ++p;
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
    Tick responseTime = curTick() + recvAtomic(pkt);
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());
        if (isRead) {
            retryResp = !port.sendTimingResp(pkt);
            if (!retryResp) {
                return true;
            }
        }
        // typically this should be added at the end, so start the
        // insertion sort with the last element, also make sure not to
        // re-order in front of some existing packet with the same
        // address, the latter is important as this memory effectively
        // hands out exclusive copies (shared is not asserted)
        auto i = packetQueue.end();
        --i;
        while (i != packetQueue.begin() && responseTime < i->tick &&
               !i->pkt->matchAddr(pkt)) {
            --i;
        }

        // emplace inserts the element before the position pointed to by
        // the iterator, so advance it one step
        packetQueue.emplace(++i, pkt, responseTime);

        if (!retryResp && !dequeueEvent.scheduled()) {
            schedule(dequeueEvent, packetQueue.back().tick);
        }
    }
    return true;
}

void
RegisterBank::dequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();

    retryResp = !port.sendTimingResp(deferred_pkt.pkt);
    if (!retryResp) {
        packetQueue.pop_front();

        // if the queue is not empty, schedule the next dequeue event,
        // otherwise signal that we are drained if we were asked to do so
        if (!packetQueue.empty()) {
            // if there were packets that got in-between then we
            // already have an event scheduled, so use re-schedule
            reschedule(dequeueEvent,
                       std::max(packetQueue.front().tick, curTick()), true);
        } else if (drainState() == DrainState::Draining) {
            DPRINTF(Drain, "Draining of ScratchpadMemory complete\n");
            signalDrainDone();
        }
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
    dequeue();
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
