/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a simple buffer for a blocking cache.
 */
#include <cstring>

#include "mem/cache/base_cache.hh"
#include "mem/cache/miss/blocking_buffer.hh"
#include "mem/cache/prefetch/base_prefetcher.hh"
#include "mem/request.hh"

/**
 * @todo Move writebacks into shared BaseBuffer class.
 */
void
BlockingBuffer::regStats(const std::string &name)
{
    MissBuffer::regStats(name);
}


void
BlockingBuffer::handleMiss(PacketPtr &pkt, int blk_size, Tick time)
{
    Addr blk_addr = pkt->getAddr() & ~(Addr)(blk_size - 1);
    if (pkt->isWrite() && (pkt->req->isUncacheable() || !writeAllocate ||
                               !pkt->needsResponse())) {
        if (!pkt->needsResponse()) {
            wb.allocateAsBuffer(pkt);
        } else {
            wb.allocate(pkt->cmd, blk_addr, blk_size, pkt);
        }

        std::memcpy(wb.pkt->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), blk_size);

        cache->setBlocked(Blocked_NoWBBuffers);
        cache->setMasterRequest(Request_WB, time);
        return;
    }

    if (!pkt->needsResponse()) {
        miss.allocateAsBuffer(pkt);
    } else {
        miss.allocate(pkt->cmd, blk_addr, blk_size, pkt);
    }
    if (!pkt->req->isUncacheable()) {
        miss.pkt->flags |= CACHE_LINE_FILL;
    }
    cache->setBlocked(Blocked_NoMSHRs);
    cache->setMasterRequest(Request_MSHR, time);
}

PacketPtr
BlockingBuffer::getPacket()
{
    if (miss.pkt && !miss.inService) {
        return miss.pkt;
    }
    return wb.pkt;
}

void
BlockingBuffer::setBusCmd(PacketPtr &pkt, MemCmd cmd)
{
    MSHR *mshr = (MSHR*) pkt->senderState;
    mshr->originalCmd = pkt->cmd;
    if (pkt->isCacheFill())
        pkt->cmdOverride(cmd);
}

void
BlockingBuffer::restoreOrigCmd(PacketPtr &pkt)
{
    pkt->cmdOverride(((MSHR*)(pkt->senderState))->originalCmd);
}

void
BlockingBuffer::markInService(PacketPtr &pkt, MSHR* mshr)
{
    if (!pkt->isCacheFill() && pkt->isWrite()) {
        // Forwarding a write/ writeback, don't need to change
        // the command
        assert(mshr == &wb);
        cache->clearMasterRequest(Request_WB);
        if (!pkt->needsResponse()) {
            assert(wb.getNumTargets() == 0);
            wb.deallocate();
            cache->clearBlocked(Blocked_NoWBBuffers);
        } else {
            wb.inService = true;
        }
    } else {
        assert(mshr == &miss);
        cache->clearMasterRequest(Request_MSHR);
        if (!pkt->needsResponse()) {
            assert(miss.getNumTargets() == 0);
            miss.deallocate();
            cache->clearBlocked(Blocked_NoMSHRs);
        } else {
            //mark in service
            miss.inService = true;
        }
    }
}

void
BlockingBuffer::handleResponse(PacketPtr &pkt, Tick time)
{
    if (pkt->isCacheFill()) {
        // targets were handled in the cache tags
        assert((MSHR*)pkt->senderState == &miss);
        miss.deallocate();
        cache->clearBlocked(Blocked_NoMSHRs);
    } else {
        if (((MSHR*)(pkt->senderState))->hasTargets()) {
            // Should only have 1 target if we had any
            assert(((MSHR*)(pkt->senderState))->getNumTargets() == 1);
            PacketPtr target = ((MSHR*)(pkt->senderState))->getTarget();
            ((MSHR*)(pkt->senderState))->popTarget();
            if (pkt->isRead()) {
                std::memcpy(target->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), target->getSize());
            }
            cache->respond(target, time);
            assert(!((MSHR*)(pkt->senderState))->hasTargets());
        }

        if (pkt->isWrite()) {
            assert(((MSHR*)(pkt->senderState)) == &wb);
            wb.deallocate();
            cache->clearBlocked(Blocked_NoWBBuffers);
        } else {
            miss.deallocate();
            cache->clearBlocked(Blocked_NoMSHRs);
        }
    }
}

void
BlockingBuffer::squash(int threadNum)
{
    if (miss.threadNum == threadNum) {
        PacketPtr target = miss.getTarget();
        miss.popTarget();
        assert(0/*target->req->getThreadNum()*/ == threadNum);
        target = NULL;
        assert(!miss.hasTargets());
        miss.ntargets=0;
        if (!miss.inService) {
            miss.deallocate();
            cache->clearBlocked(Blocked_NoMSHRs);
            cache->clearMasterRequest(Request_MSHR);
        }
    }
}

void
BlockingBuffer::doWriteback(Addr addr,
                            int size, uint8_t *data, bool compressed)
{
    // Generate request
    Request * req = new Request(addr, size, 0);
    PacketPtr pkt = new Packet(req, MemCmd::Writeback, -1);
    pkt->allocate();
    if (data) {
        std::memcpy(pkt->getPtr<uint8_t>(), data, size);
    }

    if (compressed) {
        pkt->flags |= COMPRESSED;
    }

    ///All writebacks charged to same thread @todo figure this out
    writebacks[0/*pkt->req->getThreadNum()*/]++;

    wb.allocateAsBuffer(pkt);
    cache->setMasterRequest(Request_WB, curTick);
    cache->setBlocked(Blocked_NoWBBuffers);
}



void
BlockingBuffer::doWriteback(PacketPtr &pkt)
{
    writebacks[0/*pkt->req->getThreadNum()*/]++;

    wb.allocateAsBuffer(pkt);

    // Since allocate as buffer copies the request,
    // need to copy data here.
    std::memcpy(wb.pkt->getPtr<uint8_t>(), pkt->getPtr<uint8_t>(), pkt->getSize());

    cache->setBlocked(Blocked_NoWBBuffers);
    cache->setMasterRequest(Request_WB, curTick);
}


MSHR *
BlockingBuffer::findMSHR(Addr addr)
{
    if (miss.addr == addr && miss.pkt)
        return &miss;
    return NULL;
}


bool
BlockingBuffer::findWrites(Addr addr, std::vector<MSHR*>& writes)
{
    if (wb.addr == addr && wb.pkt) {
        writes.push_back(&wb);
        return true;
    }
    return false;
}
