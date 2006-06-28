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

#include "cpu/exec_context.hh"
#include "cpu/smt.hh" //for maxThreadsPerCPU
#include "mem/cache/base_cache.hh"
#include "mem/cache/miss/blocking_buffer.hh"
#include "mem/cache/prefetch/base_prefetcher.hh"
#include "sim/eventq.hh" // for Event declaration.

using namespace TheISA;

/**
 * @todo Move writebacks into shared BaseBuffer class.
 */
void
BlockingBuffer::regStats(const std::string &name)
{
    using namespace Stats;
    writebacks
        .init(maxThreadsPerCPU)
        .name(name + ".writebacks")
        .desc("number of writebacks")
        .flags(total)
        ;
}

void
BlockingBuffer::setCache(BaseCache *_cache)
{
    cache = _cache;
    blkSize = cache->getBlockSize();
}

void
BlockingBuffer::setPrefetcher(BasePrefetcher *_prefetcher)
{
    prefetcher = _prefetcher;
}
void
BlockingBuffer::handleMiss(Packet * &pkt, int blk_size, Tick time)
{
    Addr blk_addr = pkt->paddr & ~(Addr)(blk_size - 1);
    if (pkt->cmd.isWrite() && (pkt->req->isUncacheable() || !writeAllocate ||
                               pkt->cmd.isNoResponse())) {
        if (pkt->cmd.isNoResponse()) {
            wb.allocateAsBuffer(pkt);
        } else {
            wb.allocate(pkt->cmd, blk_addr, pkt->req->asid, blk_size, pkt);
        }
        if (cache->doData()) {
            memcpy(wb.pkt->data, pkt->data, blk_size);
        }
        cache->setBlocked(Blocked_NoWBBuffers);
        cache->setMasterRequest(Request_WB, time);
        return;
    }

    if (pkt->cmd.isNoResponse()) {
        miss.allocateAsBuffer(pkt);
    } else {
        miss.allocate(pkt->cmd, blk_addr, pkt->req->asid, blk_size, pkt);
    }
    if (!pkt->req->isUncacheable()) {
        miss.pkt->flags |= CACHE_LINE_FILL;
    }
    cache->setBlocked(Blocked_NoMSHRs);
    cache->setMasterRequest(Request_MSHR, time);
}

Packet *
BlockingBuffer::getPacket()
{
    if (miss.pkt && !miss.inService) {
        return miss.pkt;
    }
    return wb.pkt;
}

void
BlockingBuffer::setBusCmd(Packet * &pkt, Packet::Command cmd)
{
    MSHR *mshr = pkt->senderState;
    mshr->originalCmd = pkt->cmd;
    if (pkt->isCacheFill())
        pkt->cmd = cmd;
}

void
BlockingBuffer::restoreOrigCmd(Packet * &pkt)
{
    pkt->cmd = pkt->senderState->originalCmd;
}

void
BlockingBuffer::markInService(Packet * &pkt)
{
    if (!pkt->isCacheFill() && pkt->cmd.isWrite()) {
        // Forwarding a write/ writeback, don't need to change
        // the command
        assert(pkt->senderState == &wb);
        cache->clearMasterRequest(Request_WB);
        if (pkt->cmd.isNoResponse()) {
            assert(wb.getNumTargets() == 0);
            wb.deallocate();
            cache->clearBlocked(Blocked_NoWBBuffers);
        } else {
            wb.inService = true;
        }
    } else {
        assert(pkt->senderState == &miss);
        cache->clearMasterRequest(Request_MSHR);
        if (pkt->cmd.isNoResponse()) {
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
BlockingBuffer::handleResponse(Packet * &pkt, Tick time)
{
    if (pkt->isCacheFill()) {
        // targets were handled in the cache tags
        assert(pkt->senderState == &miss);
        miss.deallocate();
        cache->clearBlocked(Blocked_NoMSHRs);
    } else {
        if (pkt->senderState->hasTargets()) {
            // Should only have 1 target if we had any
            assert(pkt->senderState->getNumTargets() == 1);
            Packet * target = pkt->senderState->getTarget();
            pkt->senderState->popTarget();
            if (cache->doData() && pkt->cmd.isRead()) {
                memcpy(target->data, pkt->data, target->size);
            }
            cache->respond(target, time);
            assert(!pkt->senderState->hasTargets());
        }

        if (pkt->cmd.isWrite()) {
            assert(pkt->senderState == &wb);
            wb.deallocate();
            cache->clearBlocked(Blocked_NoWBBuffers);
        } else {
            miss.deallocate();
            cache->clearBlocked(Blocked_NoMSHRs);
        }
    }
}

void
BlockingBuffer::squash(int req->getThreadNum()ber)
{
    if (miss.setThreadNum() == req->getThreadNum()ber) {
        Packet * target = miss.getTarget();
        miss.popTarget();
        assert(target->req->setThreadNum() == req->getThreadNum()ber);
        if (target->completionEvent != NULL) {
            delete target->completionEvent;
        }
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
BlockingBuffer::doWriteback(Addr addr, int asid,
                            int size, uint8_t *data, bool compressed)
{

    // Generate request
    Packet * pkt = new Packet();
    pkt->paddr = addr;
    pkt->req->asid = asid;
    pkt->size = size;
    pkt->data = new uint8_t[size];
    if (data) {
        memcpy(pkt->data, data, size);
    }
    /**
     * @todo Need to find a way to charge the writeback to the "correct"
     * thread.
     */
    pkt->req->setThreadNum() = 0;

    pkt->cmd = Writeback;
    if (compressed) {
        pkt->flags |= COMPRESSED;
    }

    writebacks[pkt->req->getThreadNum()]++;

    wb.allocateAsBuffer(pkt);
    cache->setMasterRequest(Request_WB, curTick);
    cache->setBlocked(Blocked_NoWBBuffers);
}



void
BlockingBuffer::doWriteback(Packet * &pkt)
{
    writebacks[pkt->req->getThreadNum()]++;

    wb.allocateAsBuffer(pkt);

    // Since allocate as buffer copies the request,
    // need to copy data here.
    if (cache->doData()) {
        memcpy(wb.pkt->data, pkt->data, pkt->size);
    }
    cache->setBlocked(Blocked_NoWBBuffers);
    cache->setMasterRequest(Request_WB, curTick);
}
