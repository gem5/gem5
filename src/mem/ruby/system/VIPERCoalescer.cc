/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#include "base/logging.hh"
#include "base/str.hh"
#include "config/the_isa.hh"

#if THE_ISA == X86_ISA
#include "arch/x86/insts/microldstop.hh"

#endif // X86_ISA
#include "mem/ruby/system/VIPERCoalescer.hh"

#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/GPUCoalescer.hh"
#include "debug/MemoryAccess.hh"
#include "mem/packet.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/structures/CacheMemory.hh"
#include "mem/ruby/system/GPUCoalescer.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/VIPERCoalescer.hh"

using namespace std;

VIPERCoalescer *
VIPERCoalescerParams::create()
{
    return new VIPERCoalescer(this);
}

VIPERCoalescer::VIPERCoalescer(const Params *p)
    : GPUCoalescer(p)
{
    m_max_wb_per_cycle=p->max_wb_per_cycle;
    m_max_inv_per_cycle=p->max_inv_per_cycle;
    m_outstanding_inv = 0;
    m_outstanding_wb = 0;
}

VIPERCoalescer::~VIPERCoalescer()
{
}

// Analyzes the packet to see if this request can be coalesced.
// If request can be coalesced, this request is added to the reqCoalescer table
// and makeRequest returns RequestStatus_Issued;
// If this is the first request to a cacheline, request is added to both
// newRequests queue and to the reqCoalescer table; makeRequest
// returns RequestStatus_Issued.
// If there is a pending request to this cacheline and this request
// can't be coalesced, RequestStatus_Aliased is returned and
// the packet needs to be reissued.
RequestStatus
VIPERCoalescer::makeRequest(PacketPtr pkt)
{
    if (m_outstanding_wb | m_outstanding_inv) {
        DPRINTF(GPUCoalescer,
                "There are %d Writebacks and %d Invalidatons\n",
                m_outstanding_wb, m_outstanding_inv);
    }
    // Are we in the middle of a release
    if ((m_outstanding_wb) > 0) {
        if (pkt->req->isKernel()) {
            // Everythign is fine
            // Barriers and Kernel End scan coalesce
            // If it is a Kerenl Begin flush the cache
            if (pkt->req->isAcquire() && (m_outstanding_inv == 0)) {
                invL1();
            }

            if (pkt->req->isRelease()) {
                insertKernel(pkt->req->contextId(), pkt);
            }

            return RequestStatus_Issued;
        }
//        return RequestStatus_Aliased;
    } else if (pkt->req->isKernel() && pkt->req->isRelease()) {
        // Flush Dirty Data on Kernel End
        // isKernel + isRelease
        insertKernel(pkt->req->contextId(), pkt);
        wbL1();
        if (m_outstanding_wb == 0) {
            for (auto it =  kernelEndList.begin(); it != kernelEndList.end(); it++) {
                newKernelEnds.push_back(it->first);
            }
            completeIssue();
        }
        return RequestStatus_Issued;
    }
    RequestStatus requestStatus = GPUCoalescer::makeRequest(pkt);
    if (requestStatus!=RequestStatus_Issued) {
        // Request not isssued
        // enqueue Retry
        DPRINTF(GPUCoalescer, "Request not issued by GPUCoaleser\n");
        return requestStatus;
    } else if (pkt->req->isKernel() && pkt->req->isAcquire()) {
        // Invalidate clean Data on Kernel Begin
        // isKernel + isAcquire
        invL1();
    } else if (pkt->req->isAcquire() && pkt->req->isRelease()) {
        // Deschedule the AtomicAcqRel and
        // Flush and Invalidate the L1 cache
        invwbL1();
        if (m_outstanding_wb > 0 && issueEvent.scheduled()) {
            DPRINTF(GPUCoalescer, "issueEvent Descheduled\n");
            deschedule(issueEvent);
        }
    } else if (pkt->req->isRelease()) {
        // Deschedule the StoreRel and
        // Flush the L1 cache
        wbL1();
        if (m_outstanding_wb > 0 && issueEvent.scheduled()) {
            DPRINTF(GPUCoalescer, "issueEvent Descheduled\n");
            deschedule(issueEvent);
        }
    } else if (pkt->req->isAcquire()) {
        // LoadAcq or AtomicAcq
        // Invalidate the L1 cache
        invL1();
    }
    // Request was successful
    if (m_outstanding_wb == 0) {
        if (!issueEvent.scheduled()) {
            DPRINTF(GPUCoalescer, "issueEvent Rescheduled\n");
            schedule(issueEvent, curTick());
        }
    }
    return RequestStatus_Issued;
}

void
VIPERCoalescer::wbCallback(Addr addr)
{
    m_outstanding_wb--;
    // if L1 Flush Complete
    // attemnpt to schedule issueEvent
    assert(((int) m_outstanding_wb) >= 0);
    if (m_outstanding_wb == 0) {
        for (auto it =  kernelEndList.begin(); it != kernelEndList.end(); it++) {
            newKernelEnds.push_back(it->first);
        }
        completeIssue();
    }
    trySendRetries();
}

void
VIPERCoalescer::invCallback(Addr addr)
{
    m_outstanding_inv--;
    // if L1 Flush Complete
    // attemnpt to schedule issueEvent
    // This probably won't happen, since
    // we dont wait on cache invalidations
    if (m_outstanding_wb == 0) {
        for (auto it =  kernelEndList.begin(); it != kernelEndList.end(); it++) {
            newKernelEnds.push_back(it->first);
        }
        completeIssue();
    }
    trySendRetries();
}

/**
  * Invalidate L1 cache (Acquire)
  */
void
VIPERCoalescer::invL1()
{
    int size = m_dataCache_ptr->getNumBlocks();
    DPRINTF(GPUCoalescer,
            "There are %d Invalidations outstanding before Cache Walk\n",
            m_outstanding_inv);
    // Walk the cache
    for (int i = 0; i < size; i++) {
        Addr addr = m_dataCache_ptr->getAddressAtIdx(i);
        // Evict Read-only data
        RubyRequestType request_type = RubyRequestType_REPLACEMENT;
        std::shared_ptr<RubyRequest> msg = std::make_shared<RubyRequest>(
            clockEdge(), addr, (uint8_t*) 0, 0, 0,
            request_type, RubyAccessMode_Supervisor,
            nullptr);
        assert(m_mandatory_q_ptr != NULL);
        Tick latency = cyclesToTicks(
                            m_controller->mandatoryQueueLatency(request_type));
        assert(latency > 0);
        m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
        m_outstanding_inv++;
    }
    DPRINTF(GPUCoalescer,
            "There are %d Invalidatons outstanding after Cache Walk\n",
            m_outstanding_inv);
}

/**
  * Writeback L1 cache (Release)
  */
void
VIPERCoalescer::wbL1()
{
    int size = m_dataCache_ptr->getNumBlocks();
    DPRINTF(GPUCoalescer,
            "There are %d Writebacks outstanding before Cache Walk\n",
            m_outstanding_wb);
    // Walk the cache
    for (int i = 0; i < size; i++) {
        Addr addr = m_dataCache_ptr->getAddressAtIdx(i);
        // Write dirty data back
        RubyRequestType request_type = RubyRequestType_FLUSH;
        std::shared_ptr<RubyRequest> msg = std::make_shared<RubyRequest>(
            clockEdge(), addr, (uint8_t*) 0, 0, 0,
            request_type, RubyAccessMode_Supervisor,
            nullptr);
        assert(m_mandatory_q_ptr != NULL);
        Tick latency = cyclesToTicks(
                            m_controller->mandatoryQueueLatency(request_type));
        assert(latency > 0);
        m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
        m_outstanding_wb++;
    }
    DPRINTF(GPUCoalescer,
            "There are %d Writebacks outstanding after Cache Walk\n",
            m_outstanding_wb);
}

/**
  * Invalidate and Writeback L1 cache (Acquire&Release)
  */
void
VIPERCoalescer::invwbL1()
{
    int size = m_dataCache_ptr->getNumBlocks();
    // Walk the cache
    for (int i = 0; i < size; i++) {
        Addr addr = m_dataCache_ptr->getAddressAtIdx(i);
        // Evict Read-only data
        RubyRequestType request_type = RubyRequestType_REPLACEMENT;
        std::shared_ptr<RubyRequest> msg = std::make_shared<RubyRequest>(
            clockEdge(), addr, (uint8_t*) 0, 0, 0,
            request_type, RubyAccessMode_Supervisor,
            nullptr);
        assert(m_mandatory_q_ptr != NULL);
        Tick latency = cyclesToTicks(
                            m_controller->mandatoryQueueLatency(request_type));
        assert(latency > 0);
        m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
        m_outstanding_inv++;
    }
    // Walk the cache
    for (int i = 0; i< size; i++) {
        Addr addr = m_dataCache_ptr->getAddressAtIdx(i);
        // Write dirty data back
        RubyRequestType request_type = RubyRequestType_FLUSH;
        std::shared_ptr<RubyRequest> msg = std::make_shared<RubyRequest>(
            clockEdge(), addr, (uint8_t*) 0, 0, 0,
            request_type, RubyAccessMode_Supervisor,
            nullptr);
        assert(m_mandatory_q_ptr != NULL);
        Tick latency = cyclesToTicks(
                m_controller->mandatoryQueueLatency(request_type));
        assert(latency > 0);
        m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
        m_outstanding_wb++;
    }
}
