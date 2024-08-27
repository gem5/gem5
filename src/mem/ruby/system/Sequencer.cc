/*
 * Copyright (c) 2019-2021 ARM Limited
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "mem/ruby/system/Sequencer.hh"

#include "arch/x86/ldstflags.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/str.hh"
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/LLSC.hh"
#include "debug/MemoryAccess.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubyHitMiss.hh"
#include "debug/RubySequencer.hh"
#include "debug/RubyStats.hh"
#include "mem/packet.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

Sequencer::Sequencer(const Params &p)
    : RubyPort(p), m_IncompleteTimes(MachineType_NUM),
      deadlockCheckEvent([this]{ wakeup(); }, "Sequencer deadlock check")
{
    m_outstanding_count = 0;

    m_dataCache_ptr = p.dcache;
    m_max_outstanding_requests = p.max_outstanding_requests;
    m_deadlock_threshold = p.deadlock_threshold;

    m_coreId = p.coreid; // for tracking the two CorePair sequencers
    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);

    m_unaddressedTransactionCnt = 0;

    m_runningGarnetStandalone = p.garnet_standalone;

    m_num_pending_invs = 0;
    m_cache_inv_pkt = nullptr;

    // These statistical variables are not for display.
    // The profiler will collate these across different
    // sequencers and display those collated statistics.
    m_outstandReqHist.init(10);
    m_latencyHist.init(10);
    m_hitLatencyHist.init(10);
    m_missLatencyHist.init(10);

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist.push_back(new statistics::Histogram());
        m_typeLatencyHist[i]->init(10);

        m_hitTypeLatencyHist.push_back(new statistics::Histogram());
        m_hitTypeLatencyHist[i]->init(10);

        m_missTypeLatencyHist.push_back(new statistics::Histogram());
        m_missTypeLatencyHist[i]->init(10);
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_hitMachLatencyHist.push_back(new statistics::Histogram());
        m_hitMachLatencyHist[i]->init(10);

        m_missMachLatencyHist.push_back(new statistics::Histogram());
        m_missMachLatencyHist[i]->init(10);

        m_IssueToInitialDelayHist.push_back(new statistics::Histogram());
        m_IssueToInitialDelayHist[i]->init(10);

        m_InitialToForwardDelayHist.push_back(new statistics::Histogram());
        m_InitialToForwardDelayHist[i]->init(10);

        m_ForwardToFirstResponseDelayHist.push_back(
            new statistics::Histogram());
        m_ForwardToFirstResponseDelayHist[i]->init(10);

        m_FirstResponseToCompletionDelayHist.push_back(
            new statistics::Histogram());
        m_FirstResponseToCompletionDelayHist[i]->init(10);
    }

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_hitTypeMachLatencyHist.push_back(
            std::vector<statistics::Histogram *>());
        m_missTypeMachLatencyHist.push_back(
            std::vector<statistics::Histogram *>());

        for (int j = 0; j < MachineType_NUM; j++) {
            m_hitTypeMachLatencyHist[i].push_back(new statistics::Histogram());
            m_hitTypeMachLatencyHist[i][j]->init(10);

            m_missTypeMachLatencyHist[i].push_back(
                new statistics::Histogram());
            m_missTypeMachLatencyHist[i][j]->init(10);
        }
    }

}

Sequencer::~Sequencer()
{
}

void
Sequencer::llscLoadLinked(const Addr claddr)
{
    fatal_if(m_dataCache_ptr == NULL,
        "%s must have a dcache object to support LLSC requests.", name());
    AbstractCacheEntry *line = m_dataCache_ptr->lookup(claddr);
    if (line) {
        line->setLocked(m_version);
        DPRINTF(LLSC, "LLSC Monitor - inserting load linked - "
                      "addr=0x%lx - cpu=%u\n", claddr, m_version);
    }
}

void
Sequencer::llscClearMonitor(const Addr claddr)
{
    // clear monitor is called for all stores and evictions
    if (m_dataCache_ptr == NULL)
        return;
    AbstractCacheEntry *line = m_dataCache_ptr->lookup(claddr);
    if (line && line->isLocked(m_version)) {
        line->clearLocked();
        DPRINTF(LLSC, "LLSC Monitor - clearing due to store - "
                      "addr=0x%lx - cpu=%u\n", claddr, m_version);
    }
}

bool
Sequencer::llscStoreConditional(const Addr claddr)
{
    fatal_if(m_dataCache_ptr == NULL,
        "%s must have a dcache object to support LLSC requests.", name());
    AbstractCacheEntry *line = m_dataCache_ptr->lookup(claddr);
    if (!line)
        return false;

    DPRINTF(LLSC, "LLSC Monitor - clearing due to "
                  "store conditional - "
                  "addr=0x%lx - cpu=%u\n",
                  claddr, m_version);

    if (line->isLocked(m_version)) {
        line->clearLocked();
        return true;
    } else {
        line->clearLocked();
        return false;
    }
}

bool
Sequencer::llscCheckMonitor(const Addr address)
{
    assert(m_dataCache_ptr != NULL);
    const Addr claddr = makeLineAddress(address);
    AbstractCacheEntry *line = m_dataCache_ptr->lookup(claddr);
    if (!line)
        return false;

    if (line->isLocked(m_version)) {
        return true;
    } else {
        return false;
    }
}

void
Sequencer::llscClearLocalMonitor()
{
    m_dataCache_ptr->clearLockedAll(m_version);
}

void
Sequencer::wakeup()
{
    assert(drainState() != DrainState::Draining);

    // Check for deadlock of any of the requests
    Cycles current_time = curCycle();

    // Check across all outstanding requests
    [[maybe_unused]] int total_outstanding = 0;

    for (const auto &table_entry : m_RequestTable) {
        for (const auto &seq_req : table_entry.second) {
            if (current_time - seq_req.issue_time < m_deadlock_threshold)
                continue;

            panic("Possible Deadlock detected. Aborting!\n version: %d "
                  "request.paddr: 0x%x m_readRequestTable: %d current time: "
                  "%u issue_time: %d difference: %d\n", m_version,
                  seq_req.pkt->getAddr(), table_entry.second.size(),
                  current_time * clockPeriod(), seq_req.issue_time
                  * clockPeriod(), (current_time * clockPeriod())
                  - (seq_req.issue_time * clockPeriod()));
        }
        total_outstanding += table_entry.second.size();
    }

    assert(m_outstanding_count == total_outstanding);

    if (m_outstanding_count > 0) {
        // If there are still outstanding requests, keep checking
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }
}

int
Sequencer::functionalWrite(Packet *func_pkt)
{
    int num_written = RubyPort::functionalWrite(func_pkt);

    for (const auto &table_entry : m_RequestTable) {
        for (const auto& seq_req : table_entry.second) {
            if (seq_req.functionalWrite(func_pkt))
                ++num_written;
        }
    }
    // Functional writes to addresses being monitored
    // will fail (remove) the monitor entry.
    llscClearMonitor(makeLineAddress(func_pkt->getAddr()));

    return num_written;
}

void Sequencer::resetStats()
{
    m_outstandReqHist.reset();
    m_latencyHist.reset();
    m_hitLatencyHist.reset();
    m_missLatencyHist.reset();
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist[i]->reset();
        m_hitTypeLatencyHist[i]->reset();
        m_missTypeLatencyHist[i]->reset();
        for (int j = 0; j < MachineType_NUM; j++) {
            m_hitTypeMachLatencyHist[i][j]->reset();
            m_missTypeMachLatencyHist[i][j]->reset();
        }
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_missMachLatencyHist[i]->reset();
        m_hitMachLatencyHist[i]->reset();

        m_IssueToInitialDelayHist[i]->reset();
        m_InitialToForwardDelayHist[i]->reset();
        m_ForwardToFirstResponseDelayHist[i]->reset();
        m_FirstResponseToCompletionDelayHist[i]->reset();

        m_IncompleteTimes[i] = 0;
    }
}

// Insert the request in the request table. Return RequestStatus_Aliased
// if the entry was already present.
RequestStatus
Sequencer::insertRequest(PacketPtr pkt, RubyRequestType primary_type,
                         RubyRequestType secondary_type)
{
    // See if we should schedule a deadlock check
    if (!deadlockCheckEvent.scheduled() &&
        drainState() != DrainState::Draining) {
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }

    if (isTlbiCmdRequest(primary_type)) {
        assert(primary_type == secondary_type);

        switch (primary_type) {
        case RubyRequestType_TLBI_EXT_SYNC_COMP:
            // Don't have to store any data on this
            break;
        case RubyRequestType_TLBI:
        case RubyRequestType_TLBI_SYNC:
            {
                incrementUnaddressedTransactionCnt();

                // returns pair<inserted element, was inserted>
                [[maybe_unused]] auto insert_data = \
                    m_UnaddressedRequestTable.emplace(
                        getCurrentUnaddressedTransactionID(),
                        SequencerRequest(
                            pkt, primary_type, secondary_type, curCycle()));

                // if insert_data.second is false, wasn't inserted
                assert(insert_data.second &&
                       "Another TLBI request with the same ID exists");

                DPRINTF(RubySequencer, "Inserting TLBI request %016x\n",
                        getCurrentUnaddressedTransactionID());

                break;
            }

        default:
            panic("Unexpected TLBI RubyRequestType");
        }

        return RequestStatus_Ready;
    }

    // If command is MemSyncReq, it is used to invalidate the cache.
    // As the cache invalidation requests are already issued in invL1(),
    // there is no need to create a new request for the same here.
    // Instead, return RequestStatus_Aliased, and make the sequencer skip
    // an extra issueRequest
    if (pkt->cmd == MemCmd::MemSyncReq) {
        return RequestStatus_Aliased;
    }

    Addr line_addr = makeLineAddress(pkt->getAddr());
    // Check if there is any outstanding request for the same cache line.
    auto &seq_req_list = m_RequestTable[line_addr];
    // Create a default entry
    seq_req_list.emplace_back(pkt, primary_type,
        secondary_type, curCycle());
    m_outstanding_count++;

    if (seq_req_list.size() > 1) {
        return RequestStatus_Aliased;
    }

    m_outstandReqHist.sample(m_outstanding_count);

    return RequestStatus_Ready;
}

void
Sequencer::markRemoved()
{
    m_outstanding_count--;
}

void
Sequencer::recordMissLatency(SequencerRequest* srequest, bool llscSuccess,
                             const MachineType respondingMach,
                             bool isExternalHit, Cycles initialRequestTime,
                             Cycles forwardRequestTime,
                             Cycles firstResponseTime)
{
    RubyRequestType type = srequest->m_type;
    Cycles issued_time = srequest->issue_time;
    Cycles completion_time = curCycle();

    assert(curCycle() >= issued_time);
    Cycles total_lat = completion_time - issued_time;

    if ((initialRequestTime != 0) && (initialRequestTime < issued_time)) {
        // if the request was combined in the protocol with an earlier request
        // for the same address, it is possible that it will return an
        // initialRequestTime corresponding the earlier request.  Since Cycles
        // is unsigned, we can't let this request get profiled below.

        total_lat = Cycles(0);
    }

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %d cycles\n",
             curTick(), m_version, "Seq", llscSuccess ? "Done" : "SC_Failed",
             "", "", printAddress(srequest->pkt->getAddr()), total_lat);

    m_latencyHist.sample(total_lat);
    m_typeLatencyHist[type]->sample(total_lat);

    if (isExternalHit) {
        m_missLatencyHist.sample(total_lat);
        m_missTypeLatencyHist[type]->sample(total_lat);

        if (respondingMach != MachineType_NUM) {
            m_missMachLatencyHist[respondingMach]->sample(total_lat);
            m_missTypeMachLatencyHist[type][respondingMach]->sample(total_lat);

            if ((issued_time <= initialRequestTime) &&
                (initialRequestTime <= forwardRequestTime) &&
                (forwardRequestTime <= firstResponseTime) &&
                (firstResponseTime <= completion_time)) {

                m_IssueToInitialDelayHist[respondingMach]->sample(
                    initialRequestTime - issued_time);
                m_InitialToForwardDelayHist[respondingMach]->sample(
                    forwardRequestTime - initialRequestTime);
                m_ForwardToFirstResponseDelayHist[respondingMach]->sample(
                    firstResponseTime - forwardRequestTime);
                m_FirstResponseToCompletionDelayHist[respondingMach]->sample(
                    completion_time - firstResponseTime);
            } else {
                m_IncompleteTimes[respondingMach]++;
            }
        }
    } else {
        m_hitLatencyHist.sample(total_lat);
        m_hitTypeLatencyHist[type]->sample(total_lat);

        if (respondingMach != MachineType_NUM) {
            m_hitMachLatencyHist[respondingMach]->sample(total_lat);
            m_hitTypeMachLatencyHist[type][respondingMach]->sample(total_lat);
        }
    }
}

void
Sequencer::writeCallbackScFail(Addr address, DataBlock& data)
{
    llscClearMonitor(address);
    writeCallback(address, data);
}

void
Sequencer::writeCallback(Addr address, DataBlock& data,
                         const bool externalHit, const MachineType mach,
                         const Cycles initialRequestTime,
                         const Cycles forwardRequestTime,
                         const Cycles firstResponseTime,
                         const bool noCoales)
{
    //
    // Free the whole list as we assume we have had the exclusive access
    // to this cache line when response for the write comes back
    //
    assert(address == makeLineAddress(address));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

    // Perform hitCallback on every cpu request made to this cache block while
    // ruby request was outstanding. Since only 1 ruby request was made,
    // profile the ruby latency once.
    bool ruby_request = true;
    while (!seq_req_list.empty()) {
        SequencerRequest &seq_req = seq_req_list.front();
        // Atomic Request may be executed remotly in the cache hierarchy
        bool atomic_req =
           ((seq_req.m_type == RubyRequestType_ATOMIC_RETURN) ||
            (seq_req.m_type == RubyRequestType_ATOMIC_NO_RETURN));

        if ((noCoales || atomic_req) && !ruby_request) {
            // Do not process follow-up requests
            // (e.g. if full line no present)
            // Reissue to the cache hierarchy
            issueRequest(seq_req.pkt, seq_req.m_second_type);
            break;
        }

        if (ruby_request) {
            assert(seq_req.m_type != RubyRequestType_LD);
            assert(seq_req.m_type != RubyRequestType_Load_Linked);
            assert(seq_req.m_type != RubyRequestType_IFETCH);
            assert(seq_req.m_type != RubyRequestType_ATOMIC_RETURN);
            assert(seq_req.m_type != RubyRequestType_ATOMIC_NO_RETURN);
        }

        // handle write request
        if ((seq_req.m_type != RubyRequestType_LD) &&
            (seq_req.m_type != RubyRequestType_Load_Linked) &&
            (seq_req.m_type != RubyRequestType_IFETCH)) {
            // LL/SC support (tested with ARMv8)
            bool success = true;

            if (seq_req.m_type != RubyRequestType_Store_Conditional) {
                // Regular stores to addresses being monitored
                // will fail (remove) the monitor entry.
                llscClearMonitor(address);
            } else {
                // Store conditionals must first check the monitor
                // if they will succeed or not
                success = llscStoreConditional(address);
                seq_req.pkt->req->setExtraData(success ? 1 : 0);
            }

            // Handle SLICC block_on behavior for Locked_RMW accesses. NOTE: the
            // address variable here is assumed to be a line address, so when
            // blocking buffers, must check line addresses.
            if (seq_req.m_type == RubyRequestType_Locked_RMW_Read) {
                // blockOnQueue blocks all first-level cache controller queues
                // waiting on memory accesses for the specified address that go
                // to the specified queue. In this case, a Locked_RMW_Write must
                // go to the mandatory_q before unblocking the first-level
                // controller. This will block standard loads, stores, ifetches,
                // etc.
                m_controller->blockOnQueue(address, m_mandatory_q_ptr);
            } else if (seq_req.m_type == RubyRequestType_Locked_RMW_Write) {
                m_controller->unblock(address);
            }

            if (ruby_request) {
                recordMissLatency(&seq_req, success, mach, externalHit,
                                  initialRequestTime, forwardRequestTime,
                                  firstResponseTime);
            }

            markRemoved();
            hitCallback(&seq_req, data, success, mach, externalHit,
                        initialRequestTime, forwardRequestTime,
                        firstResponseTime, !ruby_request);
            ruby_request = false;
        } else {
            // handle read request
            assert(!ruby_request);
            markRemoved();
            hitCallback(&seq_req, data, true, mach, externalHit,
                        initialRequestTime, forwardRequestTime,
                        firstResponseTime, !ruby_request);
        }
        seq_req_list.pop_front();
    }

    // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
}

bool
Sequencer::processReadCallback(SequencerRequest &seq_req,
                               DataBlock& data,
                               const bool ruby_request,
                               bool externalHit,
                               const MachineType mach,
                               Cycles initialRequestTime,
                               Cycles forwardRequestTime,
                               Cycles firstResponseTime)
{
    if (ruby_request) {
        assert((seq_req.m_type == RubyRequestType_LD) ||
               (seq_req.m_type == RubyRequestType_Load_Linked) ||
               (seq_req.m_type == RubyRequestType_IFETCH));
    }
    if ((seq_req.m_type != RubyRequestType_LD) &&
        (seq_req.m_type != RubyRequestType_Load_Linked) &&
        (seq_req.m_type != RubyRequestType_IFETCH) &&
        (seq_req.m_type != RubyRequestType_REPLACEMENT)) {
        // Write request: reissue request to the cache hierarchy
        issueRequest(seq_req.pkt, seq_req.m_second_type);
        return true;
    }
    return false;

}

void
Sequencer::readCallback(Addr address, DataBlock& data,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    //
    // Free up read requests until we hit the first Write request
    // or end of the corresponding list.
    //
    assert(address == makeLineAddress(address));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

    // Perform hitCallback on every cpu request made to this cache block while
    // ruby request was outstanding. Since only 1 ruby request was made,
    // profile the ruby latency once.
    bool ruby_request = true;
    while (!seq_req_list.empty()) {
        SequencerRequest &seq_req = seq_req_list.front();
        if (processReadCallback(seq_req, data, ruby_request, externalHit, mach,
                                initialRequestTime, forwardRequestTime,
                                firstResponseTime)) {
            break;
        }
        if (ruby_request) {
            recordMissLatency(&seq_req, true, mach, externalHit,
                              initialRequestTime, forwardRequestTime,
                              firstResponseTime);
        }
        markRemoved();
        hitCallback(&seq_req, data, true, mach, externalHit,
                    initialRequestTime, forwardRequestTime,
                    firstResponseTime, !ruby_request);
        ruby_request = false;
        seq_req_list.pop_front();
    }

    // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
}

void
Sequencer::atomicCallback(Addr address, DataBlock& data,
                         const bool externalHit, const MachineType mach,
                         const Cycles initialRequestTime,
                         const Cycles forwardRequestTime,
                         const Cycles firstResponseTime)
{
    //
    // Free the first request (an atomic operation) from the list.
    // Then issue the next request to ruby system as we cannot
    // assume the cache line is present in the cache
    // (the opperation could be performed remotly)
    //
    assert(address == makeLineAddress(address));
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    auto &seq_req_list = m_RequestTable[address];

    // Perform hitCallback only on the first cpu request that
    // issued the ruby request
    bool ruby_request = true;
    while (!seq_req_list.empty()) {
        SequencerRequest &seq_req = seq_req_list.front();

        if (ruby_request) {
            // Check that the request was an atomic memory operation
            // and record the latency
            assert((seq_req.m_type == RubyRequestType_ATOMIC_RETURN) ||
                   (seq_req.m_type == RubyRequestType_ATOMIC_NO_RETURN));
            recordMissLatency(&seq_req, true, mach, externalHit,
                              initialRequestTime, forwardRequestTime,
                              firstResponseTime);
        } else {
            // Read, Write or Atomic request:
            // reissue request to the cache hierarchy
            // (we don't know if op was performed remotly)
            issueRequest(seq_req.pkt, seq_req.m_second_type);
            break;
        }

        // Atomics clean the monitor entry
        llscClearMonitor(address);

        markRemoved();
        ruby_request = false;
        hitCallback(&seq_req, data, true, mach, externalHit,
                    initialRequestTime, forwardRequestTime,
                    firstResponseTime, false);
        seq_req_list.pop_front();
    }

    // free all outstanding requests corresponding to this address
    if (seq_req_list.empty()) {
        m_RequestTable.erase(address);
    }
}

void
Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data,
                       bool llscSuccess,
                       const MachineType mach, const bool externalHit,
                       const Cycles initialRequestTime,
                       const Cycles forwardRequestTime,
                       const Cycles firstResponseTime,
                       const bool was_coalesced)
{
    warn_once("Replacement policy updates recently became the responsibility "
              "of SLICC state machines. Make sure to setMRU() near callbacks "
              "in .sm files!");

    PacketPtr pkt = srequest->pkt;
    Addr request_address(pkt->getAddr());
    RubyRequestType type = srequest->m_type;

    if (was_coalesced) {
        // Notify the controller about a coalesced request so it can properly
        // account for it in its hit/miss stats and/or train prefetchers
        // (this is protocol-dependent)
        m_controller->notifyCoalesced(request_address, type, pkt->req,
                                      data, externalHit);
    }

    // Load-linked handling
    if (type == RubyRequestType_Load_Linked) {
        Addr line_addr = makeLineAddress(request_address);
        llscLoadLinked(line_addr);
    }

    DPRINTF(RubyHitMiss, "Cache %s at %#x\n",
                         externalHit ? "miss" : "hit",
                         printAddress(request_address));

    // update the data unless it is a non-data-carrying flush
    if (RubySystem::getWarmupEnabled()) {
        data.setData(pkt);
    } else if (!pkt->isFlush()) {
        if ((type == RubyRequestType_LD) ||
            (type == RubyRequestType_IFETCH) ||
            (type == RubyRequestType_RMW_Read) ||
            (type == RubyRequestType_Locked_RMW_Read) ||
            (type == RubyRequestType_Load_Linked) ||
            (type == RubyRequestType_ATOMIC_RETURN)) {
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));

           if (type == RubyRequestType_ATOMIC_RETURN) {
               DPRINTF(RubySequencer, "ATOMIC RETURN data %s\n", data);
           } else {
               DPRINTF(RubySequencer, "read data %s\n", data);
           }
        } else if (pkt->req->isSwap()) {
            assert(!pkt->isMaskedWrite());
            std::vector<uint8_t> overwrite_val(pkt->getSize());
            pkt->writeData(&overwrite_val[0]);
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            data.setData(&overwrite_val[0],
                         getOffset(request_address), pkt->getSize());
            DPRINTF(RubySequencer, "swap data %s\n", data);
        } else if (pkt->isAtomicOp()) {
            // Set the data in the packet to the old value in the cache
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
            DPRINTF(RubySequencer, "AMO original data %s\n", data);
            // execute AMO operation
            (*(pkt->getAtomicOp()))(
                data.getDataMod(getOffset(request_address)));
            DPRINTF(RubySequencer, "AMO new data %s\n", data);
        } else if (type != RubyRequestType_Store_Conditional || llscSuccess) {
            // Types of stores set the actual data here, apart from
            // failed Store Conditional requests
            data.setData(pkt);
            DPRINTF(RubySequencer, "set data %s\n", data);
        }
    }

    // If using the RubyTester, update the RubyTester sender state's
    // subBlock with the recieved data.  The tester will later access
    // this state.
    if (m_usingRubyTester) {
        DPRINTF(RubySequencer, "hitCallback %s 0x%x using RubyTester\n",
                pkt->cmdString(), pkt->getAddr());
        RubyTester::SenderState* testerSenderState =
            pkt->findNextSenderState<RubyTester::SenderState>();
        assert(testerSenderState);
        testerSenderState->subBlock.mergeFrom(data);
    }

    RubySystem *rs = m_ruby_system;
    if (RubySystem::getWarmupEnabled()) {
        assert(pkt->req);
        delete pkt;
        rs->m_cache_recorder->enqueueNextFetchRequest();
    } else if (RubySystem::getCooldownEnabled()) {
        delete pkt;
        rs->m_cache_recorder->enqueueNextFlushRequest();
    } else {
        ruby_hit_callback(pkt);
        testDrainComplete();
    }
}

void
Sequencer::unaddressedCallback(Addr unaddressedReqId,
                               RubyRequestType reqType,
                               const MachineType mach,
                               const Cycles initialRequestTime,
                               const Cycles forwardRequestTime,
                               const Cycles firstResponseTime)
{
    DPRINTF(RubySequencer, "unaddressedCallback ID:%08x type:%d\n",
            unaddressedReqId, reqType);

    switch (reqType) {
      case RubyRequestType_TLBI_EXT_SYNC:
      {
        // This should trigger the CPU to wait for stale translations
        // and send an EXT_SYNC_COMP once complete.

        // Don't look for the ID in our requestTable.
        // It won't be there because we didn't request this Sync
        ruby_stale_translation_callback(unaddressedReqId);
        break;
      }
      case RubyRequestType_TLBI:
      case RubyRequestType_TLBI_SYNC:
      {
        // These signal that a TLBI operation that this core initiated
        // of the respective type (TLBI or Sync) has finished.

        assert(m_UnaddressedRequestTable.find(unaddressedReqId)
               != m_UnaddressedRequestTable.end());

        {
            SequencerRequest &seq_req =
                m_UnaddressedRequestTable.at(unaddressedReqId);
            assert(seq_req.m_type == reqType);

            PacketPtr pkt = seq_req.pkt;

            ruby_unaddressed_callback(pkt);
            testDrainComplete();
        }

        m_UnaddressedRequestTable.erase(unaddressedReqId);
        break;
      }
      default:
        panic("Unexpected TLBI RubyRequestType");
    }
}

void
Sequencer::completeHitCallback(std::vector<PacketPtr> & mylist)
{
    for (auto& pkt : mylist) {
        // When Ruby is in warmup or cooldown phase, the requests come
        // from the cache recorder. They do not track which port to use
        // and do not need to send the response back
        if (!RubySystem::getWarmupEnabled()
                && !RubySystem::getCooldownEnabled()) {
            RubyPort::SenderState *ss =
                safe_cast<RubyPort::SenderState *>(pkt->senderState);
            MemResponsePort *port = ss->port;
            assert(port != NULL);

            pkt->senderState = ss->predecessor;

            if (pkt->cmd != MemCmd::WriteReq) {
                // for WriteReq, we keep the original senderState until
                // writeCompleteCallback
                delete ss;
            }

            port->hitCallback(pkt);
            trySendRetries();
        }
    }

    RubySystem *rs = m_ruby_system;
    if (RubySystem::getWarmupEnabled()) {
        rs->m_cache_recorder->enqueueNextFetchRequest();
    } else if (RubySystem::getCooldownEnabled()) {
        rs->m_cache_recorder->enqueueNextFlushRequest();
    } else {
        testDrainComplete();
    }
}

void
Sequencer::invL1Callback()
{
    // Since L1 invalidate is currently done with paddr = 0
    assert(m_cache_inv_pkt && m_num_pending_invs > 0);

    m_num_pending_invs--;

    if (m_num_pending_invs == 0) {
        std::vector<PacketPtr> pkt_list { m_cache_inv_pkt };
        m_cache_inv_pkt = nullptr;
        completeHitCallback(pkt_list);
    }
}

void
Sequencer::invL1()
{
    int size = m_dataCache_ptr->getNumBlocks();
    DPRINTF(RubySequencer,
            "There are %d Invalidations outstanding before Cache Walk\n",
            m_num_pending_invs);
    // Walk the cache
    for (int i = 0; i < size; i++) {
        Addr addr = m_dataCache_ptr->getAddressAtIdx(i);
        // Evict Read-only data
        RubyRequestType request_type = RubyRequestType_REPLACEMENT;
        std::shared_ptr<RubyRequest> msg = std::make_shared<RubyRequest>(
            clockEdge(), addr, 0, 0,
            request_type, RubyAccessMode_Supervisor,
            nullptr);
        DPRINTF(RubySequencer, "Evicting addr 0x%x\n", addr);
        assert(m_mandatory_q_ptr != NULL);
        Tick latency = cyclesToTicks(
            m_controller->mandatoryQueueLatency(request_type));
        m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
        m_num_pending_invs++;
    }
    DPRINTF(RubySequencer,
            "There are %d Invalidations outstanding after Cache Walk\n",
            m_num_pending_invs);
}

bool
Sequencer::empty() const
{
    return m_RequestTable.empty() &&
           m_UnaddressedRequestTable.empty();
}

RequestStatus
Sequencer::makeRequest(PacketPtr pkt)
{
    // HTM abort signals must be allowed to reach the Sequencer
    // the same cycle they are issued. They cannot be retried.
    if ((m_outstanding_count >= m_max_outstanding_requests) &&
        !pkt->req->isHTMAbort()) {
        return RequestStatus_BufferFull;
    }

    RubyRequestType primary_type = RubyRequestType_NULL;
    RubyRequestType secondary_type = RubyRequestType_NULL;

    if (pkt->isLLSC()) {
        // LL/SC instructions need to be handled carefully by the cache
        // coherence protocol to ensure they follow the proper semantics. In
        // particular, by identifying the operations as atomic, the protocol
        // should understand that migratory sharing optimizations should not
        // be performed (i.e. a load between the LL and SC should not steal
        // away exclusive permission).
        //
        // The following logic works correctly with the semantics
        // of armV8 LDEX/STEX instructions.

        if (pkt->isWrite()) {
            DPRINTF(RubySequencer, "Issuing SC\n");
            primary_type = RubyRequestType_Store_Conditional;
#if defined (PROTOCOL_MESI_Three_Level) || defined (PROTOCOL_MESI_Three_Level_HTM)
            secondary_type = RubyRequestType_Store_Conditional;
#else
            secondary_type = RubyRequestType_ST;
#endif
        } else {
            DPRINTF(RubySequencer, "Issuing LL\n");
            assert(pkt->isRead());
            primary_type = RubyRequestType_Load_Linked;
            secondary_type = RubyRequestType_LD;
        }
    } else if (pkt->req->isLockedRMW()) {
        //
        // x86 locked instructions are translated to store cache coherence
        // requests because these requests should always be treated as read
        // exclusive operations and should leverage any migratory sharing
        // optimization built into the protocol.
        //
        if (pkt->isWrite()) {
            DPRINTF(RubySequencer, "Issuing Locked RMW Write\n");
            primary_type = RubyRequestType_Locked_RMW_Write;
        } else {
            DPRINTF(RubySequencer, "Issuing Locked RMW Read\n");
            assert(pkt->isRead());
            primary_type = RubyRequestType_Locked_RMW_Read;
        }
        secondary_type = RubyRequestType_ST;
    } else if (pkt->req->isTlbiCmd()) {
        primary_type = secondary_type = tlbiCmdToRubyRequestType(pkt);
        DPRINTF(RubySequencer, "Issuing TLBI\n");
#if defined (PROTOCOL_CHI)
    } else if (pkt->isAtomicOp()) {
        if (pkt->req->isAtomicReturn()){
            DPRINTF(RubySequencer, "Issuing ATOMIC RETURN \n");
            primary_type = secondary_type =
                           RubyRequestType_ATOMIC_RETURN;
        } else {
            DPRINTF(RubySequencer, "Issuing ATOMIC NO RETURN\n");
            primary_type = secondary_type =
                           RubyRequestType_ATOMIC_NO_RETURN;

        }
#endif
    } else if (pkt->req->hasNoAddr()) {
        primary_type = secondary_type = RubyRequestType_hasNoAddr;
    } else {
        //
        // To support SwapReq, we need to check isWrite() first: a SwapReq
        // should always be treated like a write, but since a SwapReq implies
        // both isWrite() and isRead() are true, check isWrite() first here.
        //
        if (pkt->isWrite()) {
            //
            // Note: M5 packets do not differentiate ST from RMW_Write
            //
            primary_type = secondary_type = RubyRequestType_ST;
        } else if (pkt->isRead()) {
            // hardware transactional memory commands
            if (pkt->req->isHTMCmd()) {
                primary_type = secondary_type = htmCmdToRubyRequestType(pkt);
            } else if (pkt->req->isInstFetch()) {
                primary_type = secondary_type = RubyRequestType_IFETCH;
            } else {
                if (pkt->req->isReadModifyWrite()) {
                    primary_type = RubyRequestType_RMW_Read;
                    secondary_type = RubyRequestType_ST;
                } else {
                    primary_type = secondary_type = RubyRequestType_LD;
                }
            }
        } else if (pkt->isFlush()) {
          primary_type = secondary_type = RubyRequestType_FLUSH;
        } else if (pkt->cmd == MemCmd::MemSyncReq) {
            primary_type = secondary_type = RubyRequestType_REPLACEMENT;
            assert(!m_cache_inv_pkt);
            m_cache_inv_pkt = pkt;
            invL1();
        } else {
            panic("Unsupported ruby packet type\n");
        }
    }

    // Check if the line is blocked for a Locked_RMW
    if (!pkt->req->isMemMgmt() &&
        m_controller->isBlocked(makeLineAddress(pkt->getAddr())) &&
        (primary_type != RubyRequestType_Locked_RMW_Write)) {
        // Return that this request's cache line address aliases with
        // a prior request that locked the cache line. The request cannot
        // proceed until the cache line is unlocked by a Locked_RMW_Write
        return RequestStatus_Aliased;
    }

    RequestStatus status = insertRequest(pkt, primary_type, secondary_type);

    // It is OK to receive RequestStatus_Aliased, it can be considered Issued
    if (status != RequestStatus_Ready && status != RequestStatus_Aliased)
        return status;
    // non-aliased with any existing request in the request table, just issue
    // to the cache
    if (status != RequestStatus_Aliased)
        issueRequest(pkt, secondary_type);

    // TODO: issue hardware prefetches here
    return RequestStatus_Issued;
}

void
Sequencer::issueRequest(PacketPtr pkt, RubyRequestType secondary_type)
{
    assert(pkt != NULL);
    ContextID proc_id = pkt->req->hasContextId() ?
        pkt->req->contextId() : InvalidContextID;

    ContextID core_id = coreId();

    // If valid, copy the pc to the ruby request
    Addr pc = 0;
    if (pkt->req->hasPC()) {
        pc = pkt->req->getPC();
    }

    // check if the packet has data as for example prefetch and flush
    // requests do not
    std::shared_ptr<RubyRequest> msg;
    if (pkt->req->isMemMgmt()) {
        msg = std::make_shared<RubyRequest>(clockEdge(),
                                            pc, secondary_type,
                                            RubyAccessMode_Supervisor, pkt,
                                            proc_id, core_id);

        DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s\n",
                curTick(), m_version, "Seq", "Begin", "", "",
                RubyRequestType_to_string(secondary_type));

        if (pkt->req->isTlbiCmd()) {
            msg->m_isTlbi = true;
            switch (secondary_type) {
              case RubyRequestType_TLBI_EXT_SYNC_COMP:
                msg->m_tlbiTransactionUid = pkt->req->getExtraData();
                break;
              case RubyRequestType_TLBI:
              case RubyRequestType_TLBI_SYNC:
                msg->m_tlbiTransactionUid = \
                    getCurrentUnaddressedTransactionID();
                break;
              default:
                panic("Unexpected TLBI RubyRequestType");
            }
            DPRINTF(RubySequencer, "Issuing TLBI %016x\n",
                    msg->m_tlbiTransactionUid);
        }
    } else {
        msg = std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                                            pkt->getSize(), pc, secondary_type,
                                            RubyAccessMode_Supervisor, pkt,
                                            PrefetchBit_No, proc_id, core_id);

        if (pkt->isAtomicOp() &&
            ((secondary_type == RubyRequestType_ATOMIC_RETURN) ||
             (secondary_type == RubyRequestType_ATOMIC_NO_RETURN))){
            // Create the blocksize, access mask and atomicops
            uint32_t offset = getOffset(pkt->getAddr());
            std::vector<std::pair<int,AtomicOpFunctor*>> atomicOps;
            atomicOps.push_back(std::make_pair<int,AtomicOpFunctor*>
                                (offset, pkt->getAtomicOp()));

            msg->setWriteMask(offset, pkt->getSize(), atomicOps);
        }

        DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %s\n",
                curTick(), m_version, "Seq", "Begin", "", "",
                printAddress(msg->getPhysicalAddress()),
                RubyRequestType_to_string(secondary_type));
    }

    // hardware transactional memory
    // If the request originates in a transaction,
    // then mark the Ruby message as such.
    if (pkt->isHtmTransactional()) {
        msg->m_htmFromTransaction = true;
        msg->m_htmTransactionUid = pkt->getHtmTransactionUid();
    }

    Tick latency = cyclesToTicks(
                        m_controller->mandatoryQueueLatency(secondary_type));
    assert(latency > 0);

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
}

template <class KEY, class VALUE>
std::ostream &
operator<<(std::ostream &out, const std::unordered_map<KEY, VALUE> &map)
{
    for (const auto &table_entry : map) {
        out << "[ " << table_entry.first << " =";
        for (const auto &seq_req : table_entry.second) {
            out << " " << RubyRequestType_to_string(seq_req.m_second_type);
        }
    }
    out << " ]";

    return out;
}

void
Sequencer::print(std::ostream& out) const
{
    out << "[Sequencer: " << m_version
        << ", outstanding requests: " << m_outstanding_count
        << ", request table: " << m_RequestTable
        << "]";
}

void
Sequencer::recordRequestType(SequencerRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            SequencerRequestType_to_string(requestType));
}

void
Sequencer::evictionCallback(Addr address)
{
    llscClearMonitor(address);
    ruby_eviction_callback(address);
}

void
Sequencer::incrementUnaddressedTransactionCnt()
{
    m_unaddressedTransactionCnt++;
    // Limit m_unaddressedTransactionCnt to 32 bits,
    // top 32 bits should always be zeroed out
    uint64_t aligned_txid = \
        m_unaddressedTransactionCnt << RubySystem::getBlockSizeBits();

    if (aligned_txid > 0xFFFFFFFFull) {
        m_unaddressedTransactionCnt = 0;
    }
}

uint64_t
Sequencer::getCurrentUnaddressedTransactionID() const
{
    return (
        uint64_t(m_version & 0xFFFFFFFF) << 32) |
        (m_unaddressedTransactionCnt << RubySystem::getBlockSizeBits()
    );
}

} // namespace ruby
} // namespace gem5
