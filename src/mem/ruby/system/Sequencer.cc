/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#include "arch/x86/ldstflags.hh"
#include "base/misc.hh"
#include "base/str.hh"
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/MemoryAccess.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubySequencer.hh"
#include "debug/RubyStats.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/packet.hh"
#include "sim/system.hh"

using namespace std;

Sequencer *
RubySequencerParams::create()
{
    return new Sequencer(this);
}

Sequencer::Sequencer(const Params *p)
    : RubyPort(p), m_IncompleteTimes(MachineType_NUM), deadlockCheckEvent(this)
{
    m_outstanding_count = 0;

    m_instCache_ptr = p->icache;
    m_dataCache_ptr = p->dcache;
    m_data_cache_hit_latency = p->dcache_hit_latency;
    m_inst_cache_hit_latency = p->icache_hit_latency;
    m_max_outstanding_requests = p->max_outstanding_requests;
    m_deadlock_threshold = p->deadlock_threshold;

    m_coreId = p->coreid; // for tracking the two CorePair sequencers
    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr != NULL);
    assert(m_dataCache_ptr != NULL);
    assert(m_data_cache_hit_latency > 0);
    assert(m_inst_cache_hit_latency > 0);

    m_runningGarnetStandalone = p->garnet_standalone;
}

Sequencer::~Sequencer()
{
}

void
Sequencer::wakeup()
{
    assert(drainState() != DrainState::Draining);

    // Check for deadlock of any of the requests
    Cycles current_time = curCycle();

    // Check across all outstanding requests
    int total_outstanding = 0;

    RequestTable::iterator read = m_readRequestTable.begin();
    RequestTable::iterator read_end = m_readRequestTable.end();
    for (; read != read_end; ++read) {
        SequencerRequest* request = read->second;
        if (current_time - request->issue_time < m_deadlock_threshold)
            continue;

        panic("Possible Deadlock detected. Aborting!\n"
              "version: %d request.paddr: 0x%x m_readRequestTable: %d "
              "current time: %u issue_time: %d difference: %d\n", m_version,
              request->pkt->getAddr(), m_readRequestTable.size(),
              current_time * clockPeriod(), request->issue_time * clockPeriod(),
              (current_time * clockPeriod()) - (request->issue_time * clockPeriod()));
    }

    RequestTable::iterator write = m_writeRequestTable.begin();
    RequestTable::iterator write_end = m_writeRequestTable.end();
    for (; write != write_end; ++write) {
        SequencerRequest* request = write->second;
        if (current_time - request->issue_time < m_deadlock_threshold)
            continue;

        panic("Possible Deadlock detected. Aborting!\n"
              "version: %d request.paddr: 0x%x m_writeRequestTable: %d "
              "current time: %u issue_time: %d difference: %d\n", m_version,
              request->pkt->getAddr(), m_writeRequestTable.size(),
              current_time * clockPeriod(), request->issue_time * clockPeriod(),
              (current_time * clockPeriod()) - (request->issue_time * clockPeriod()));
    }

    total_outstanding += m_writeRequestTable.size();
    total_outstanding += m_readRequestTable.size();

    assert(m_outstanding_count == total_outstanding);

    if (m_outstanding_count > 0) {
        // If there are still outstanding requests, keep checking
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }
}

void Sequencer::resetStats()
{
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

// Insert the request on the correct request table.  Return true if
// the entry was already present.
RequestStatus
Sequencer::insertRequest(PacketPtr pkt, RubyRequestType request_type)
{
    assert(m_outstanding_count ==
        (m_writeRequestTable.size() + m_readRequestTable.size()));

    // See if we should schedule a deadlock check
    if (!deadlockCheckEvent.scheduled() &&
        drainState() != DrainState::Draining) {
        schedule(deadlockCheckEvent, clockEdge(m_deadlock_threshold));
    }

    Addr line_addr = makeLineAddress(pkt->getAddr());

    // Check if the line is blocked for a Locked_RMW
    if (m_controller->isBlocked(line_addr) &&
        (request_type != RubyRequestType_Locked_RMW_Write)) {
        // Return that this request's cache line address aliases with
        // a prior request that locked the cache line. The request cannot
        // proceed until the cache line is unlocked by a Locked_RMW_Write
        return RequestStatus_Aliased;
    }

    // Create a default entry, mapping the address to NULL, the cast is
    // there to make gcc 4.4 happy
    RequestTable::value_type default_entry(line_addr,
                                           (SequencerRequest*) NULL);

    if ((request_type == RubyRequestType_ST) ||
        (request_type == RubyRequestType_RMW_Read) ||
        (request_type == RubyRequestType_RMW_Write) ||
        (request_type == RubyRequestType_Load_Linked) ||
        (request_type == RubyRequestType_Store_Conditional) ||
        (request_type == RubyRequestType_Locked_RMW_Read) ||
        (request_type == RubyRequestType_Locked_RMW_Write) ||
        (request_type == RubyRequestType_FLUSH)) {

        // Check if there is any outstanding read request for the same
        // cache line.
        if (m_readRequestTable.count(line_addr) > 0) {
            m_store_waiting_on_load++;
            return RequestStatus_Aliased;
        }

        pair<RequestTable::iterator, bool> r =
            m_writeRequestTable.insert(default_entry);
        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new SequencerRequest(pkt, request_type, curCycle());
            m_outstanding_count++;
        } else {
          // There is an outstanding write request for the cache line
          m_store_waiting_on_store++;
          return RequestStatus_Aliased;
        }
    } else {
        // Check if there is any outstanding write request for the same
        // cache line.
        if (m_writeRequestTable.count(line_addr) > 0) {
            m_load_waiting_on_store++;
            return RequestStatus_Aliased;
        }

        pair<RequestTable::iterator, bool> r =
            m_readRequestTable.insert(default_entry);

        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new SequencerRequest(pkt, request_type, curCycle());
            m_outstanding_count++;
        } else {
            // There is an outstanding read request for the cache line
            m_load_waiting_on_load++;
            return RequestStatus_Aliased;
        }
    }

    m_outstandReqHist.sample(m_outstanding_count);
    assert(m_outstanding_count ==
        (m_writeRequestTable.size() + m_readRequestTable.size()));

    return RequestStatus_Ready;
}

void
Sequencer::markRemoved()
{
    m_outstanding_count--;
    assert(m_outstanding_count ==
           m_writeRequestTable.size() + m_readRequestTable.size());
}

void
Sequencer::invalidateSC(Addr address)
{
    AbstractCacheEntry *e = m_dataCache_ptr->lookup(address);
    // The controller has lost the coherence permissions, hence the lock
    // on the cache line maintained by the cache should be cleared.
    if (e && e->isLocked(m_version)) {
        e->clearLocked();
    }
}

bool
Sequencer::handleLlsc(Addr address, SequencerRequest* request)
{
    AbstractCacheEntry *e = m_dataCache_ptr->lookup(address);
    if (!e)
        return true;

    // The success flag indicates whether the LLSC operation was successful.
    // LL ops will always succeed, but SC may fail if the cache line is no
    // longer locked.
    bool success = true;
    if (request->m_type == RubyRequestType_Store_Conditional) {
        if (!e->isLocked(m_version)) {
            //
            // For failed SC requests, indicate the failure to the cpu by
            // setting the extra data to zero.
            //
            request->pkt->req->setExtraData(0);
            success = false;
        } else {
            //
            // For successful SC requests, indicate the success to the cpu by
            // setting the extra data to one.
            //
            request->pkt->req->setExtraData(1);
        }
        //
        // Independent of success, all SC operations must clear the lock
        //
        e->clearLocked();
    } else if (request->m_type == RubyRequestType_Load_Linked) {
        //
        // Note: To fully follow Alpha LLSC semantics, should the LL clear any
        // previously locked cache lines?
        //
        e->setLocked(m_version);
    } else if (e->isLocked(m_version)) {
        //
        // Normal writes should clear the locked address
        //
        e->clearLocked();
    }
    return success;
}

void
Sequencer::recordMissLatency(const Cycles cycles, const RubyRequestType type,
                             const MachineType respondingMach,
                             bool isExternalHit, Cycles issuedTime,
                             Cycles initialRequestTime,
                             Cycles forwardRequestTime,
                             Cycles firstResponseTime, Cycles completionTime)
{
    m_latencyHist.sample(cycles);
    m_typeLatencyHist[type]->sample(cycles);

    if (isExternalHit) {
        m_missLatencyHist.sample(cycles);
        m_missTypeLatencyHist[type]->sample(cycles);

        if (respondingMach != MachineType_NUM) {
            m_missMachLatencyHist[respondingMach]->sample(cycles);
            m_missTypeMachLatencyHist[type][respondingMach]->sample(cycles);

            if ((issuedTime <= initialRequestTime) &&
                (initialRequestTime <= forwardRequestTime) &&
                (forwardRequestTime <= firstResponseTime) &&
                (firstResponseTime <= completionTime)) {

                m_IssueToInitialDelayHist[respondingMach]->sample(
                    initialRequestTime - issuedTime);
                m_InitialToForwardDelayHist[respondingMach]->sample(
                    forwardRequestTime - initialRequestTime);
                m_ForwardToFirstResponseDelayHist[respondingMach]->sample(
                    firstResponseTime - forwardRequestTime);
                m_FirstResponseToCompletionDelayHist[respondingMach]->sample(
                    completionTime - firstResponseTime);
            } else {
                m_IncompleteTimes[respondingMach]++;
            }
        }
    } else {
        m_hitLatencyHist.sample(cycles);
        m_hitTypeLatencyHist[type]->sample(cycles);

        if (respondingMach != MachineType_NUM) {
            m_hitMachLatencyHist[respondingMach]->sample(cycles);
            m_hitTypeMachLatencyHist[type][respondingMach]->sample(cycles);
        }
    }
}

void
Sequencer::writeCallback(Addr address, DataBlock& data,
                         const bool externalHit, const MachineType mach,
                         const Cycles initialRequestTime,
                         const Cycles forwardRequestTime,
                         const Cycles firstResponseTime)
{
    assert(address == makeLineAddress(address));
    assert(m_writeRequestTable.count(makeLineAddress(address)));

    RequestTable::iterator i = m_writeRequestTable.find(address);
    assert(i != m_writeRequestTable.end());
    SequencerRequest* request = i->second;

    m_writeRequestTable.erase(i);
    markRemoved();

    assert((request->m_type == RubyRequestType_ST) ||
           (request->m_type == RubyRequestType_ATOMIC) ||
           (request->m_type == RubyRequestType_RMW_Read) ||
           (request->m_type == RubyRequestType_RMW_Write) ||
           (request->m_type == RubyRequestType_Load_Linked) ||
           (request->m_type == RubyRequestType_Store_Conditional) ||
           (request->m_type == RubyRequestType_Locked_RMW_Read) ||
           (request->m_type == RubyRequestType_Locked_RMW_Write) ||
           (request->m_type == RubyRequestType_FLUSH));

    //
    // For Alpha, properly handle LL, SC, and write requests with respect to
    // locked cache blocks.
    //
    // Not valid for Garnet_standalone protocl
    //
    bool success = true;
    if (!m_runningGarnetStandalone)
        success = handleLlsc(address, request);

    // Handle SLICC block_on behavior for Locked_RMW accesses. NOTE: the
    // address variable here is assumed to be a line address, so when
    // blocking buffers, must check line addresses.
    if (request->m_type == RubyRequestType_Locked_RMW_Read) {
        // blockOnQueue blocks all first-level cache controller queues
        // waiting on memory accesses for the specified address that go to
        // the specified queue. In this case, a Locked_RMW_Write must go to
        // the mandatory_q before unblocking the first-level controller.
        // This will block standard loads, stores, ifetches, etc.
        m_controller->blockOnQueue(address, m_mandatory_q_ptr);
    } else if (request->m_type == RubyRequestType_Locked_RMW_Write) {
        m_controller->unblock(address);
    }

    hitCallback(request, data, success, mach, externalHit,
                initialRequestTime, forwardRequestTime, firstResponseTime);
}

void
Sequencer::readCallback(Addr address, DataBlock& data,
                        bool externalHit, const MachineType mach,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{
    assert(address == makeLineAddress(address));
    assert(m_readRequestTable.count(makeLineAddress(address)));

    RequestTable::iterator i = m_readRequestTable.find(address);
    assert(i != m_readRequestTable.end());
    SequencerRequest* request = i->second;

    m_readRequestTable.erase(i);
    markRemoved();

    assert((request->m_type == RubyRequestType_LD) ||
           (request->m_type == RubyRequestType_IFETCH));

    hitCallback(request, data, true, mach, externalHit,
                initialRequestTime, forwardRequestTime, firstResponseTime);
}

void
Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data,
                       bool llscSuccess,
                       const MachineType mach, const bool externalHit,
                       const Cycles initialRequestTime,
                       const Cycles forwardRequestTime,
                       const Cycles firstResponseTime)
{
    warn_once("Replacement policy updates recently became the responsibility "
              "of SLICC state machines. Make sure to setMRU() near callbacks "
              "in .sm files!");

    PacketPtr pkt = srequest->pkt;
    Addr request_address(pkt->getAddr());
    RubyRequestType type = srequest->m_type;
    Cycles issued_time = srequest->issue_time;

    assert(curCycle() >= issued_time);
    Cycles total_latency = curCycle() - issued_time;

    // Profile the latency for all demand accesses.
    recordMissLatency(total_latency, type, mach, externalHit, issued_time,
                      initialRequestTime, forwardRequestTime,
                      firstResponseTime, curCycle());

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %d cycles\n",
             curTick(), m_version, "Seq",
             llscSuccess ? "Done" : "SC_Failed", "", "",
             printAddress(request_address), total_latency);

    // update the data unless it is a non-data-carrying flush
    if (RubySystem::getWarmupEnabled()) {
        data.setData(pkt->getConstPtr<uint8_t>(),
                     getOffset(request_address), pkt->getSize());
    } else if (!pkt->isFlush()) {
        if ((type == RubyRequestType_LD) ||
            (type == RubyRequestType_IFETCH) ||
            (type == RubyRequestType_RMW_Read) ||
            (type == RubyRequestType_Locked_RMW_Read) ||
            (type == RubyRequestType_Load_Linked)) {
            memcpy(pkt->getPtr<uint8_t>(),
                   data.getData(getOffset(request_address), pkt->getSize()),
                   pkt->getSize());
            DPRINTF(RubySequencer, "read data %s\n", data);
        } else if (pkt->req->isSwap()) {
            std::vector<uint8_t> overwrite_val(pkt->getSize());
            memcpy(&overwrite_val[0], pkt->getConstPtr<uint8_t>(),
                   pkt->getSize());
            memcpy(pkt->getPtr<uint8_t>(),
                   data.getData(getOffset(request_address), pkt->getSize()),
                   pkt->getSize());
            data.setData(&overwrite_val[0],
                         getOffset(request_address), pkt->getSize());
            DPRINTF(RubySequencer, "swap data %s\n", data);
        } else {
            data.setData(pkt->getConstPtr<uint8_t>(),
                         getOffset(request_address), pkt->getSize());
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

    delete srequest;

    RubySystem *rs = m_ruby_system;
    if (RubySystem::getWarmupEnabled()) {
        assert(pkt->req);
        delete pkt->req;
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

bool
Sequencer::empty() const
{
    return m_writeRequestTable.empty() && m_readRequestTable.empty();
}

RequestStatus
Sequencer::makeRequest(PacketPtr pkt)
{
    if (m_outstanding_count >= m_max_outstanding_requests) {
        return RequestStatus_BufferFull;
    }

    RubyRequestType primary_type = RubyRequestType_NULL;
    RubyRequestType secondary_type = RubyRequestType_NULL;

    if (pkt->isLLSC()) {
        //
        // Alpha LL/SC instructions need to be handled carefully by the cache
        // coherence protocol to ensure they follow the proper semantics. In
        // particular, by identifying the operations as atomic, the protocol
        // should understand that migratory sharing optimizations should not
        // be performed (i.e. a load between the LL and SC should not steal
        // away exclusive permission).
        //
        if (pkt->isWrite()) {
            DPRINTF(RubySequencer, "Issuing SC\n");
            primary_type = RubyRequestType_Store_Conditional;
        } else {
            DPRINTF(RubySequencer, "Issuing LL\n");
            assert(pkt->isRead());
            primary_type = RubyRequestType_Load_Linked;
        }
        secondary_type = RubyRequestType_ATOMIC;
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
            if (pkt->req->isInstFetch()) {
                primary_type = secondary_type = RubyRequestType_IFETCH;
            } else {
                bool storeCheck = false;
                // only X86 need the store check
                if (system->getArch() == Arch::X86ISA) {
                    uint32_t flags = pkt->req->getFlags();
                    storeCheck = flags &
                        (X86ISA::StoreCheck << X86ISA::FlagShift);
                }
                if (storeCheck) {
                    primary_type = RubyRequestType_RMW_Read;
                    secondary_type = RubyRequestType_ST;
                } else {
                    primary_type = secondary_type = RubyRequestType_LD;
                }
            }
        } else if (pkt->isFlush()) {
          primary_type = secondary_type = RubyRequestType_FLUSH;
        } else {
            panic("Unsupported ruby packet type\n");
        }
    }

    RequestStatus status = insertRequest(pkt, primary_type);
    if (status != RequestStatus_Ready)
        return status;

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
    std::shared_ptr<RubyRequest> msg =
        std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                                      pkt->isFlush() ?
                                      nullptr : pkt->getPtr<uint8_t>(),
                                      pkt->getSize(), pc, secondary_type,
                                      RubyAccessMode_Supervisor, pkt,
                                      PrefetchBit_No, proc_id, core_id);

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %s\n",
            curTick(), m_version, "Seq", "Begin", "", "",
            printAddress(msg->getPhysicalAddress()),
            RubyRequestType_to_string(secondary_type));

    // The Sequencer currently assesses instruction and data cache hit latency
    // for the top-level caches at the beginning of a memory access.
    // TODO: Eventually, this latency should be moved to represent the actual
    // cache access latency portion of the memory access. This will require
    // changing cache controller protocol files to assess the latency on the
    // access response path.
    Cycles latency(0);  // Initialize to zero to catch misconfigured latency
    if (secondary_type == RubyRequestType_IFETCH)
        latency = m_inst_cache_hit_latency;
    else
        latency = m_data_cache_hit_latency;

    // Send the message to the cache controller
    assert(latency > 0);

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(latency));
}

template <class KEY, class VALUE>
std::ostream &
operator<<(ostream &out, const std::unordered_map<KEY, VALUE> &map)
{
    auto i = map.begin();
    auto end = map.end();

    out << "[";
    for (; i != end; ++i)
        out << " " << i->first << "=" << i->second;
    out << " ]";

    return out;
}

void
Sequencer::print(ostream& out) const
{
    out << "[Sequencer: " << m_version
        << ", outstanding requests: " << m_outstanding_count
        << ", read request table: " << m_readRequestTable
        << ", write request table: " << m_writeRequestTable
        << "]";
}

// this can be called from setState whenever coherence permissions are
// upgraded when invoked, coherence violations will be checked for the
// given block
void
Sequencer::checkCoherence(Addr addr)
{
#ifdef CHECK_COHERENCE
    m_ruby_system->checkGlobalCoherenceInvariant(addr);
#endif
}

void
Sequencer::recordRequestType(SequencerRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            SequencerRequestType_to_string(requestType));
}


void
Sequencer::evictionCallback(Addr address)
{
    ruby_eviction_callback(address);
}

void
Sequencer::regStats()
{
    RubyPort::regStats();

    m_store_waiting_on_load
        .name(name() + ".store_waiting_on_load")
        .desc("Number of times a store aliased with a pending load")
        .flags(Stats::nozero);
    m_store_waiting_on_store
        .name(name() + ".store_waiting_on_store")
        .desc("Number of times a store aliased with a pending store")
        .flags(Stats::nozero);
    m_load_waiting_on_load
        .name(name() + ".load_waiting_on_load")
        .desc("Number of times a load aliased with a pending load")
        .flags(Stats::nozero);
    m_load_waiting_on_store
        .name(name() + ".load_waiting_on_store")
        .desc("Number of times a load aliased with a pending store")
        .flags(Stats::nozero);

    // These statistical variables are not for display.
    // The profiler will collate these across different
    // sequencers and display those collated statistics.
    m_outstandReqHist.init(10);
    m_latencyHist.init(10);
    m_hitLatencyHist.init(10);
    m_missLatencyHist.init(10);

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist.push_back(new Stats::Histogram());
        m_typeLatencyHist[i]->init(10);

        m_hitTypeLatencyHist.push_back(new Stats::Histogram());
        m_hitTypeLatencyHist[i]->init(10);

        m_missTypeLatencyHist.push_back(new Stats::Histogram());
        m_missTypeLatencyHist[i]->init(10);
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_hitMachLatencyHist.push_back(new Stats::Histogram());
        m_hitMachLatencyHist[i]->init(10);

        m_missMachLatencyHist.push_back(new Stats::Histogram());
        m_missMachLatencyHist[i]->init(10);

        m_IssueToInitialDelayHist.push_back(new Stats::Histogram());
        m_IssueToInitialDelayHist[i]->init(10);

        m_InitialToForwardDelayHist.push_back(new Stats::Histogram());
        m_InitialToForwardDelayHist[i]->init(10);

        m_ForwardToFirstResponseDelayHist.push_back(new Stats::Histogram());
        m_ForwardToFirstResponseDelayHist[i]->init(10);

        m_FirstResponseToCompletionDelayHist.push_back(new Stats::Histogram());
        m_FirstResponseToCompletionDelayHist[i]->init(10);
    }

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_hitTypeMachLatencyHist.push_back(std::vector<Stats::Histogram *>());
        m_missTypeMachLatencyHist.push_back(std::vector<Stats::Histogram *>());

        for (int j = 0; j < MachineType_NUM; j++) {
            m_hitTypeMachLatencyHist[i].push_back(new Stats::Histogram());
            m_hitTypeMachLatencyHist[i][j]->init(10);

            m_missTypeMachLatencyHist[i].push_back(new Stats::Histogram());
            m_missTypeMachLatencyHist[i][j]->init(10);
        }
    }
}
