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

#include "base/misc.hh"
#include "base/str.hh"
#include "config/the_isa.hh"
#if THE_ISA == X86_ISA
#include "arch/x86/insts/microldstop.hh"
#endif // X86_ISA
#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/MemoryAccess.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubySequencer.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/system/CacheMemory.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/packet.hh"
#include "params/RubySequencer.hh"

using namespace std;

Sequencer *
RubySequencerParams::create()
{
    return new Sequencer(this);
}

Sequencer::Sequencer(const Params *p)
    : RubyPort(p), deadlockCheckEvent(this)
{
    m_store_waiting_on_load_cycles = 0;
    m_store_waiting_on_store_cycles = 0;
    m_load_waiting_on_store_cycles = 0;
    m_load_waiting_on_load_cycles = 0;

    m_outstanding_count = 0;

    m_instCache_ptr = p->icache;
    m_dataCache_ptr = p->dcache;
    m_max_outstanding_requests = p->max_outstanding_requests;
    m_deadlock_threshold = p->deadlock_threshold;

    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr != NULL);
    assert(m_dataCache_ptr != NULL);

    m_usingNetworkTester = p->using_network_tester;
}

Sequencer::~Sequencer()
{
}

void
Sequencer::wakeup()
{
    // Check for deadlock of any of the requests
    Time current_time = g_eventQueue_ptr->getTime();

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
             Address(request->pkt->getAddr()), m_readRequestTable.size(),
             current_time, request->issue_time,
             current_time - request->issue_time);
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
             Address(request->pkt->getAddr()), m_writeRequestTable.size(),
             current_time, request->issue_time,
             current_time - request->issue_time);
    }

    total_outstanding += m_writeRequestTable.size();
    total_outstanding += m_readRequestTable.size();

    assert(m_outstanding_count == total_outstanding);

    if (m_outstanding_count > 0) {
        // If there are still outstanding requests, keep checking
        schedule(deadlockCheckEvent,
                 m_deadlock_threshold * g_eventQueue_ptr->getClock() +
                 curTick());
    }
}

void
Sequencer::printStats(ostream & out) const
{
    out << "Sequencer: " << m_name << endl
        << "  store_waiting_on_load_cycles: "
        << m_store_waiting_on_load_cycles << endl
        << "  store_waiting_on_store_cycles: "
        << m_store_waiting_on_store_cycles << endl
        << "  load_waiting_on_load_cycles: "
        << m_load_waiting_on_load_cycles << endl
        << "  load_waiting_on_store_cycles: "
        << m_load_waiting_on_store_cycles << endl;
}

void
Sequencer::printProgress(ostream& out) const
{
#if 0
    int total_demand = 0;
    out << "Sequencer Stats Version " << m_version << endl;
    out << "Current time = " << g_eventQueue_ptr->getTime() << endl;
    out << "---------------" << endl;
    out << "outstanding requests" << endl;

    out << "proc " << m_Read
        << " version Requests = " << m_readRequestTable.size() << endl;

    // print the request table
    RequestTable::iterator read = m_readRequestTable.begin();
    RequestTable::iterator read_end = m_readRequestTable.end();
    for (; read != read_end; ++read) {
        SequencerRequest* request = read->second;
        out << "\tRequest[ " << i << " ] = " << request->type
            << " Address " << rkeys[i]
            << " Posted " << request->issue_time
            << " PF " << PrefetchBit_No << endl;
        total_demand++;
    }

    out << "proc " << m_version
        << " Write Requests = " << m_writeRequestTable.size << endl;

    // print the request table
    RequestTable::iterator write = m_writeRequestTable.begin();
    RequestTable::iterator write_end = m_writeRequestTable.end();
    for (; write != write_end; ++write) {
        SequencerRequest* request = write->second;
        out << "\tRequest[ " << i << " ] = " << request.getType()
            << " Address " << wkeys[i]
            << " Posted " << request.getTime()
            << " PF " << request.getPrefetch() << endl;
        if (request.getPrefetch() == PrefetchBit_No) {
            total_demand++;
        }
    }

    out << endl;

    out << "Total Number Outstanding: " << m_outstanding_count << endl
        << "Total Number Demand     : " << total_demand << endl
        << "Total Number Prefetches : " << m_outstanding_count - total_demand
        << endl << endl << endl;
#endif
}

void
Sequencer::printConfig(ostream& out) const
{
    out << "Seqeuncer config: " << m_name << endl
        << "  controller: " << m_controller->getName() << endl
        << "  version: " << m_version << endl
        << "  max_outstanding_requests: " << m_max_outstanding_requests << endl
        << "  deadlock_threshold: " << m_deadlock_threshold << endl;
}

// Insert the request on the correct request table.  Return true if
// the entry was already present.
RequestStatus
Sequencer::insertRequest(PacketPtr pkt, RubyRequestType request_type)
{
    assert(m_outstanding_count ==
        (m_writeRequestTable.size() + m_readRequestTable.size()));

    // See if we should schedule a deadlock check
    if (deadlockCheckEvent.scheduled() == false) {
        schedule(deadlockCheckEvent,
                 m_deadlock_threshold * g_eventQueue_ptr->getClock()
                 + curTick());
    }

    Address line_addr(pkt->getAddr());
    line_addr.makeLineAddress();
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
            m_store_waiting_on_load_cycles++;
            return RequestStatus_Aliased;
        }

        pair<RequestTable::iterator, bool> r =
            m_writeRequestTable.insert(RequestTable::value_type(line_addr, 0));
        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new SequencerRequest(pkt, request_type,
                                             g_eventQueue_ptr->getTime());
            m_outstanding_count++;
        } else {
          // There is an outstanding write request for the cache line
          m_store_waiting_on_store_cycles++;
          return RequestStatus_Aliased;
        }
    } else {
        // Check if there is any outstanding write request for the same
        // cache line.
        if (m_writeRequestTable.count(line_addr) > 0) {
            m_load_waiting_on_store_cycles++;
            return RequestStatus_Aliased;
        }

        pair<RequestTable::iterator, bool> r =
            m_readRequestTable.insert(RequestTable::value_type(line_addr, 0));

        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new SequencerRequest(pkt, request_type,
                                             g_eventQueue_ptr->getTime());
            m_outstanding_count++;
        } else {
            // There is an outstanding read request for the cache line
            m_load_waiting_on_load_cycles++;
            return RequestStatus_Aliased;
        }
    }

    g_system_ptr->getProfiler()->sequencerRequests(m_outstanding_count);
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
Sequencer::removeRequest(SequencerRequest* srequest)
{
    assert(m_outstanding_count ==
           m_writeRequestTable.size() + m_readRequestTable.size());

    Address line_addr(srequest->pkt->getAddr());
    line_addr.makeLineAddress();
    if ((srequest->m_type == RubyRequestType_ST) ||
        (srequest->m_type == RubyRequestType_RMW_Read) ||
        (srequest->m_type == RubyRequestType_RMW_Write) ||
        (srequest->m_type == RubyRequestType_Load_Linked) ||
        (srequest->m_type == RubyRequestType_Store_Conditional) ||
        (srequest->m_type == RubyRequestType_Locked_RMW_Read) ||
        (srequest->m_type == RubyRequestType_Locked_RMW_Write)) {
        m_writeRequestTable.erase(line_addr);
    } else {
        m_readRequestTable.erase(line_addr);
    }

    markRemoved();
}

bool
Sequencer::handleLlsc(const Address& address, SequencerRequest* request)
{
    //
    // The success flag indicates whether the LLSC operation was successful.
    // LL ops will always succeed, but SC may fail if the cache line is no
    // longer locked.
    //
    bool success = true;
    if (request->m_type == RubyRequestType_Store_Conditional) {
        if (!m_dataCache_ptr->isLocked(address, m_version)) {
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
        m_dataCache_ptr->clearLocked(address);
    } else if (request->m_type == RubyRequestType_Load_Linked) {
        //
        // Note: To fully follow Alpha LLSC semantics, should the LL clear any
        // previously locked cache lines?
        //
        m_dataCache_ptr->setLocked(address, m_version);
    } else if ((m_dataCache_ptr->isTagPresent(address)) &&
               (m_dataCache_ptr->isLocked(address, m_version))) {
        //
        // Normal writes should clear the locked address
        //
        m_dataCache_ptr->clearLocked(address);
    }
    return success;
}

void
Sequencer::writeCallback(const Address& address, DataBlock& data)
{
    writeCallback(address, GenericMachineType_NULL, data);
}

void
Sequencer::writeCallback(const Address& address,
                         GenericMachineType mach, 
                         DataBlock& data)
{
    writeCallback(address, mach, data, 0, 0, 0);
}

void
Sequencer::writeCallback(const Address& address,
                         GenericMachineType mach, 
                         DataBlock& data,
                         Time initialRequestTime,
                         Time forwardRequestTime,
                         Time firstResponseTime)
{
    assert(address == line_address(address));
    assert(m_writeRequestTable.count(line_address(address)));

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
    // Not valid for Network_test protocl
    //
    bool success = true;
    if(!m_usingNetworkTester)
        success = handleLlsc(address, request);

    if (request->m_type == RubyRequestType_Locked_RMW_Read) {
        m_controller->blockOnQueue(address, m_mandatory_q_ptr);
    } else if (request->m_type == RubyRequestType_Locked_RMW_Write) {
        m_controller->unblock(address);
    }

    hitCallback(request, mach, data, success, 
                initialRequestTime, forwardRequestTime, firstResponseTime);
}

void
Sequencer::readCallback(const Address& address, DataBlock& data)
{
    readCallback(address, GenericMachineType_NULL, data);
}

void
Sequencer::readCallback(const Address& address,
                        GenericMachineType mach,
                        DataBlock& data)
{
    readCallback(address, mach, data, 0, 0, 0);
}

void
Sequencer::readCallback(const Address& address,
                        GenericMachineType mach,
                        DataBlock& data,
                        Time initialRequestTime,
                        Time forwardRequestTime,
                        Time firstResponseTime)
{
    assert(address == line_address(address));
    assert(m_readRequestTable.count(line_address(address)));

    RequestTable::iterator i = m_readRequestTable.find(address);
    assert(i != m_readRequestTable.end());
    SequencerRequest* request = i->second;

    m_readRequestTable.erase(i);
    markRemoved();

    assert((request->m_type == RubyRequestType_LD) ||
           (request->m_type == RubyRequestType_IFETCH));

    hitCallback(request, mach, data, true, 
                initialRequestTime, forwardRequestTime, firstResponseTime);
}

void
Sequencer::hitCallback(SequencerRequest* srequest,
                       GenericMachineType mach,
                       DataBlock& data,
                       bool success,
                       Time initialRequestTime,
                       Time forwardRequestTime,
                       Time firstResponseTime)
{
    PacketPtr pkt = srequest->pkt;
    Address request_address(pkt->getAddr());
    Address request_line_address(pkt->getAddr());
    request_line_address.makeLineAddress();
    RubyRequestType type = srequest->m_type;
    Time issued_time = srequest->issue_time;

    // Set this cache entry to the most recently used
    if (type == RubyRequestType_IFETCH) {
        m_instCache_ptr->setMRU(request_line_address);
    } else {
        m_dataCache_ptr->setMRU(request_line_address);
    }

    assert(g_eventQueue_ptr->getTime() >= issued_time);
    Time miss_latency = g_eventQueue_ptr->getTime() - issued_time;

    // Profile the miss latency for all non-zero demand misses
    if (miss_latency != 0) {
        g_system_ptr->getProfiler()->missLatency(miss_latency, type, mach);

        if (mach == GenericMachineType_L1Cache_wCC) {
            g_system_ptr->getProfiler()->missLatencyWcc(issued_time,
                                                   initialRequestTime,
                                                   forwardRequestTime,
                                                   firstResponseTime,
                                                   g_eventQueue_ptr->getTime());
        }

        if (mach == GenericMachineType_Directory) {
            g_system_ptr->getProfiler()->missLatencyDir(issued_time,
                                                   initialRequestTime,
                                                   forwardRequestTime,
                                                   firstResponseTime,
                                                   g_eventQueue_ptr->getTime());
        }

        DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %d cycles\n",
                 curTick(), m_version, "Seq",
                 success ? "Done" : "SC_Failed", "", "",
                 request_address, miss_latency);
    }

    // update the data
    if (g_system_ptr->m_warmup_enabled) {
        assert(pkt->getPtr<uint8_t>(false) != NULL);
        data.setData(pkt->getPtr<uint8_t>(false),
                     request_address.getOffset(), pkt->getSize());
    } else if (pkt->getPtr<uint8_t>(true) != NULL) {
        if ((type == RubyRequestType_LD) ||
            (type == RubyRequestType_IFETCH) ||
            (type == RubyRequestType_RMW_Read) ||
            (type == RubyRequestType_Locked_RMW_Read) ||
            (type == RubyRequestType_Load_Linked)) {
            memcpy(pkt->getPtr<uint8_t>(true),
                   data.getData(request_address.getOffset(), pkt->getSize()),
                   pkt->getSize());
        } else {
            data.setData(pkt->getPtr<uint8_t>(true),
                         request_address.getOffset(), pkt->getSize());
        }
    } else {
        DPRINTF(MemoryAccess,
                "WARNING.  Data not transfered from Ruby to M5 for type %s\n",
                RubyRequestType_to_string(type));
    }

    // If using the RubyTester, update the RubyTester sender state's
    // subBlock with the recieved data.  The tester will later access
    // this state.
    // Note: RubyPort will access it's sender state before the
    // RubyTester.
    if (m_usingRubyTester) {
        RubyPort::SenderState *requestSenderState =
            safe_cast<RubyPort::SenderState*>(pkt->senderState);
        RubyTester::SenderState* testerSenderState =
            safe_cast<RubyTester::SenderState*>(requestSenderState->saved);
        testerSenderState->subBlock->mergeFrom(data);
    }

    delete srequest;

    if (g_system_ptr->m_warmup_enabled) {
        delete pkt;
        g_system_ptr->m_cache_recorder->enqueueNextFetchRequest();
    } else if (g_system_ptr->m_cooldown_enabled) {
        delete pkt;
        g_system_ptr->m_cache_recorder->enqueueNextFlushRequest();
    } else {
        ruby_hit_callback(pkt);
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
    } else if (pkt->req->isLocked()) {
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
        if (pkt->isRead()) {
            if (pkt->req->isInstFetch()) {
                primary_type = secondary_type = RubyRequestType_IFETCH;
            } else {
#if THE_ISA == X86_ISA
                uint32_t flags = pkt->req->getFlags();
                bool storeCheck = flags &
                        (TheISA::StoreCheck << TheISA::FlagShift);
#else
                bool storeCheck = false;
#endif // X86_ISA
                if (storeCheck) {
                    primary_type = RubyRequestType_RMW_Read;
                    secondary_type = RubyRequestType_ST;
                } else {
                    primary_type = secondary_type = RubyRequestType_LD;
                }
            }
        } else if (pkt->isWrite()) {
            //
            // Note: M5 packets do not differentiate ST from RMW_Write
            //
            primary_type = secondary_type = RubyRequestType_ST;
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
    int proc_id = -1;
    if (pkt != NULL && pkt->req->hasContextId()) {
        proc_id = pkt->req->contextId();
    }

    // If valid, copy the pc to the ruby request
    Addr pc = 0;
    if (pkt->req->hasPC()) {
        pc = pkt->req->getPC();
    }

    RubyRequest *msg = new RubyRequest(pkt->getAddr(),
                                       pkt->getPtr<uint8_t>(true),
                                       pkt->getSize(), pc, secondary_type,
                                       RubyAccessMode_Supervisor, pkt,
                                       PrefetchBit_No, proc_id);

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %s\n",
            curTick(), m_version, "Seq", "Begin", "", "",
            msg->getPhysicalAddress(),
            RubyRequestType_to_string(secondary_type));

    Time latency = 0;  // initialzed to an null value

    if (secondary_type == RubyRequestType_IFETCH)
        latency = m_instCache_ptr->getLatency();
    else
        latency = m_dataCache_ptr->getLatency();

    // Send the message to the cache controller
    assert(latency > 0);

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, latency);
}

template <class KEY, class VALUE>
std::ostream &
operator<<(ostream &out, const m5::hash_map<KEY, VALUE> &map)
{
    typename m5::hash_map<KEY, VALUE>::const_iterator i = map.begin();
    typename m5::hash_map<KEY, VALUE>::const_iterator end = map.end();

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
Sequencer::checkCoherence(const Address& addr)
{
#ifdef CHECK_COHERENCE
    g_system_ptr->checkGlobalCoherenceInvariant(addr);
#endif
}

void
Sequencer::evictionCallback(const Address& address)
{
    ruby_eviction_callback(address);
}
