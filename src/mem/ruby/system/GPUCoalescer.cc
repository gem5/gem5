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
 *
 * Authors: Sooraj Puthoor
 */

#include "base/logging.hh"
#include "base/str.hh"
#include "config/the_isa.hh"

#if THE_ISA == X86_ISA
#include "arch/x86/insts/microldstop.hh"

#endif // X86_ISA
#include "mem/ruby/system/GPUCoalescer.hh"

#include "cpu/testers/rubytest/RubyTester.hh"
#include "debug/GPUCoalescer.hh"
#include "debug/MemoryAccess.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubyPort.hh"
#include "debug/RubyStats.hh"
#include "gpu-compute/shader.hh"
#include "mem/packet.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "mem/ruby/structures/CacheMemory.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/RubyGPUCoalescer.hh"

using namespace std;

GPUCoalescer *
RubyGPUCoalescerParams::create()
{
    return new GPUCoalescer(this);
}

HSAScope
reqScopeToHSAScope(const RequestPtr &req)
{
    HSAScope accessScope = HSAScope_UNSPECIFIED;
    if (req->isScoped()) {
        if (req->isWavefrontScope()) {
            accessScope = HSAScope_WAVEFRONT;
        } else if (req->isWorkgroupScope()) {
            accessScope = HSAScope_WORKGROUP;
        } else if (req->isDeviceScope()) {
            accessScope = HSAScope_DEVICE;
        } else if (req->isSystemScope()) {
            accessScope = HSAScope_SYSTEM;
        } else {
            fatal("Bad scope type");
        }
    }
    return accessScope;
}

HSASegment
reqSegmentToHSASegment(const RequestPtr &req)
{
    HSASegment accessSegment = HSASegment_GLOBAL;

    if (req->isGlobalSegment()) {
        accessSegment = HSASegment_GLOBAL;
    } else if (req->isGroupSegment()) {
        accessSegment = HSASegment_GROUP;
    } else if (req->isPrivateSegment()) {
        accessSegment = HSASegment_PRIVATE;
    } else if (req->isKernargSegment()) {
        accessSegment = HSASegment_KERNARG;
    } else if (req->isReadonlySegment()) {
        accessSegment = HSASegment_READONLY;
    } else if (req->isSpillSegment()) {
        accessSegment = HSASegment_SPILL;
    } else if (req->isArgSegment()) {
        accessSegment = HSASegment_ARG;
    } else {
        fatal("Bad segment type");
    }

    return accessSegment;
}

GPUCoalescer::GPUCoalescer(const Params *p)
    : RubyPort(p),
      issueEvent([this]{ completeIssue(); }, "Issue coalesced request",
                 false, Event::Progress_Event_Pri),
      deadlockCheckEvent([this]{ wakeup(); }, "GPUCoalescer deadlock check")
{
    m_store_waiting_on_load_cycles = 0;
    m_store_waiting_on_store_cycles = 0;
    m_load_waiting_on_store_cycles = 0;
    m_load_waiting_on_load_cycles = 0;

    m_outstanding_count = 0;

    m_max_outstanding_requests = 0;
    m_deadlock_threshold = 0;
    m_instCache_ptr = nullptr;
    m_dataCache_ptr = nullptr;

    m_instCache_ptr = p->icache;
    m_dataCache_ptr = p->dcache;
    m_max_outstanding_requests = p->max_outstanding_requests;
    m_deadlock_threshold = p->deadlock_threshold;

    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr);
    assert(m_dataCache_ptr);

    m_data_cache_hit_latency = p->dcache_hit_latency;

    m_runningGarnetStandalone = p->garnet_standalone;
    assumingRfOCoherence = p->assume_rfo;
}

GPUCoalescer::~GPUCoalescer()
{
}

void
GPUCoalescer::wakeup()
{
    // Check for deadlock of any of the requests
    Cycles current_time = curCycle();

    // Check across all outstanding requests
    int total_outstanding = 0;

    RequestTable::iterator read = m_readRequestTable.begin();
    RequestTable::iterator read_end = m_readRequestTable.end();
    for (; read != read_end; ++read) {
        GPUCoalescerRequest* request = read->second;
        if (current_time - request->issue_time < m_deadlock_threshold)
            continue;

        panic("Possible Deadlock detected. Aborting!\n"
             "version: %d request.paddr: 0x%x m_readRequestTable: %d "
             "current time: %u issue_time: %d difference: %d\n", m_version,
              request->pkt->getAddr(), m_readRequestTable.size(),
              current_time * clockPeriod(), request->issue_time * clockPeriod(),
              (current_time - request->issue_time)*clockPeriod());
    }

    RequestTable::iterator write = m_writeRequestTable.begin();
    RequestTable::iterator write_end = m_writeRequestTable.end();
    for (; write != write_end; ++write) {
        GPUCoalescerRequest* request = write->second;
        if (current_time - request->issue_time < m_deadlock_threshold)
            continue;

        panic("Possible Deadlock detected. Aborting!\n"
             "version: %d request.paddr: 0x%x m_writeRequestTable: %d "
             "current time: %u issue_time: %d difference: %d\n", m_version,
              request->pkt->getAddr(), m_writeRequestTable.size(),
              current_time * clockPeriod(), request->issue_time * clockPeriod(),
              (current_time - request->issue_time) * clockPeriod());
    }

    total_outstanding += m_writeRequestTable.size();
    total_outstanding += m_readRequestTable.size();

    assert(m_outstanding_count == total_outstanding);

    if (m_outstanding_count > 0) {
        // If there are still outstanding requests, keep checking
        schedule(deadlockCheckEvent,
                 m_deadlock_threshold * clockPeriod() +
                 curTick());
    }
}

void
GPUCoalescer::resetStats()
{
    m_latencyHist.reset();
    m_missLatencyHist.reset();
    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist[i]->reset();
        m_missTypeLatencyHist[i]->reset();
        for (int j = 0; j < MachineType_NUM; j++) {
            m_missTypeMachLatencyHist[i][j]->reset();
        }
    }

    for (int i = 0; i < MachineType_NUM; i++) {
        m_missMachLatencyHist[i]->reset();

        m_IssueToInitialDelayHist[i]->reset();
        m_InitialToForwardDelayHist[i]->reset();
        m_ForwardToFirstResponseDelayHist[i]->reset();
        m_FirstResponseToCompletionDelayHist[i]->reset();
    }
}

void
GPUCoalescer::printProgress(ostream& out) const
{
}

RequestStatus
GPUCoalescer::getRequestStatus(PacketPtr pkt, RubyRequestType request_type)
{
    Addr line_addr = makeLineAddress(pkt->getAddr());

    if (!m_mandatory_q_ptr->areNSlotsAvailable(1, clockEdge())) {
        return RequestStatus_BufferFull;
    }

    if (m_controller->isBlocked(line_addr) &&
       request_type != RubyRequestType_Locked_RMW_Write) {
        return RequestStatus_Aliased;
    }

    if ((request_type == RubyRequestType_ST) ||
        (request_type == RubyRequestType_ATOMIC) ||
        (request_type == RubyRequestType_ATOMIC_RETURN) ||
        (request_type == RubyRequestType_ATOMIC_NO_RETURN) ||
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

        if (m_writeRequestTable.count(line_addr) > 0) {
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

        if (m_readRequestTable.count(line_addr) > 0) {
            // There is an outstanding read request for the cache line
            m_load_waiting_on_load_cycles++;
            return RequestStatus_Aliased;
        }
    }

    return RequestStatus_Ready;

}



// sets the kernelEndList
void
GPUCoalescer::insertKernel(int wavefront_id, PacketPtr pkt)
{
    // Don't know if this will happen or is possible
    // but I just want to be careful and not have it become
    // simulator hang in the future
    DPRINTF(GPUCoalescer, "inserting wf: %d to kernelEndlist\n", wavefront_id);
    assert(kernelEndList.count(wavefront_id) == 0);

    kernelEndList[wavefront_id] = pkt;
    DPRINTF(GPUCoalescer, "kernelEndList->size() = %d\n",
            kernelEndList.size());
}


// Insert the request on the correct request table.  Return true if
// the entry was already present.
bool
GPUCoalescer::insertRequest(PacketPtr pkt, RubyRequestType request_type)
{
    assert(getRequestStatus(pkt, request_type) == RequestStatus_Ready ||
           pkt->req->isLockedRMW() ||
           !m_mandatory_q_ptr->areNSlotsAvailable(1, clockEdge()));

    int total_outstanding M5_VAR_USED =
        m_writeRequestTable.size() + m_readRequestTable.size();

    assert(m_outstanding_count == total_outstanding);

    // See if we should schedule a deadlock check
    if (!deadlockCheckEvent.scheduled()) {
        schedule(deadlockCheckEvent, m_deadlock_threshold + curTick());
    }

    Addr line_addr = makeLineAddress(pkt->getAddr());
    if ((request_type == RubyRequestType_ST) ||
        (request_type == RubyRequestType_ATOMIC) ||
        (request_type == RubyRequestType_ATOMIC_RETURN) ||
        (request_type == RubyRequestType_ATOMIC_NO_RETURN) ||
        (request_type == RubyRequestType_RMW_Read) ||
        (request_type == RubyRequestType_RMW_Write) ||
        (request_type == RubyRequestType_Load_Linked) ||
        (request_type == RubyRequestType_Store_Conditional) ||
        (request_type == RubyRequestType_Locked_RMW_Read) ||
        (request_type == RubyRequestType_Locked_RMW_Write) ||
        (request_type == RubyRequestType_FLUSH)) {

        pair<RequestTable::iterator, bool> r =
          m_writeRequestTable.insert(RequestTable::value_type(line_addr,
                                       (GPUCoalescerRequest*) NULL));
        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new GPUCoalescerRequest(pkt, request_type,
                                                curCycle());
            DPRINTF(GPUCoalescer,
                    "Inserting write request for paddr %#x for type %d\n",
                    pkt->req->getPaddr(), i->second->m_type);
            m_outstanding_count++;
        } else {
            return true;
        }
    } else {
        pair<RequestTable::iterator, bool> r =
            m_readRequestTable.insert(RequestTable::value_type(line_addr,
                                        (GPUCoalescerRequest*) NULL));

        if (r.second) {
            RequestTable::iterator i = r.first;
            i->second = new GPUCoalescerRequest(pkt, request_type,
                                             curCycle());
            DPRINTF(GPUCoalescer,
                    "Inserting read request for paddr %#x for type %d\n",
                    pkt->req->getPaddr(), i->second->m_type);
            m_outstanding_count++;
        } else {
            return true;
        }
    }

    m_outstandReqHist.sample(m_outstanding_count);

    total_outstanding = m_writeRequestTable.size() + m_readRequestTable.size();
    assert(m_outstanding_count == total_outstanding);

    return false;
}

void
GPUCoalescer::markRemoved()
{
    m_outstanding_count--;
    assert(m_outstanding_count ==
           m_writeRequestTable.size() + m_readRequestTable.size());
}

void
GPUCoalescer::removeRequest(GPUCoalescerRequest* srequest)
{
    assert(m_outstanding_count ==
           m_writeRequestTable.size() + m_readRequestTable.size());

    Addr line_addr = makeLineAddress(srequest->pkt->getAddr());
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
GPUCoalescer::handleLlsc(Addr address, GPUCoalescerRequest* request)
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
GPUCoalescer::writeCallback(Addr address, DataBlock& data)
{
    writeCallback(address, MachineType_NULL, data);
}

void
GPUCoalescer::writeCallback(Addr address,
                         MachineType mach,
                         DataBlock& data)
{
    writeCallback(address, mach, data, Cycles(0), Cycles(0), Cycles(0));
}

void
GPUCoalescer::writeCallback(Addr address,
                         MachineType mach,
                         DataBlock& data,
                         Cycles initialRequestTime,
                         Cycles forwardRequestTime,
                         Cycles firstResponseTime)
{
    writeCallback(address, mach, data,
                  initialRequestTime, forwardRequestTime, firstResponseTime,
                  false);
}

void
GPUCoalescer::writeCallback(Addr address,
                         MachineType mach,
                         DataBlock& data,
                         Cycles initialRequestTime,
                         Cycles forwardRequestTime,
                         Cycles firstResponseTime,
                         bool isRegion)
{
    assert(address == makeLineAddress(address));

    DPRINTF(GPUCoalescer, "write callback for address %#x\n", address);
    assert(m_writeRequestTable.count(makeLineAddress(address)));

    RequestTable::iterator i = m_writeRequestTable.find(address);
    assert(i != m_writeRequestTable.end());
    GPUCoalescerRequest* request = i->second;

    m_writeRequestTable.erase(i);
    markRemoved();

    assert((request->m_type == RubyRequestType_ST) ||
           (request->m_type == RubyRequestType_ATOMIC) ||
           (request->m_type == RubyRequestType_ATOMIC_RETURN) ||
           (request->m_type == RubyRequestType_ATOMIC_NO_RETURN) ||
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

    if (request->m_type == RubyRequestType_Locked_RMW_Read) {
        m_controller->blockOnQueue(address, m_mandatory_q_ptr);
    } else if (request->m_type == RubyRequestType_Locked_RMW_Write) {
        m_controller->unblock(address);
    }

    hitCallback(request, mach, data, success,
                request->issue_time, forwardRequestTime, firstResponseTime,
                isRegion);
}

void
GPUCoalescer::readCallback(Addr address, DataBlock& data)
{
    readCallback(address, MachineType_NULL, data);
}

void
GPUCoalescer::readCallback(Addr address,
                        MachineType mach,
                        DataBlock& data)
{
    readCallback(address, mach, data, Cycles(0), Cycles(0), Cycles(0));
}

void
GPUCoalescer::readCallback(Addr address,
                        MachineType mach,
                        DataBlock& data,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime)
{

    readCallback(address, mach, data,
                 initialRequestTime, forwardRequestTime, firstResponseTime,
                 false);
}

void
GPUCoalescer::readCallback(Addr address,
                        MachineType mach,
                        DataBlock& data,
                        Cycles initialRequestTime,
                        Cycles forwardRequestTime,
                        Cycles firstResponseTime,
                        bool isRegion)
{
    assert(address == makeLineAddress(address));
    assert(m_readRequestTable.count(makeLineAddress(address)));

    DPRINTF(GPUCoalescer, "read callback for address %#x\n", address);
    RequestTable::iterator i = m_readRequestTable.find(address);
    assert(i != m_readRequestTable.end());
    GPUCoalescerRequest* request = i->second;

    m_readRequestTable.erase(i);
    markRemoved();

    assert((request->m_type == RubyRequestType_LD) ||
           (request->m_type == RubyRequestType_IFETCH));

    hitCallback(request, mach, data, true,
                request->issue_time, forwardRequestTime, firstResponseTime,
                isRegion);
}

void
GPUCoalescer::hitCallback(GPUCoalescerRequest* srequest,
                       MachineType mach,
                       DataBlock& data,
                       bool success,
                       Cycles initialRequestTime,
                       Cycles forwardRequestTime,
                       Cycles firstResponseTime,
                       bool isRegion)
{
    PacketPtr pkt = srequest->pkt;
    Addr request_address = pkt->getAddr();
    Addr request_line_address = makeLineAddress(request_address);

    RubyRequestType type = srequest->m_type;

    // Set this cache entry to the most recently used
    if (type == RubyRequestType_IFETCH) {
        if (m_instCache_ptr->isTagPresent(request_line_address))
            m_instCache_ptr->setMRU(request_line_address);
    } else {
        if (m_dataCache_ptr->isTagPresent(request_line_address))
            m_dataCache_ptr->setMRU(request_line_address);
    }

    recordMissLatency(srequest, mach,
                      initialRequestTime,
                      forwardRequestTime,
                      firstResponseTime,
                      success, isRegion);
    // update the data
    //
    // MUST AD DOING THIS FOR EACH REQUEST IN COALESCER
    int len = reqCoalescer[request_line_address].size();
    std::vector<PacketPtr> mylist;
    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = reqCoalescer[request_line_address][i].pkt;
        assert(type == reqCoalescer[request_line_address][i].primaryType);
        request_address = pkt->getAddr();
        request_line_address = makeLineAddress(pkt->getAddr());
        if (pkt->getPtr<uint8_t>()) {
            if ((type == RubyRequestType_LD) ||
                (type == RubyRequestType_ATOMIC) ||
                (type == RubyRequestType_ATOMIC_RETURN) ||
                (type == RubyRequestType_IFETCH) ||
                (type == RubyRequestType_RMW_Read) ||
                (type == RubyRequestType_Locked_RMW_Read) ||
                (type == RubyRequestType_Load_Linked)) {
                pkt->setData(
                    data.getData(getOffset(request_address), pkt->getSize()));
            } else {
                data.setData(pkt->getPtr<uint8_t>(),
                             getOffset(request_address), pkt->getSize());
            }
        } else {
            DPRINTF(MemoryAccess,
                    "WARNING.  Data not transfered from Ruby to M5 for type " \
                    "%s\n",
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
                safe_cast<RubyTester::SenderState*>(requestSenderState->predecessor);
            testerSenderState->subBlock.mergeFrom(data);
        }

        mylist.push_back(pkt);
    }
    delete srequest;
    reqCoalescer.erase(request_line_address);
    assert(!reqCoalescer.count(request_line_address));



    completeHitCallback(mylist, len);
}

bool
GPUCoalescer::empty() const
{
    return m_writeRequestTable.empty() && m_readRequestTable.empty();
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
GPUCoalescer::makeRequest(PacketPtr pkt)
{
    // Check for GPU Barrier Kernel End or Kernel Begin
    // Leave these to be handled by the child class
    // Kernel End/Barrier = isFlush + isRelease
    // Kernel Begin = isFlush + isAcquire
    if (pkt->req->isKernel()) {
        if (pkt->req->isAcquire()){
            // This is a Kernel Begin leave handling to
            // virtual xCoalescer::makeRequest
            return RequestStatus_Issued;
        }else if (pkt->req->isRelease()) {
            // This is a Kernel End leave handling to
            // virtual xCoalescer::makeRequest
            // If we are here then we didn't call
            // a virtual version of this function
            // so we will also schedule the callback
            int wf_id = 0;
            if (pkt->req->hasContextId()) {
                wf_id = pkt->req->contextId();
            }
            insertKernel(wf_id, pkt);
            newKernelEnds.push_back(wf_id);
            if (!issueEvent.scheduled()) {
                schedule(issueEvent, curTick());
            }
            return RequestStatus_Issued;
        }
    }

    // If number of outstanding requests greater than the max allowed,
    // return RequestStatus_BufferFull. This logic can be extended to
    // support proper backpressure.
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
            primary_type = RubyRequestType_Store_Conditional;
        } else {
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
            primary_type = RubyRequestType_Locked_RMW_Write;
        } else {
            assert(pkt->isRead());
            primary_type = RubyRequestType_Locked_RMW_Read;
        }
        secondary_type = RubyRequestType_ST;
    } else if (pkt->isAtomicOp()) {
        //
        // GPU Atomic Operation
        //
        primary_type = RubyRequestType_ATOMIC;
        secondary_type = RubyRequestType_ATOMIC;
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
        } else if (pkt->req->isRelease() || pkt->req->isAcquire()) {
            if (assumingRfOCoherence) {
                // If we reached here, this request must be a memFence
                // and the protocol implements RfO, the coalescer can
                // assume sequentially consistency and schedule the callback
                // immediately.
                // Currently the code implements fence callbacks
                // by reusing the mechanism for kernel completions.
                // This should be fixed.
                int wf_id = 0;
                if (pkt->req->hasContextId()) {
                    wf_id = pkt->req->contextId();
                }
                insertKernel(wf_id, pkt);
                newKernelEnds.push_back(wf_id);
                if (!issueEvent.scheduled()) {
                    schedule(issueEvent, curTick());
                }
                return RequestStatus_Issued;
            } else {
                // If not RfO, return issued here and let the child coalescer
                // take care of it.
                return RequestStatus_Issued;
            }
        } else {
            panic("Unsupported ruby packet type\n");
        }
    }

    // Check if there is any pending request to this cache line from
    // previous cycles.
    // If there is a pending request, return aliased. Since coalescing
    // across time is not permitted, aliased requests are not coalesced.
    // If a request for this address has already been issued, we must block
    RequestStatus status = getRequestStatus(pkt, primary_type);
    if (status != RequestStatus_Ready)
        return status;

    Addr line_addr = makeLineAddress(pkt->getAddr());

    // Check if this request can be coalesced with previous
    // requests from this cycle.
    if (!reqCoalescer.count(line_addr)) {
        // This is the first access to this cache line.
        // A new request to the memory subsystem has to be
        // made in the next cycle for this cache line, so
        // add this line addr to the "newRequests" queue
        newRequests.push_back(line_addr);

    // There was a request to this cache line in this cycle,
    // let us see if we can coalesce this request with the previous
    // requests from this cycle
    } else if (primary_type !=
               reqCoalescer[line_addr][0].primaryType) {
        // can't coalesce loads, stores and atomics!
        return RequestStatus_Aliased;
    } else if (pkt->req->isLockedRMW() ||
               reqCoalescer[line_addr][0].pkt->req->isLockedRMW()) {
        // can't coalesce locked accesses, but can coalesce atomics!
        return RequestStatus_Aliased;
    } else if (pkt->req->hasContextId() && pkt->req->isRelease() &&
               pkt->req->contextId() !=
               reqCoalescer[line_addr][0].pkt->req->contextId()) {
        // can't coalesce releases from different wavefronts
        return RequestStatus_Aliased;
    }

    // in addition to the packet, we need to save both request types
    reqCoalescer[line_addr].emplace_back(pkt, primary_type, secondary_type);
    if (!issueEvent.scheduled())
        schedule(issueEvent, curTick());
    // TODO: issue hardware prefetches here
    return RequestStatus_Issued;
}

void
GPUCoalescer::issueRequest(PacketPtr pkt, RubyRequestType secondary_type)
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

    // At the moment setting scopes only counts
    // for GPU spill space accesses
    // which is pkt->req->isStack()
    // this scope is REPLACE since it
    // does not need to be flushed at the end
    // of a kernel Private and local may need
    // to be visible at the end of the kernel
    HSASegment accessSegment = reqSegmentToHSASegment(pkt->req);
    HSAScope accessScope = reqScopeToHSAScope(pkt->req);

    Addr line_addr = makeLineAddress(pkt->getAddr());

    // Creating WriteMask that records written bytes
    // and atomic operations. This enables partial writes
    // and partial reads of those writes
    DataBlock dataBlock;
    dataBlock.clear();
    uint32_t blockSize = RubySystem::getBlockSizeBytes();
    std::vector<bool> accessMask(blockSize,false);
    std::vector< std::pair<int,AtomicOpFunctor*> > atomicOps;
    uint32_t tableSize = reqCoalescer[line_addr].size();
    for (int i = 0; i < tableSize; i++) {
        PacketPtr tmpPkt = reqCoalescer[line_addr][i].pkt;
        uint32_t tmpOffset = (tmpPkt->getAddr()) - line_addr;
        uint32_t tmpSize = tmpPkt->getSize();
        if (tmpPkt->isAtomicOp()) {
            std::pair<int,AtomicOpFunctor *> tmpAtomicOp(tmpOffset,
                                                        tmpPkt->getAtomicOp());
            atomicOps.push_back(tmpAtomicOp);
        } else if (tmpPkt->isWrite()) {
            dataBlock.setData(tmpPkt->getPtr<uint8_t>(),
                              tmpOffset, tmpSize);
        }
        for (int j = 0; j < tmpSize; j++) {
            accessMask[tmpOffset + j] = true;
        }
    }
    std::shared_ptr<RubyRequest> msg;
    if (pkt->isAtomicOp()) {
        msg = std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                              pkt->getPtr<uint8_t>(),
                              pkt->getSize(), pc, secondary_type,
                              RubyAccessMode_Supervisor, pkt,
                              PrefetchBit_No, proc_id, 100,
                              blockSize, accessMask,
                              dataBlock, atomicOps,
                              accessScope, accessSegment);
    } else {
        msg = std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                              pkt->getPtr<uint8_t>(),
                              pkt->getSize(), pc, secondary_type,
                              RubyAccessMode_Supervisor, pkt,
                              PrefetchBit_No, proc_id, 100,
                              blockSize, accessMask,
                              dataBlock,
                              accessScope, accessSegment);
    }
    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %s\n",
             curTick(), m_version, "Coal", "Begin", "", "",
             printAddress(msg->getPhysicalAddress()),
             RubyRequestType_to_string(secondary_type));

    fatal_if(secondary_type == RubyRequestType_IFETCH,
             "there should not be any I-Fetch requests in the GPU Coalescer");

    // Send the message to the cache controller
    fatal_if(m_data_cache_hit_latency == 0,
             "should not have a latency of zero");

    assert(m_mandatory_q_ptr);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), m_data_cache_hit_latency);
}

template <class KEY, class VALUE>
std::ostream &
operator<<(ostream &out, const std::unordered_map<KEY, VALUE> &map)
{
    out << "[";
    for (auto i = map.begin(); i != map.end(); ++i)
        out << " " << i->first << "=" << i->second;
    out << " ]";

    return out;
}

void
GPUCoalescer::print(ostream& out) const
{
    out << "[GPUCoalescer: " << m_version
        << ", outstanding requests: " << m_outstanding_count
        << ", read request table: " << m_readRequestTable
        << ", write request table: " << m_writeRequestTable
        << "]";
}

// this can be called from setState whenever coherence permissions are
// upgraded when invoked, coherence violations will be checked for the
// given block
void
GPUCoalescer::checkCoherence(Addr addr)
{
#ifdef CHECK_COHERENCE
    m_ruby_system->checkGlobalCoherenceInvariant(addr);
#endif
}

void
GPUCoalescer::recordRequestType(SequencerRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            SequencerRequestType_to_string(requestType));
}


void
GPUCoalescer::completeIssue()
{
    // newRequests has the cacheline addresses of all the
    // requests which need to be issued to the memory subsystem
    // in this cycle
    int len = newRequests.size();
    DPRINTF(GPUCoalescer, "Completing issue for %d new requests.\n", len);
    for (int i = 0; i < len; ++i) {
        // Get the requests from reqCoalescer table. Get only the
        // first request for each cacheline, the remaining requests
        // can be coalesced with the first request. So, only
        // one request is issued per cacheline.
        RequestDesc info = reqCoalescer[newRequests[i]][0];
        PacketPtr pkt = info.pkt;
        DPRINTF(GPUCoalescer, "Completing for newReq %d: paddr %#x\n",
                i, pkt->req->getPaddr());
        // Insert this request to the read/writeRequestTables. These tables
        // are used to track aliased requests in makeRequest subroutine
        bool found = insertRequest(pkt, info.primaryType);

        if (found) {
            panic("GPUCoalescer::makeRequest should never be called if the "
                  "request is already outstanding\n");
        }

        // Issue request to ruby subsystem
        issueRequest(pkt, info.secondaryType);
    }
    newRequests.clear();

    // have Kernel End releases been issued this cycle
    len = newKernelEnds.size();
    for (int i = 0; i < len; i++) {
        kernelCallback(newKernelEnds[i]);
    }
    newKernelEnds.clear();
}

void
GPUCoalescer::evictionCallback(Addr address)
{
    ruby_eviction_callback(address);
}

void
GPUCoalescer::kernelCallback(int wavefront_id)
{
    assert(kernelEndList.count(wavefront_id));

    ruby_hit_callback(kernelEndList[wavefront_id]);

    kernelEndList.erase(wavefront_id);
}

void
GPUCoalescer::atomicCallback(Addr address,
                             MachineType mach,
                             const DataBlock& data)
{
    assert(address == makeLineAddress(address));

    DPRINTF(GPUCoalescer, "atomic callback for address %#x\n", address);
    assert(m_writeRequestTable.count(makeLineAddress(address)));

    RequestTable::iterator i = m_writeRequestTable.find(address);
    assert(i != m_writeRequestTable.end());
    GPUCoalescerRequest* srequest = i->second;

    m_writeRequestTable.erase(i);
    markRemoved();

    assert((srequest->m_type == RubyRequestType_ATOMIC) ||
           (srequest->m_type == RubyRequestType_ATOMIC_RETURN) ||
           (srequest->m_type == RubyRequestType_ATOMIC_NO_RETURN));


    // Atomics don't write to cache, so there is no MRU update...

    recordMissLatency(srequest, mach,
                      srequest->issue_time, Cycles(0), Cycles(0), true, false);

    PacketPtr pkt = srequest->pkt;
    Addr request_address = pkt->getAddr();
    Addr request_line_address = makeLineAddress(pkt->getAddr());

    int len = reqCoalescer[request_line_address].size();
    std::vector<PacketPtr> mylist;
    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = reqCoalescer[request_line_address][i].pkt;
        assert(srequest->m_type ==
               reqCoalescer[request_line_address][i].primaryType);
        request_address = (pkt->getAddr());
        request_line_address = makeLineAddress(request_address);
        if (pkt->getPtr<uint8_t>() &&
            srequest->m_type != RubyRequestType_ATOMIC_NO_RETURN) {
            /* atomics are done in memory, and return the data *before* the atomic op... */
            pkt->setData(
                data.getData(getOffset(request_address), pkt->getSize()));
        } else {
            DPRINTF(MemoryAccess,
                    "WARNING.  Data not transfered from Ruby to M5 for type " \
                    "%s\n",
                    RubyRequestType_to_string(srequest->m_type));
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
                safe_cast<RubyTester::SenderState*>(requestSenderState->predecessor);
            testerSenderState->subBlock.mergeFrom(data);
        }

        mylist.push_back(pkt);
    }
    delete srequest;
    reqCoalescer.erase(request_line_address);
    assert(!reqCoalescer.count(request_line_address));

    completeHitCallback(mylist, len);
}

void
GPUCoalescer::recordCPReadCallBack(MachineID myMachID, MachineID senderMachID)
{
    if (myMachID == senderMachID) {
        CP_TCPLdHits++;
    } else if (machineIDToMachineType(senderMachID) == MachineType_TCP) {
        CP_TCPLdTransfers++;
    } else if (machineIDToMachineType(senderMachID) == MachineType_TCC) {
        CP_TCCLdHits++;
    } else {
        CP_LdMiss++;
    }
}

void
GPUCoalescer::recordCPWriteCallBack(MachineID myMachID, MachineID senderMachID)
{
    if (myMachID == senderMachID) {
        CP_TCPStHits++;
    } else if (machineIDToMachineType(senderMachID) == MachineType_TCP) {
        CP_TCPStTransfers++;
    } else if (machineIDToMachineType(senderMachID) == MachineType_TCC) {
        CP_TCCStHits++;
    } else {
        CP_StMiss++;
    }
}

void
GPUCoalescer::completeHitCallback(std::vector<PacketPtr> & mylist, int len)
{
    for (int i = 0; i < len; ++i) {
        RubyPort::SenderState *ss =
            safe_cast<RubyPort::SenderState *>(mylist[i]->senderState);
        MemSlavePort *port = ss->port;
        assert(port != NULL);

        mylist[i]->senderState = ss->predecessor;
        delete ss;
        port->hitCallback(mylist[i]);
        trySendRetries();
    }

    testDrainComplete();
}

PacketPtr
GPUCoalescer::mapAddrToPkt(Addr address)
{
    RequestTable::iterator i = m_readRequestTable.find(address);
    assert(i != m_readRequestTable.end());
    GPUCoalescerRequest* request = i->second;
    return request->pkt;
}

void
GPUCoalescer::recordMissLatency(GPUCoalescerRequest* srequest,
                                MachineType mach,
                                Cycles initialRequestTime,
                                Cycles forwardRequestTime,
                                Cycles firstResponseTime,
                                bool success, bool isRegion)
{
    RubyRequestType type = srequest->m_type;
    Cycles issued_time = srequest->issue_time;
    Cycles completion_time = curCycle();
    assert(completion_time >= issued_time);
    Cycles total_lat = completion_time - issued_time;

    // cache stats (valid for RfO protocol only)
    if (mach == MachineType_TCP) {
        if (type == RubyRequestType_LD) {
            GPU_TCPLdHits++;
        } else {
            GPU_TCPStHits++;
        }
    } else if (mach == MachineType_L1Cache_wCC) {
        if (type == RubyRequestType_LD) {
            GPU_TCPLdTransfers++;
        } else {
            GPU_TCPStTransfers++;
        }
    } else if (mach == MachineType_TCC) {
        if (type == RubyRequestType_LD) {
            GPU_TCCLdHits++;
        } else {
            GPU_TCCStHits++;
        }
    } else  {
        if (type == RubyRequestType_LD) {
            GPU_LdMiss++;
        } else {
            GPU_StMiss++;
        }
    }

    // Profile all access latency, even zero latency accesses
    m_latencyHist.sample(total_lat);
    m_typeLatencyHist[type]->sample(total_lat);

    // Profile the miss latency for all non-zero demand misses
    if (total_lat != Cycles(0)) {
        m_missLatencyHist.sample(total_lat);
        m_missTypeLatencyHist[type]->sample(total_lat);

        if (mach != MachineType_NUM) {
            m_missMachLatencyHist[mach]->sample(total_lat);
            m_missTypeMachLatencyHist[type][mach]->sample(total_lat);

            if ((issued_time <= initialRequestTime) &&
                (initialRequestTime <= forwardRequestTime) &&
                (forwardRequestTime <= firstResponseTime) &&
                (firstResponseTime <= completion_time)) {

                m_IssueToInitialDelayHist[mach]->sample(
                    initialRequestTime - issued_time);
                m_InitialToForwardDelayHist[mach]->sample(
                    forwardRequestTime - initialRequestTime);
                m_ForwardToFirstResponseDelayHist[mach]->sample(
                    firstResponseTime - forwardRequestTime);
                m_FirstResponseToCompletionDelayHist[mach]->sample(
                    completion_time - firstResponseTime);
            }
        }

    }

    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %d cycles\n",
             curTick(), m_version, "Coal",
             success ? "Done" : "SC_Failed", "", "",
             printAddress(srequest->pkt->getAddr()), total_lat);
}

void
GPUCoalescer::regStats()
{
    RubyPort::regStats();

    // These statistical variables are not for display.
    // The profiler will collate these across different
    // coalescers and display those collated statistics.
    m_outstandReqHist.init(10);
    m_latencyHist.init(10);
    m_missLatencyHist.init(10);

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist.push_back(new Stats::Histogram());
        m_typeLatencyHist[i]->init(10);

        m_missTypeLatencyHist.push_back(new Stats::Histogram());
        m_missTypeLatencyHist[i]->init(10);
    }

    for (int i = 0; i < MachineType_NUM; i++) {
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
        m_missTypeMachLatencyHist.push_back(std::vector<Stats::Histogram *>());

        for (int j = 0; j < MachineType_NUM; j++) {
            m_missTypeMachLatencyHist[i].push_back(new Stats::Histogram());
            m_missTypeMachLatencyHist[i][j]->init(10);
        }
    }

    // GPU cache stats
    GPU_TCPLdHits
        .name(name() + ".gpu_tcp_ld_hits")
        .desc("loads that hit in the TCP")
        ;
    GPU_TCPLdTransfers
        .name(name() + ".gpu_tcp_ld_transfers")
        .desc("TCP to TCP load transfers")
        ;
    GPU_TCCLdHits
        .name(name() + ".gpu_tcc_ld_hits")
        .desc("loads that hit in the TCC")
        ;
    GPU_LdMiss
        .name(name() + ".gpu_ld_misses")
        .desc("loads that miss in the GPU")
        ;

    GPU_TCPStHits
        .name(name() + ".gpu_tcp_st_hits")
        .desc("stores that hit in the TCP")
        ;
    GPU_TCPStTransfers
        .name(name() + ".gpu_tcp_st_transfers")
        .desc("TCP to TCP store transfers")
        ;
    GPU_TCCStHits
        .name(name() + ".gpu_tcc_st_hits")
        .desc("stores that hit in the TCC")
        ;
    GPU_StMiss
        .name(name() + ".gpu_st_misses")
        .desc("stores that miss in the GPU")
        ;

    // CP cache stats
    CP_TCPLdHits
        .name(name() + ".cp_tcp_ld_hits")
        .desc("loads that hit in the TCP")
        ;
    CP_TCPLdTransfers
        .name(name() + ".cp_tcp_ld_transfers")
        .desc("TCP to TCP load transfers")
        ;
    CP_TCCLdHits
        .name(name() + ".cp_tcc_ld_hits")
        .desc("loads that hit in the TCC")
        ;
    CP_LdMiss
        .name(name() + ".cp_ld_misses")
        .desc("loads that miss in the GPU")
        ;

    CP_TCPStHits
        .name(name() + ".cp_tcp_st_hits")
        .desc("stores that hit in the TCP")
        ;
    CP_TCPStTransfers
        .name(name() + ".cp_tcp_st_transfers")
        .desc("TCP to TCP store transfers")
        ;
    CP_TCCStHits
        .name(name() + ".cp_tcc_st_hits")
        .desc("stores that hit in the TCC")
        ;
    CP_StMiss
        .name(name() + ".cp_st_misses")
        .desc("stores that miss in the GPU")
        ;
}
