/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#include "mem/ruby/system/GPUCoalescer.hh"

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/str.hh"
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

namespace gem5
{

namespace ruby
{

UncoalescedTable::UncoalescedTable(GPUCoalescer *gc)
    : coalescer(gc)
{
}

void
UncoalescedTable::insertPacket(PacketPtr pkt)
{
    uint64_t seqNum = pkt->req->getReqInstSeqNum();

    instMap[seqNum].push_back(pkt);
    DPRINTF(GPUCoalescer, "Adding 0x%X seqNum %d to map. (map %d vec %d)\n",
            pkt->getAddr(), seqNum, instMap.size(), instMap[seqNum].size());
}

void
UncoalescedTable::insertReqType(PacketPtr pkt, RubyRequestType type)
{
    uint64_t seqNum = pkt->req->getReqInstSeqNum();

    reqTypeMap[seqNum] = type;
}

bool
UncoalescedTable::packetAvailable()
{
    return !instMap.empty();
}

void
UncoalescedTable::initPacketsRemaining(InstSeqNum seqNum, int count)
{
    if (!instPktsRemaining.count(seqNum)) {
        instPktsRemaining[seqNum] = count;
    }
}

int
UncoalescedTable::getPacketsRemaining(InstSeqNum seqNum)
{
    return instPktsRemaining[seqNum];
}

void
UncoalescedTable::setPacketsRemaining(InstSeqNum seqNum, int count)
{
    instPktsRemaining[seqNum] = count;
}

PerInstPackets*
UncoalescedTable::getInstPackets(int offset)
{
    if (offset >= instMap.size()) {
        return nullptr;
    }

    auto instMapIter = instMap.begin();
    std::advance(instMapIter, offset);

    return &(instMapIter->second);
}

void
UncoalescedTable::updateResources()
{
    for (auto iter = instMap.begin(); iter != instMap.end(); ) {
        InstSeqNum seq_num = iter->first;
        DPRINTF(GPUCoalescer, "%s checking remaining pkts for %d\n",
                coalescer->name().c_str(), seq_num);
        assert(instPktsRemaining.count(seq_num));

        if (instPktsRemaining[seq_num] == 0) {
            assert(iter->second.empty());

            // Remove from both maps
            instMap.erase(iter++);
            instPktsRemaining.erase(seq_num);

            // Release the token if the Ruby system is not in cooldown
            // or warmup phases. When in these phases, the RubyPorts
            // are accessed directly using the makeRequest() command
            // instead of accessing through the port. This makes
            // sending tokens through the port unnecessary
            if (!RubySystem::getWarmupEnabled()
                    && !RubySystem::getCooldownEnabled()) {
                if (reqTypeMap[seq_num] != RubyRequestType_FLUSH) {
                    DPRINTF(GPUCoalescer,
                            "Returning token seqNum %d\n", seq_num);
                    coalescer->getGMTokenPort().sendTokens(1);
                }
            }

            reqTypeMap.erase(seq_num);
        } else {
            ++iter;
        }
    }
}

bool
UncoalescedTable::areRequestsDone(const uint64_t instSeqNum) {
    // iterate the instructions held in UncoalescedTable to see whether there
    // are more requests to issue; if yes, not yet done; otherwise, done
    for (auto& inst : instMap) {
        DPRINTF(GPUCoalescer, "instSeqNum= %d, pending packets=%d\n"
            ,inst.first, inst.second.size());
        if (inst.first == instSeqNum) { return false; }
    }

    return true;
}

void
UncoalescedTable::printRequestTable(std::stringstream& ss)
{
    ss << "Listing pending packets from " << instMap.size() << " instructions";

    for (auto& inst : instMap) {
        ss << "\tAddr: " << printAddress(inst.first) << " with "
           << inst.second.size() << " pending packets" << std::endl;
    }
}

void
UncoalescedTable::checkDeadlock(Tick threshold)
{
    Tick current_time = curTick();

    for (auto &it : instMap) {
        for (auto &pkt : it.second) {
            if (current_time - pkt->req->time() > threshold) {
                std::stringstream ss;
                printRequestTable(ss);

                panic("Possible Deadlock detected. Aborting!\n"
                     "version: %d request.paddr: 0x%x uncoalescedTable: %d "
                     "current time: %u issue_time: %d difference: %d\n"
                     "Request Tables:\n\n%s", coalescer->getId(),
                      pkt->getAddr(), instMap.size(), current_time,
                      pkt->req->time(), current_time - pkt->req->time(),
                      ss.str());
            }
        }
    }
}

GPUCoalescer::GPUCoalescer(const Params &p)
    : RubyPort(p),
      issueEvent([this]{ completeIssue(); }, "Issue coalesced request",
                 false, Event::Progress_Event_Pri),
      uncoalescedTable(this),
      deadlockCheckEvent([this]{ wakeup(); }, "GPUCoalescer deadlock check"),
      gmTokenPort(name() + ".gmTokenPort")
{
    m_store_waiting_on_load_cycles = 0;
    m_store_waiting_on_store_cycles = 0;
    m_load_waiting_on_store_cycles = 0;
    m_load_waiting_on_load_cycles = 0;

    m_outstanding_count = 0;

    coalescingWindow = p.max_coalesces_per_cycle;

    m_max_outstanding_requests = 0;
    m_instCache_ptr = nullptr;
    m_dataCache_ptr = nullptr;

    m_instCache_ptr = p.icache;
    m_dataCache_ptr = p.dcache;
    m_max_outstanding_requests = p.max_outstanding_requests;
    m_deadlock_threshold = p.deadlock_threshold;

    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr);
    assert(m_dataCache_ptr);

    m_runningGarnetStandalone = p.garnet_standalone;


    // These statistical variables are not for display.
    // The profiler will collate these across different
    // coalescers and display those collated statistics.
    m_outstandReqHist.init(10);
    m_latencyHist.init(10);
    m_missLatencyHist.init(10);

    for (int i = 0; i < RubyRequestType_NUM; i++) {
        m_typeLatencyHist.push_back(new statistics::Histogram());
        m_typeLatencyHist[i]->init(10);

        m_missTypeLatencyHist.push_back(new statistics::Histogram());
        m_missTypeLatencyHist[i]->init(10);
    }

    for (int i = 0; i < MachineType_NUM; i++) {
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
        m_missTypeMachLatencyHist.push_back(
            std::vector<statistics::Histogram *>());

        for (int j = 0; j < MachineType_NUM; j++) {
            m_missTypeMachLatencyHist[i].push_back(
                new statistics::Histogram());
            m_missTypeMachLatencyHist[i][j]->init(10);
        }
    }

}

GPUCoalescer::~GPUCoalescer()
{
}

Port &
GPUCoalescer::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "gmTokenPort") {
        return gmTokenPort;
    }

    // delgate to RubyPort otherwise
    return RubyPort::getPort(if_name, idx);
}

void
GPUCoalescer::wakeup()
{
    Cycles current_time = curCycle();
    for (auto& requestList : coalescedTable) {
        for (auto& req : requestList.second) {
            if (current_time - req->getIssueTime() > m_deadlock_threshold) {
                std::stringstream ss;
                printRequestTable(ss);
                warn("GPUCoalescer %d Possible deadlock detected!\n%s\n",
                     m_version, ss.str());
                panic("Aborting due to deadlock!\n");
            }
        }
    }

    Tick tick_threshold = cyclesToTicks(m_deadlock_threshold);
    uncoalescedTable.checkDeadlock(tick_threshold);

    if (m_outstanding_count > 0) {
        schedule(deadlockCheckEvent,
                 m_deadlock_threshold * clockPeriod() +
                 curTick());
    }
}

void
GPUCoalescer::printRequestTable(std::stringstream& ss)
{
    ss << "Printing out " << coalescedTable.size()
       << " outstanding requests in the coalesced table\n";

    for (auto& requestList : coalescedTable) {
        for (auto& request : requestList.second) {
            ss << "\tAddr: " << printAddress(requestList.first) << "\n"
               << "\tInstruction sequence number: "
               << request->getSeqNum() << "\n"
               << "\t\tType: "
               << RubyRequestType_to_string(request->getRubyType()) << "\n"
               << "\t\tNumber of associated packets: "
               << request->getPackets().size() << "\n"
               << "\t\tIssue time: "
               << request->getIssueTime() * clockPeriod() << "\n"
               << "\t\tDifference from current tick: "
               << (curCycle() - request->getIssueTime()) * clockPeriod()
               << "\n";
        }
    }

    // print out packets waiting to be issued in uncoalesced table
    uncoalescedTable.printRequestTable(ss);
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
GPUCoalescer::printProgress(std::ostream& out) const
{
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
    assert(coalescedTable.count(address));

    auto crequest = coalescedTable.at(address).front();

    hitCallback(crequest, mach, data, true, crequest->getIssueTime(),
                forwardRequestTime, firstResponseTime, isRegion);

    // remove this crequest in coalescedTable
    delete crequest;
    coalescedTable.at(address).pop_front();

    if (coalescedTable.at(address).empty()) {
        coalescedTable.erase(address);
    } else {
        auto nextRequest = coalescedTable.at(address).front();
        issueRequest(nextRequest);
    }
}

void
GPUCoalescer::writeCompleteCallback(Addr address,
                                    uint64_t instSeqNum,
                                    MachineType mach)
{
    DPRINTF(GPUCoalescer, "writeCompleteCallback for address 0x%x"
            " instSeqNum = %d\n", address, instSeqNum);

    assert(pendingWriteInsts.count(instSeqNum) == 1);
    PendingWriteInst& inst = pendingWriteInsts[instSeqNum];

    // check the uncoalescedTable to see whether all requests for the inst
    // have been issued or not
    bool reqsAllIssued = uncoalescedTable.areRequestsDone(instSeqNum);
    DPRINTF(GPUCoalescer, "instSeqNum = %d, pendingStores=%d, "
                    "reqsAllIssued=%d\n", reqsAllIssued,
                    inst.getNumPendingStores()-1, reqsAllIssued);

    if (inst.receiveWriteCompleteAck() && reqsAllIssued ) {
        // if the pending write instruction has received all write completion
        // callbacks for its issued Ruby requests, we can now start respond
        // the requesting CU in one response packet.
        inst.ackWriteCompletion(m_usingRubyTester);

        DPRINTF(GPUCoalescer, "write inst %d completed at coalescer\n",
                instSeqNum);
        pendingWriteInsts.erase(instSeqNum);
    }
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
    assert(coalescedTable.count(address));

    auto crequest = coalescedTable.at(address).front();
    fatal_if(crequest->getRubyType() != RubyRequestType_LD,
             "readCallback received non-read type response\n");

    // Iterate over the coalesced requests to respond to as many loads as
    // possible until another request type is seen. Models MSHR for TCP.
    while (crequest->getRubyType() == RubyRequestType_LD) {
        hitCallback(crequest, mach, data, true, crequest->getIssueTime(),
                    forwardRequestTime, firstResponseTime, isRegion);

        delete crequest;
        coalescedTable.at(address).pop_front();
        if (coalescedTable.at(address).empty()) {
            break;
        }

        crequest = coalescedTable.at(address).front();
    }

    if (coalescedTable.at(address).empty()) {
        coalescedTable.erase(address);
    } else {
        auto nextRequest = coalescedTable.at(address).front();
        issueRequest(nextRequest);
    }
}

void
GPUCoalescer::hitCallback(CoalescedRequest* crequest,
                       MachineType mach,
                       DataBlock& data,
                       bool success,
                       Cycles initialRequestTime,
                       Cycles forwardRequestTime,
                       Cycles firstResponseTime,
                       bool isRegion)
{
    PacketPtr pkt = crequest->getFirstPkt();
    Addr request_address = pkt->getAddr();
    [[maybe_unused]] Addr request_line_address =
        makeLineAddress(request_address);

    RubyRequestType type = crequest->getRubyType();

    DPRINTF(GPUCoalescer, "Got hitCallback for 0x%X\n", request_line_address);

    recordMissLatency(crequest, mach,
                      initialRequestTime,
                      forwardRequestTime,
                      firstResponseTime,
                      success, isRegion);
    // update the data
    //
    // MUST ADD DOING THIS FOR EACH REQUEST IN COALESCER
    std::vector<PacketPtr> pktList = crequest->getPackets();

    uint8_t* log = nullptr;
    DPRINTF(GPUCoalescer, "Responding to %d packets for addr 0x%X\n",
            pktList.size(), request_line_address);
    uint32_t offset;
    int pkt_size;
    for (auto& pkt : pktList) {
        offset = getOffset(pkt->getAddr());
        pkt_size = pkt->getSize();
        request_address = pkt->getAddr();

        // When the Ruby system is cooldown phase, the requests come from
        // the cache recorder. These requests do not get coalesced and
        // do not return valid data.
        if (RubySystem::getCooldownEnabled())
            continue;

        if (pkt->getPtr<uint8_t>()) {
            switch(type) {
                // Store and AtomicNoReturns follow the same path, as the
                // data response is not needed.
                case RubyRequestType_ATOMIC_NO_RETURN:
                    assert(pkt->isAtomicOp());
                case RubyRequestType_ST:
                    data.setData(pkt->getPtr<uint8_t>(), offset, pkt_size);
                    break;
                case RubyRequestType_LD:
                    pkt->setData(data.getData(offset, pkt_size));
                    break;
                case RubyRequestType_ATOMIC_RETURN:
                    assert(pkt->isAtomicOp());
                    // Atomic operations are performed by the WriteMask
                    // in packet order, set by the crequest. Thus, when
                    // unpacking the changes from the log, we read from
                    // the front of the log to correctly map response
                    // data into the packets.

                    // Log entry contains the old value before the current
                    // atomic operation occurred.
                    log = data.popAtomicLogEntryFront();
                    pkt->setData(&log[offset]);
                    delete [] log;
                    log = nullptr;
                    break;
                default:
                    panic("Unsupported ruby packet type:%s\n",
                                    RubyRequestType_to_string(type));
                    break;
            }
        } else {
            DPRINTF(MemoryAccess,
                    "WARNING.  Data not transfered from Ruby to M5 for type " \
                    "%s\n",
                    RubyRequestType_to_string(type));
        }
    }
    assert(data.numAtomicLogEntries() == 0);

    m_outstanding_count--;
    assert(m_outstanding_count >= 0);

    completeHitCallback(pktList);
}

bool
GPUCoalescer::empty() const
{
    return coalescedTable.empty();
}

RubyRequestType
GPUCoalescer::getRequestType(PacketPtr pkt)
{
    RubyRequestType req_type = RubyRequestType_NULL;

    // These types are not support or not used in GPU caches.
    assert(!pkt->req->isLLSC());
    assert(!pkt->req->isLockedRMW());
    assert(!pkt->req->isInstFetch());

    if (pkt->req->isAtomicReturn()) {
        req_type = RubyRequestType_ATOMIC_RETURN;
    } else if (pkt->req->isAtomicNoReturn()) {
        req_type = RubyRequestType_ATOMIC_NO_RETURN;
    } else if (pkt->isRead()) {
        req_type = RubyRequestType_LD;
    } else if (pkt->isWrite()) {
        req_type = RubyRequestType_ST;
    } else if (pkt->isFlush()) {
        req_type = RubyRequestType_FLUSH;
    } else {
        panic("Unsupported ruby packet type\n");
    }

    return req_type;
}

// Places an uncoalesced packet in uncoalescedTable. If the packet is a
// special type (MemFence, scoping, etc), it is issued immediately.
RequestStatus
GPUCoalescer::makeRequest(PacketPtr pkt)
{
    // all packets must have valid instruction sequence numbers
    assert(pkt->req->hasInstSeqNum());

    if (pkt->cmd == MemCmd::MemSyncReq) {
        // issue mem_sync requests immediately to the cache system without
        // going through uncoalescedTable like normal LD/ST/Atomic requests
        issueMemSyncRequest(pkt);
    } else {
        // otherwise, this must be either read or write command
        assert(pkt->isRead() || pkt->isWrite() || pkt->isFlush());

        InstSeqNum seq_num = pkt->req->getReqInstSeqNum();

        // in the case of protocol tester, there is one packet per sequence
        // number. The number of packets during simulation depends on the
        // number of lanes actives for that vmem request (i.e., the popcnt
        // of the exec_mask.
        int num_packets = 1;

        // When Ruby is in warmup or cooldown phase, the requests come from
        // the cache recorder. There is no dynamic instruction associated
        // with these requests either
        if (!RubySystem::getWarmupEnabled()
                && !RubySystem::getCooldownEnabled()) {
            if (!m_usingRubyTester) {
                num_packets = 0;
                for (int i = 0; i < TheGpuISA::NumVecElemPerVecReg; i++) {
                    num_packets += getDynInst(pkt)->getLaneStatus(i);
                }
            }
        }

        // the pkt is temporarily stored in the uncoalesced table until
        // it's picked for coalescing process later in this cycle or in a
        // future cycle. Packets remaining is set to the number of excepted
        // requests from the instruction based on its exec_mask.
        uncoalescedTable.insertPacket(pkt);
        uncoalescedTable.insertReqType(pkt, getRequestType(pkt));
        uncoalescedTable.initPacketsRemaining(seq_num, num_packets);
        DPRINTF(GPUCoalescer, "Put pkt with addr 0x%X to uncoalescedTable\n",
                pkt->getAddr());

        // we schedule an issue event here to process the uncoalesced table
        // and try to issue Ruby request to cache system
        if (!issueEvent.scheduled()) {
            DPRINTF(GPUCoalescer, "Scheduled issueEvent for seqNum %d\n",
                    seq_num);
            schedule(issueEvent, curTick());
        }
    }

    // we always return RequestStatus_Issued in this coalescer
    // b/c the coalescer's resouce was checked ealier and the coalescer is
    // queueing up aliased requets in its coalesced table
    return RequestStatus_Issued;
}

template <class KEY, class VALUE>
std::ostream &
operator<<(std::ostream &out, const std::unordered_map<KEY, VALUE> &map)
{
    out << "[";
    for (auto i = map.begin(); i != map.end(); ++i)
        out << " " << i->first << "=" << i->second;
    out << " ]";

    return out;
}

void
GPUCoalescer::print(std::ostream& out) const
{
    out << "[GPUCoalescer: " << m_version
        << ", outstanding requests: " << m_outstanding_count
        << "]";
}

GPUDynInstPtr
GPUCoalescer::getDynInst(PacketPtr pkt) const
{
    RubyPort::SenderState* ss =
            safe_cast<RubyPort::SenderState*>(pkt->senderState);

    ComputeUnit::DataPort::SenderState* cu_state =
        safe_cast<ComputeUnit::DataPort::SenderState*>
            (ss->predecessor);

    return cu_state->_gpuDynInst;
}

bool
GPUCoalescer::coalescePacket(PacketPtr pkt)
{
    uint64_t seqNum = pkt->req->getReqInstSeqNum();
    Addr line_addr = makeLineAddress(pkt->getAddr());

    // If the packet has the same line address as a request already in the
    // coalescedTable and has the same sequence number, it can be coalesced.
    if (coalescedTable.count(line_addr)) {
        // Search for a previous coalesced request with the same seqNum.
        auto& creqQueue = coalescedTable.at(line_addr);
        auto citer = std::find_if(creqQueue.begin(), creqQueue.end(),
            [&](CoalescedRequest* c) { return c->getSeqNum() == seqNum; }
        );
        if (citer != creqQueue.end()) {
            (*citer)->insertPacket(pkt);
            return true;
        }
    }

    if (m_outstanding_count < m_max_outstanding_requests) {
        // This is an "aliased" or new request. Create a RubyRequest and
        // append it to the list of "targets" in the coalescing table.
        DPRINTF(GPUCoalescer, "Creating new or aliased request for 0x%X\n",
                line_addr);

        CoalescedRequest *creq = new CoalescedRequest(seqNum);
        creq->insertPacket(pkt);
        creq->setRubyType(getRequestType(pkt));
        creq->setIssueTime(curCycle());

        if (!coalescedTable.count(line_addr)) {
            // If there is no outstanding request for this line address,
            // create a new coalecsed request and issue it immediately.
            auto reqList = std::deque<CoalescedRequest*> { creq };
            coalescedTable.insert(std::make_pair(line_addr, reqList));
            if (!coalescedReqs.count(seqNum)) {
                coalescedReqs.insert(std::make_pair(seqNum, reqList));
            } else {
                coalescedReqs.at(seqNum).push_back(creq);
            }
        } else {
            // The request is for a line address that is already outstanding
            // but for a different instruction. Add it as a new request to be
            // issued when the current outstanding request is completed.
            coalescedTable.at(line_addr).push_back(creq);
            DPRINTF(GPUCoalescer, "found address 0x%X with new seqNum %d\n",
                    line_addr, seqNum);
        }

        // In both cases, requests are added to the coalescing table and will
        // be counted as outstanding requests.
        m_outstanding_count++;

        // We track all issued or to-be-issued Ruby requests associated with
        // write instructions. An instruction may have multiple Ruby
        // requests.
        if (pkt->cmd == MemCmd::WriteReq) {
            DPRINTF(GPUCoalescer, "adding write inst %d at line 0x%x to"
                    " the pending write instruction list\n", seqNum,
                    line_addr);

            RubyPort::SenderState* ss =
                    safe_cast<RubyPort::SenderState*>(pkt->senderState);

            // we need to save this port because it will be used to call
            // back the requesting CU when we receive write
            // complete callbacks for all issued Ruby requests of this
            // instruction.
            RubyPort::MemResponsePort* mem_response_port = ss->port;

            GPUDynInstPtr gpuDynInst = nullptr;

            if (!m_usingRubyTester) {
                // If this coalescer is connected to a real CU, we need
                // to save the corresponding gpu dynamic instruction.
                // CU will use that instruction to decrement wait counters
                // in the issuing wavefront.
                // For Ruby tester, gpuDynInst == nullptr
                gpuDynInst = getDynInst(pkt);
            }

            PendingWriteInst& inst = pendingWriteInsts[seqNum];
            inst.addPendingReq(mem_response_port, gpuDynInst,
                               m_usingRubyTester);
        }

        return true;
    }

    // The maximum number of outstanding requests have been issued.
    return false;
}

void
GPUCoalescer::completeIssue()
{
    // Iterate over the maximum number of instructions we can coalesce
    // per cycle (coalescingWindow).
    for (int instIdx = 0; instIdx < coalescingWindow; ++instIdx) {
        PerInstPackets *pkt_list =
            uncoalescedTable.getInstPackets(instIdx);

        // getInstPackets will return nullptr if no instruction
        // exists at the current offset.
        if (!pkt_list) {
            break;
        } else if (pkt_list->empty()) {
            // Found something, but it has not been cleaned up by update
            // resources yet. See if there is anything else to coalesce.
            // Assume we can't check anymore if the coalescing window is 1.
            continue;
        } else {
            // All packets in the list have the same seqNum, use first.
            InstSeqNum seq_num = pkt_list->front()->req->getReqInstSeqNum();

            // The difference in list size before and after tells us the
            // number of packets which were coalesced.
            size_t pkt_list_size = pkt_list->size();

            // Since we have a pointer to the list of packets in the inst,
            // erase them from the list if coalescing is successful and
            // leave them in the list otherwise. This aggressively attempts
            // to coalesce as many packets as possible from the current inst.
            pkt_list->remove_if(
                [&](PacketPtr pkt) { return coalescePacket(pkt); }
            );

            if (coalescedReqs.count(seq_num)) {
                auto& creqs = coalescedReqs.at(seq_num);
                for (auto creq : creqs) {
                    DPRINTF(GPUCoalescer, "Issued req type %s seqNum %d\n",
                            RubyRequestType_to_string(creq->getRubyType()),
                                                      seq_num);
                    issueRequest(creq);
                }
                coalescedReqs.erase(seq_num);
            }

            assert(pkt_list_size >= pkt_list->size());
            size_t pkt_list_diff = pkt_list_size - pkt_list->size();

            int num_remaining = uncoalescedTable.getPacketsRemaining(seq_num);
            num_remaining -= pkt_list_diff;
            assert(num_remaining >= 0);

            uncoalescedTable.setPacketsRemaining(seq_num, num_remaining);
            DPRINTF(GPUCoalescer,
                    "Coalesced %d pkts for seqNum %d, %d remaining\n",
                    pkt_list_diff, seq_num, num_remaining);
        }
    }

    // Clean up any instructions in the uncoalesced table that have had
    // all of their packets coalesced and return a token for that column.
    uncoalescedTable.updateResources();

    // have Kernel End releases been issued this cycle
    int len = newKernelEnds.size();
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
    assert(coalescedTable.count(address));

    auto crequest = coalescedTable.at(address).front();

    fatal_if((crequest->getRubyType() != RubyRequestType_ATOMIC &&
              crequest->getRubyType() != RubyRequestType_ATOMIC_RETURN &&
              crequest->getRubyType() != RubyRequestType_ATOMIC_NO_RETURN),
             "atomicCallback saw non-atomic type response\n");

    hitCallback(crequest, mach, (DataBlock&)data, true,
                crequest->getIssueTime(), Cycles(0), Cycles(0), false);

    delete crequest;
    coalescedTable.at(address).pop_front();

    if (coalescedTable.at(address).empty()) {
        coalescedTable.erase(address);
    } else {
        auto nextRequest = coalescedTable.at(address).front();
        issueRequest(nextRequest);
    }
}

void
GPUCoalescer::completeHitCallback(std::vector<PacketPtr> & mylist)
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

    // We schedule an event in the same tick as hitCallback (similar to
    // makeRequest) rather than calling completeIssue directly to reduce
    // function calls to complete issue. This can only happen if the max
    // outstanding requests is less than the number of slots in the
    // uncoalesced table and makeRequest is not called again.
    if (uncoalescedTable.packetAvailable() && !issueEvent.scheduled()) {
        schedule(issueEvent, curTick());
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
GPUCoalescer::recordMissLatency(CoalescedRequest* crequest,
                                MachineType mach,
                                Cycles initialRequestTime,
                                Cycles forwardRequestTime,
                                Cycles firstResponseTime,
                                bool success, bool isRegion)
{
}

} // namespace ruby
} // namespace gem5
