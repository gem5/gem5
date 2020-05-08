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

bool
UncoalescedTable::packetAvailable()
{
    return !instMap.empty();
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
        if (iter->second.empty()) {
            instMap.erase(iter++);
            coalescer->getGMTokenPort().sendTokens(1);
        } else {
            ++iter;
        }
    }
}

void
UncoalescedTable::printRequestTable(std::stringstream& ss)
{
    ss << "UncoalescedTable contains " << instMap.size()
       << " address entries." << std::endl;
    for (auto& inst : instMap) {
        ss << "Addr 0x" << std::hex << inst.first << std::dec
           << " with " << inst.second.size() << " packets"
           << std::endl;
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

GPUCoalescer::GPUCoalescer(const Params *p)
    : RubyPort(p),
      issueEvent([this]{ completeIssue(); }, "Issue coalesced request",
                 false, Event::Progress_Event_Pri),
      uncoalescedTable(this),
      deadlockCheckEvent([this]{ wakeup(); }, "GPUCoalescer deadlock check"),
      gmTokenPort(name() + ".gmTokenPort", this)
{
    m_store_waiting_on_load_cycles = 0;
    m_store_waiting_on_store_cycles = 0;
    m_load_waiting_on_store_cycles = 0;
    m_load_waiting_on_load_cycles = 0;

    m_outstanding_count = 0;

    coalescingWindow = p->max_coalesces_per_cycle;

    m_max_outstanding_requests = 0;
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

    m_runningGarnetStandalone = p->garnet_standalone;
    assumingRfOCoherence = p->assume_rfo;
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
                ss << "Outstanding requests: " << m_outstanding_count
                   << std::endl;

                panic("Possible Deadlock detected. Aborting!\n"
                     "version: %d request.paddr: 0x%x coalescedTable: %d "
                     "current time: %u issue_time: %d difference: %d\n"
                     "Request Tables:\n %s", m_version,
                      req->getFirstPkt()->getAddr(),
                      coalescedTable.size(), cyclesToTicks(current_time),
                      cyclesToTicks(req->getIssueTime()),
                      cyclesToTicks(current_time - req->getIssueTime()),
                      ss.str());
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
    uncoalescedTable.printRequestTable(ss);

    ss << "CoalescedTable contains " << coalescedTable.size()
       << " address entries." << std::endl;
    for (auto& requestList : coalescedTable) {
        ss << "Addr 0x" << std::hex << requestList.first << std::dec
           << ": type-";
        for (auto& request : requestList.second) {
            ss << RubyRequestType_to_string(request->getRubyType())
               << " pkts-" << request->getPackets().size()
               << " issued-" << request->getIssueTime() << " seqNum-"
               << request->getSeqNum() << "; ";
        }
        ss << std::endl;
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
    Addr request_line_address = makeLineAddress(request_address);

    RubyRequestType type = crequest->getRubyType();

    DPRINTF(GPUCoalescer, "Got hitCallback for 0x%X\n", request_line_address);

    recordMissLatency(crequest, mach,
                      initialRequestTime,
                      forwardRequestTime,
                      firstResponseTime,
                      success, isRegion);
    // update the data
    //
    // MUST AD DOING THIS FOR EACH REQUEST IN COALESCER
    std::vector<PacketPtr> pktList = crequest->getPackets();
    DPRINTF(GPUCoalescer, "Responding to %d packets for addr 0x%X\n",
            pktList.size(), request_line_address);
    for (auto& pkt : pktList) {
        request_address = pkt->getAddr();
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
                safe_cast<RubyTester::SenderState*>
                    (requestSenderState->predecessor);
            testerSenderState->subBlock.mergeFrom(data);
        }
    }



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
    assert(!pkt->isFlush());

    if (pkt->req->isAtomicReturn()) {
        req_type = RubyRequestType_ATOMIC_RETURN;
    } else if (pkt->req->isAtomicNoReturn()) {
        req_type = RubyRequestType_ATOMIC_NO_RETURN;
    } else if (pkt->isRead()) {
        req_type = RubyRequestType_LD;
    } else if (pkt->isWrite()) {
        req_type = RubyRequestType_ST;
    } else {
        // Acquire and release packets will have been issued by
        // makeRequest, so we do not need to check for it here.
        panic("Unsupported ruby packet type\n");
    }

    return req_type;
}

// Places an uncoalesced packet in uncoalescedTable. If the packet is a
// special type (MemFence, scoping, etc), it is issued immediately.
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

    if (!pkt->isLLSC() && !pkt->req->isLockedRMW() && !pkt->isAtomicOp() &&
        !pkt->isRead() && !pkt->isWrite() && !pkt->isFlush() &&
        (pkt->req->isRelease() || pkt->req->isAcquire())) {
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
    }

    uncoalescedTable.insertPacket(pkt);
    DPRINTF(GPUCoalescer, "UC insertPacket 0x%X\n", pkt->getAddr());

    if (!issueEvent.scheduled())
        schedule(issueEvent, curTick());
    // TODO: issue hardware prefetches here
    return RequestStatus_Issued;
}

void
GPUCoalescer::issueRequest(CoalescedRequest* crequest)
{
    PacketPtr pkt = crequest->getFirstPkt();

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
    uint32_t tableSize = crequest->getPackets().size();
    for (int i = 0; i < tableSize; i++) {
        PacketPtr tmpPkt = crequest->getPackets()[i];
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
                              pkt->getSize(), pc, crequest->getRubyType(),
                              RubyAccessMode_Supervisor, pkt,
                              PrefetchBit_No, proc_id, 100,
                              blockSize, accessMask,
                              dataBlock, atomicOps,
                              accessScope, accessSegment);
    } else {
        msg = std::make_shared<RubyRequest>(clockEdge(), pkt->getAddr(),
                              pkt->getPtr<uint8_t>(),
                              pkt->getSize(), pc, crequest->getRubyType(),
                              RubyAccessMode_Supervisor, pkt,
                              PrefetchBit_No, proc_id, 100,
                              blockSize, accessMask,
                              dataBlock,
                              accessScope, accessSegment);
    }
    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %s\n",
             curTick(), m_version, "Coal", "Begin", "", "",
             printAddress(msg->getPhysicalAddress()),
             RubyRequestType_to_string(crequest->getRubyType()));

    fatal_if(crequest->getRubyType() == RubyRequestType_IFETCH,
             "there should not be any I-Fetch requests in the GPU Coalescer");

    Tick latency = cyclesToTicks(
                m_controller->mandatoryQueueLatency(crequest->getRubyType()));
    assert(latency > 0);

    if (!deadlockCheckEvent.scheduled()) {
        schedule(deadlockCheckEvent,
                 m_deadlock_threshold * clockPeriod() +
                 curTick());
    }

    assert(m_mandatory_q_ptr);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), latency);
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
        << "]";
}


void
GPUCoalescer::recordRequestType(SequencerRequestType requestType) {
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            SequencerRequestType_to_string(requestType));
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

            DPRINTF(GPUCoalescer, "Issued req type %s seqNum %d\n",
                    RubyRequestType_to_string(creq->getRubyType()), seqNum);
            issueRequest(creq);
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
        PerInstPackets *pktList =
            uncoalescedTable.getInstPackets(instIdx);

        // getInstPackets will return nullptr if no instruction
        // exists at the current offset.
        if (!pktList) {
            break;
        } else {
            // Since we have a pointer to the list of packets in the inst,
            // erase them from the list if coalescing is successful and
            // leave them in the list otherwise. This aggressively attempts
            // to coalesce as many packets as possible from the current inst.
            pktList->remove_if(
                [&](PacketPtr pkt) { return coalescePacket(pkt); }
            );
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
GPUCoalescer::completeHitCallback(std::vector<PacketPtr> & mylist)
{
    for (auto& pkt : mylist) {
        RubyPort::SenderState *ss =
            safe_cast<RubyPort::SenderState *>(pkt->senderState);
        MemSlavePort *port = ss->port;
        assert(port != NULL);

        pkt->senderState = ss->predecessor;
        delete ss;
        port->hitCallback(pkt);
        trySendRetries();
    }

    // We schedule an event in the same tick as hitCallback (similar to
    // makeRequest) rather than calling completeIssue directly to reduce
    // function calls to complete issue. This can only happen if the max
    // outstanding requests is less than the number of slots in the
    // uncoalesced table and makeRequest is not called again.
    if (uncoalescedTable.packetAvailable() && !issueEvent.scheduled()) {
        schedule(issueEvent, curTick());
    }

    testDrainComplete();
}

void
GPUCoalescer::recordMissLatency(CoalescedRequest* crequest,
                                MachineType mach,
                                Cycles initialRequestTime,
                                Cycles forwardRequestTime,
                                Cycles firstResponseTime,
                                bool success, bool isRegion)
{
    RubyRequestType type = crequest->getRubyType();
    Cycles issued_time = crequest->getIssueTime();
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
             printAddress(crequest->getFirstPkt()->getAddr()), total_lat);
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
