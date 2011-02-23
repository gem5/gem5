/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#include <vector>
#include <list>

#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/utility.hh"
#include "arch/predecoder.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/resources/cache_unit.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/resource_pool.hh"
#include "mem/request.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

#if TRACING_ON
static std::string
printMemData(uint8_t *data, unsigned size)
{
    std::stringstream dataStr;
    for (unsigned pos = 0; pos < size; pos++) {
        ccprintf(dataStr, "%02x", data[pos]);
    }
    return dataStr.str();
}
#endif

Tick
CacheUnit::CachePort::recvAtomic(PacketPtr pkt)
{
    panic("CacheUnit::CachePort doesn't expect recvAtomic callback!");
    return curTick();
}

void
CacheUnit::CachePort::recvFunctional(PacketPtr pkt)
{
    panic("CacheUnit::CachePort doesn't expect recvFunctional callback!");
}

void
CacheUnit::CachePort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("CacheUnit::CachePort doesn't expect recvStatusChange callback!");
}

bool
CacheUnit::CachePort::recvTiming(Packet *pkt)
{
    cachePortUnit->processCacheCompletion(pkt);
    return true;
}

void
CacheUnit::CachePort::recvRetry()
{
    cachePortUnit->recvRetry();
}

CacheUnit::CacheUnit(string res_name, int res_id, int res_width,
        int res_latency, InOrderCPU *_cpu, ThePipeline::Params *params)
    : Resource(res_name, res_id, res_width, res_latency, _cpu),
      cachePortBlocked(false)
{
    cachePort = new CachePort(this);

    // Hard-Code Selection For Now
    if (res_name == "icache_port")
        _tlb = params->itb;
    else if (res_name == "dcache_port")
        _tlb = params->dtb;
    else
        fatal("Unrecognized TLB name passed by user");

    for (int i=0; i < MaxThreads; i++) {
        tlbBlocked[i] = false;
    }
}

TheISA::TLB*
CacheUnit::tlb()
{
    return _tlb;

}

Port *
CacheUnit::getPort(const string &if_name, int idx)
{
    if (if_name == resName)
        return cachePort;
    else
        return NULL;
}

void
CacheUnit::init()
{
    for (int i = 0; i < width; i++) {
        reqs[i] = new CacheRequest(this);
    }

    // Currently Used to Model TLB Latency. Eventually
    // Switch to Timing TLB translations.
    resourceEvent = new CacheUnitEvent[width];

    cacheBlkSize = this->cachePort->peerBlockSize();
    cacheBlkMask = cacheBlkSize  - 1;

    initSlots();
}

int
CacheUnit::getSlot(DynInstPtr inst)
{
    ThreadID tid = inst->readTid();
    
    if (tlbBlocked[inst->threadNumber]) {
        return -1;
    }

    // For a Split-Load, the instruction would have processed once already
    // causing the address to be unset.
    if (!inst->validMemAddr() && !inst->splitInst) {
        panic("[tid:%i][sn:%i] Mem. Addr. must be set before requesting "
              "cache access\n", inst->readTid(), inst->seqNum);
    }

    Addr req_addr = inst->getMemAddr();

    if (resName == "icache_port" ||
        find(addrList[tid].begin(), addrList[tid].end(), req_addr) == 
        addrList[tid].end()) {

        int new_slot = Resource::getSlot(inst);

        if (new_slot == -1)
            return -1;

        inst->memTime = curTick();
        setAddrDependency(inst);            
        return new_slot;
    } else {
        // Allow same instruction multiple accesses to same address
        // should only happen maybe after a squashed inst. needs to replay
        if (addrMap[tid][req_addr] == inst->seqNum) {
            int new_slot = Resource::getSlot(inst);
        
            if (new_slot == -1)
                return -1;     

            return new_slot;       
        } else {                    
            DPRINTF(InOrderCachePort,
                "[tid:%i] Denying request because there is an outstanding"
                " request to/for addr. %08p. by [sn:%i] @ tick %i\n",
                inst->readTid(), req_addr, addrMap[tid][req_addr], inst->memTime);
            return -1;
        }        
    }

    return -1;   
}

void
CacheUnit::setAddrDependency(DynInstPtr inst)
{
    Addr req_addr = inst->getMemAddr();
    ThreadID tid = inst->readTid();

    addrList[tid].push_back(req_addr);
    addrMap[tid][req_addr] = inst->seqNum;

    DPRINTF(AddrDep,
            "[tid:%i]: [sn:%i]: Address %08p added to dependency list (size=%i)\n",
            inst->readTid(), inst->seqNum, req_addr, addrList[tid].size());

    //@NOTE: 10 is an arbitrarily "high" number, but to be exact
    //       we would need to know the # of outstanding accesses
    //       a priori. Information like fetch width, stage width,
    //       fetch buffer, and the branch resolution stage would be
    //       useful for the icache_port. For the dcache port, the #
    //       of outstanding cache accesses (mshrs) would be a good
    //       sanity check here.
    //assert(addrList[tid].size() < 10);
}

void
CacheUnit::removeAddrDependency(DynInstPtr inst)
{
    ThreadID tid = inst->readTid();

    Addr mem_addr = inst->getMemAddr();
    
    inst->unsetMemAddr();

    // Erase from Address List
    vector<Addr>::iterator vect_it = find(addrList[tid].begin(),
                                          addrList[tid].end(),
                                          mem_addr);
    assert(vect_it != addrList[tid].end() || inst->splitInst);

    if (vect_it != addrList[tid].end()) {
        DPRINTF(AddrDep,
                "[tid:%i]: [sn:%i] Address %08p removed from dependency "
                "list\n", inst->readTid(), inst->seqNum, (*vect_it));

        addrList[tid].erase(vect_it);

        // Erase From Address Map (Used for Debugging)
        addrMap[tid].erase(addrMap[tid].find(mem_addr));
    }
    

}

ResReqPtr
CacheUnit::findRequest(DynInstPtr inst)
{
    for (int i = 0; i < width; i++) {
        CacheRequest* cache_req =
            dynamic_cast<CacheRequest*>(reqs[i]);
        assert(cache_req);

        if (cache_req->valid &&
            cache_req->getInst() == inst &&
            cache_req->instIdx == inst->curSkedEntry->idx) {
            return cache_req;
        }
    }

    return NULL;
}

ResReqPtr
CacheUnit::findRequest(DynInstPtr inst, int idx)
{
    for (int i = 0; i < width; i++) {
        CacheRequest* cache_req =
            dynamic_cast<CacheRequest*>(reqs[i]);
        assert(cache_req);

        if (cache_req->valid &&
            cache_req->getInst() == inst &&
            cache_req->instIdx == idx) {
            return cache_req;
        }
    }

    return NULL;
}


ResReqPtr
CacheUnit::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    ScheduleEntry* sched_entry = *inst->curSkedEntry;
    CacheRequest* cache_req = dynamic_cast<CacheRequest*>(reqs[slot_num]);

    if (!inst->validMemAddr()) {
        panic("Mem. Addr. must be set before requesting cache access\n");
    }

    MemCmd::Command pkt_cmd;

    switch (sched_entry->cmd)
    {
      case InitSecondSplitRead:
        pkt_cmd = MemCmd::ReadReq;

        DPRINTF(InOrderCachePort,
                "[tid:%i]: Read request from [sn:%i] for addr %08p\n",
                inst->readTid(), inst->seqNum, inst->split2ndAddr);
        break;

      case InitiateReadData:
        pkt_cmd = MemCmd::ReadReq;

        DPRINTF(InOrderCachePort,
                "[tid:%i]: Read request from [sn:%i] for addr %08p\n",
                inst->readTid(), inst->seqNum, inst->getMemAddr());
        break;

      case InitSecondSplitWrite:
        pkt_cmd = MemCmd::WriteReq;

        DPRINTF(InOrderCachePort,
                "[tid:%i]: Write request from [sn:%i] for addr %08p\n",
                inst->readTid(), inst->seqNum, inst->split2ndAddr);
        break;

      case InitiateWriteData:
        pkt_cmd = MemCmd::WriteReq;

        DPRINTF(InOrderCachePort,
                "[tid:%i]: Write request from [sn:%i] for addr %08p\n",
                inst->readTid(), inst->seqNum, inst->getMemAddr());
        break;

      default:
        panic("%i: Unexpected request type (%i) to %s", curTick(),
              sched_entry->cmd, name());
    }

    cache_req->setRequest(inst, stage_num, id, slot_num,
                          sched_entry->cmd, pkt_cmd,
                          inst->curSkedEntry->idx);
    return cache_req;
}

void
CacheUnit::requestAgain(DynInstPtr inst, bool &service_request)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(findRequest(inst));
    assert(cache_req);

    // Check to see if this instruction is requesting the same command
    // or a different one
    if (cache_req->cmd != inst->curSkedEntry->cmd &&
        cache_req->instIdx == inst->curSkedEntry->idx) {
        // If different, then update command in the request
        cache_req->cmd = inst->curSkedEntry->cmd;
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Updating the command for this "
                "instruction\n ", inst->readTid(), inst->seqNum);

        service_request = true;
    } else if (inst->curSkedEntry->idx != CacheUnit::InitSecondSplitRead &&
               inst->curSkedEntry->idx != CacheUnit::InitSecondSplitWrite) {
        // If same command, just check to see if memory access was completed
        // but dont try to re-execute
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: requesting this resource again\n",
                inst->readTid(), inst->seqNum);

        service_request = true;
    }
}

void
CacheUnit::setupMemRequest(DynInstPtr inst, CacheReqPtr cache_req,
                           int acc_size, int flags)
{
    ThreadID tid = inst->readTid();
    Addr aligned_addr = inst->getMemAddr();

    if (!cache_req->is2ndSplit()) {
        if (inst->dataMemReq == NULL) {
            inst->dataMemReq =
                new Request(cpu->asid[tid], aligned_addr, acc_size, flags,
                            inst->instAddr(), cpu->readCpuId(),
                            tid);
            cache_req->memReq = inst->dataMemReq;
        }
    } else {
        assert(inst->splitInst);

        if (inst->splitMemReq ==  NULL) {
            inst->splitMemReq = new Request(cpu->asid[tid], 
                                            inst->split2ndAddr,
                                            acc_size, 
                                            flags, 
                                            inst->instAddr(),
                                            cpu->readCpuId(), 
                                            tid);
        }

        cache_req->memReq = inst->splitMemReq;
    }
}

void
CacheUnit::doTLBAccess(DynInstPtr inst, CacheReqPtr cache_req, int acc_size,
                       int flags, TheISA::TLB::Mode tlb_mode)
{
    ThreadID tid = inst->readTid();
    //Addr aligned_addr = inst->getMemAddr();
    unsigned stage_num = cache_req->getStageNum();
    unsigned slot_idx = cache_req->getSlot();

    setupMemRequest(inst, cache_req, acc_size, flags);

    inst->fault =
        _tlb->translateAtomic(cache_req->memReq,
                              cpu->thread[tid]->getTC(), tlb_mode);

    if (inst->fault != NoFault) {
        DPRINTF(InOrderTLB, "[tid:%i]: %s encountered while translating "
                "addr:%08p for [sn:%i].\n", tid, inst->fault->name(),
                cache_req->memReq->getVaddr(), inst->seqNum);

        cpu->pipelineStage[stage_num]->setResStall(cache_req, tid);

        tlbBlocked[tid] = true;

        cache_req->tlbStall = true;

        scheduleEvent(slot_idx, 1);

        cpu->trap(inst->fault, tid, inst);
    } else {
        DPRINTF(InOrderTLB, "[tid:%i]: [sn:%i] virt. addr %08p translated "
                "to phys. addr:%08p.\n", tid, inst->seqNum,
                cache_req->memReq->getVaddr(),
                cache_req->memReq->getPaddr());
    }

}

Fault
CacheUnit::read(DynInstPtr inst, Addr addr,
                uint8_t *data, unsigned size, unsigned flags)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(findRequest(inst));
    assert(cache_req && "Can't Find Instruction for Read!");

    // The block size of our peer
    unsigned blockSize = this->cachePort->peerBlockSize();

    //The size of the data we're trying to read.
    int fullSize = size;
    inst->totalSize = size;

    if (inst->traceData) {
        inst->traceData->setAddr(addr);
    }

    if (inst->split2ndAccess) {     
        size = inst->split2ndSize;
        cache_req->splitAccess = true;        
        cache_req->split2ndAccess = true;
        
        DPRINTF(InOrderCachePort, "[sn:%i] Split Read Access (2 of 2) for "
                "(%#x, %#x).\n", inst->seqNum, inst->getMemAddr(),
                inst->split2ndAddr);
    }  
    

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    
    if (secondAddr > addr && !inst->split2ndAccess) {
        DPRINTF(InOrderCachePort, "%i: sn[%i] Split Read Access (1 of 2) for "
                "(%#x, %#x).\n", curTick(), inst->seqNum, addr, secondAddr);
        
        // Save All "Total" Split Information
        // ==============================
        inst->splitInst = true;        
        inst->splitMemData = new uint8_t[size];
        
        if (!inst->splitInstSked) {
            assert(0 && "Split Requests Not Supported for Now...");

            // Schedule Split Read/Complete for Instruction
            // ==============================
            int stage_num = cache_req->getStageNum();
            RSkedPtr inst_sked = (stage_num >= ThePipeline::BackEndStartStage) ?
                inst->backSked : inst->frontSked;

            // this is just an arbitrarily high priority to ensure that this
            // gets pushed to the back of the list
            int stage_pri = 20;
        
            int isplit_cmd = CacheUnit::InitSecondSplitRead;
            inst_sked->push(new
                                ScheduleEntry(stage_num,
                                              stage_pri,
                                              cpu->resPool->getResIdx(DCache),
                                              isplit_cmd,
                                              1));

            int csplit_cmd = CacheUnit::CompleteSecondSplitRead;
            inst_sked->push(new
                                ScheduleEntry(stage_num + 1,
                                              1/*stage_pri*/,
                                              cpu->resPool->getResIdx(DCache),
                                              csplit_cmd,
                                              1));
            inst->splitInstSked = true;
        } else {
            DPRINTF(InOrderCachePort, "[tid:%i] [sn:%i] Retrying Split Read "
                    "Access (1 of 2) for (%#x, %#x).\n", inst->readTid(),
                    inst->seqNum, addr, secondAddr);
        }

        // Split Information for First Access
        // ==============================
        size = secondAddr - addr;
        cache_req->splitAccess = true;

        // Split Information for Second Access
        // ==============================
        inst->split2ndSize = addr + fullSize - secondAddr;
        inst->split2ndAddr = secondAddr;            
        inst->split2ndDataPtr = inst->splitMemData + size;
        inst->split2ndFlags = flags;        
    }
    
    doTLBAccess(inst, cache_req, size, flags, TheISA::TLB::Read);

    if (inst->fault == NoFault) {
        if (!cache_req->splitAccess) {            
            cache_req->reqData = new uint8_t[size];
            doCacheAccess(inst, NULL);
        } else {
            if (!inst->split2ndAccess) {                
                cache_req->reqData = inst->splitMemData;
            } else {
                cache_req->reqData = inst->split2ndDataPtr;                
            }
            
            doCacheAccess(inst, NULL, cache_req);            
        }        
    }

    return inst->fault;
}

Fault
CacheUnit::write(DynInstPtr inst, uint8_t *data, unsigned size,
                 Addr addr, unsigned flags, uint64_t *write_res)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(findRequest(inst));
    assert(cache_req && "Can't Find Instruction for Write!");

    // The block size of our peer
    unsigned blockSize = this->cachePort->peerBlockSize();

    //The size of the data we're trying to write.
    int fullSize = size;
    inst->totalSize = size;

    if (inst->traceData) {
        inst->traceData->setAddr(addr);
    }

    if (inst->split2ndAccess) {     
        size = inst->split2ndSize;
        cache_req->splitAccess = true;        
        cache_req->split2ndAccess = true;
        
        DPRINTF(InOrderCachePort, "[sn:%i] Split Write Access (2 of 2) for "
                "(%#x, %#x).\n", inst->seqNum, inst->getMemAddr(),
                inst->split2ndAddr);
    }  

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if (secondAddr > addr && !inst->split2ndAccess) {
            
        DPRINTF(InOrderCachePort, "[sn:%i] Split Write Access (1 of 2) for "
                "(%#x, %#x).\n", inst->seqNum, addr, secondAddr);

        // Save All "Total" Split Information
        // ==============================
        inst->splitInst = true;        

        if (!inst->splitInstSked) {
            assert(0 && "Split Requests Not Supported for Now...");

            // Schedule Split Read/Complete for Instruction
            // ==============================
            int stage_num = cache_req->getStageNum();
            RSkedPtr inst_sked = (stage_num >= ThePipeline::BackEndStartStage) ?
                inst->backSked : inst->frontSked;
        
            // this is just an arbitrarily high priority to ensure that this
            // gets pushed to the back of the list
            int stage_pri = 20;
        
            int isplit_cmd = CacheUnit::InitSecondSplitWrite;
            inst_sked->push(new
                            ScheduleEntry(stage_num,
                                          stage_pri,
                                          cpu->resPool->getResIdx(DCache),
                                          isplit_cmd,
                                          1));

            int csplit_cmd = CacheUnit::CompleteSecondSplitWrite;
            inst_sked->push(new
                            ScheduleEntry(stage_num + 1,
                                          1/*stage_pri*/,
                                          cpu->resPool->getResIdx(DCache),
                                          csplit_cmd,
                                          1));
            inst->splitInstSked = true;
        } else {
            DPRINTF(InOrderCachePort, "[tid:%i] sn:%i] Retrying Split Read "
                    "Access (1 of 2) for (%#x, %#x).\n",
                    inst->readTid(), inst->seqNum, addr, secondAddr);                   
        }
        
        

        // Split Information for First Access
        // ==============================
        size = secondAddr - addr;
        cache_req->splitAccess = true;

        // Split Information for Second Access
        // ==============================
        inst->split2ndSize = addr + fullSize - secondAddr;
        inst->split2ndAddr = secondAddr;            
        inst->split2ndStoreDataPtr = &cache_req->inst->storeData;
        inst->split2ndStoreDataPtr += size;
        inst->split2ndFlags = flags;        
        inst->splitInstSked = true;
    }    
        
    doTLBAccess(inst, cache_req, size, flags, TheISA::TLB::Write);

    if (inst->fault == NoFault) {
        if (!cache_req->splitAccess) {            
            doCacheAccess(inst, write_res);
        } else {            
            doCacheAccess(inst, write_res, cache_req);            
        }        
        
    }
    
    return inst->fault;
}


void
CacheUnit::execute(int slot_num)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(reqs[slot_num]);
    assert(cache_req);

    if (cachePortBlocked &&
        (cache_req->cmd == InitiateReadData ||
         cache_req->cmd == InitiateWriteData ||
         cache_req->cmd == InitSecondSplitRead ||
         cache_req->cmd == InitSecondSplitWrite)) {
        DPRINTF(InOrderCachePort, "Cache Port Blocked. Cannot Access\n");
        cache_req->done(false);
        return;
    }

    DynInstPtr inst = cache_req->inst;
#if TRACING_ON
    ThreadID tid = inst->readTid();
    std::string acc_type = "write";
#endif

    inst->fault = NoFault;

    switch (cache_req->cmd)
    {

      case InitiateReadData:
#if TRACING_ON
        acc_type = "read";
#endif        
      case InitiateWriteData:
        if (cachePortBlocked) {
            DPRINTF(InOrderCachePort, "Cache Port Blocked. Cannot Access\n");
            cache_req->done(false);
            return;
        }

        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i] Initiating data %s access to %s for "
                "addr. %08p\n", tid, inst->seqNum, acc_type, name(),
                cache_req->inst->getMemAddr());

        inst->setCurResSlot(slot_num);

        if (inst->isDataPrefetch() || inst->isInstPrefetch()) {
            inst->execute();
        } else {
            inst->initiateAcc();
        }
        
        break;

      case InitSecondSplitRead:
        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i] Initiating split data read access to %s "
                "for addr. %08p\n", tid, inst->seqNum, name(),
                cache_req->inst->split2ndAddr);
        inst->split2ndAccess = true;
        assert(inst->split2ndAddr != 0);
        read(inst, inst->split2ndAddr, &inst->split2ndData,
             inst->totalSize, inst->split2ndFlags);
        break;

      case InitSecondSplitWrite:
        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i] Initiating split data write access to %s "
                "for addr. %08p\n", tid, inst->seqNum, name(),
                cache_req->inst->getMemAddr());

        inst->split2ndAccess = true;
        assert(inst->split2ndAddr != 0);
        write(inst, &inst->split2ndData, inst->totalSize,
              inst->split2ndAddr, inst->split2ndFlags, NULL);
        break;

      case CompleteReadData:
      case CompleteWriteData:
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Trying to Complete Data Access\n",
                tid, inst->seqNum);

        if (cache_req->isMemAccComplete() ||
            inst->isDataPrefetch() ||
            inst->isInstPrefetch()) {
            removeAddrDependency(inst);
            cache_req->setMemStall(false);            
            cache_req->done();
        } else {
            DPRINTF(InOrderStall, "STALL: [tid:%i]: Data miss from %08p\n",
                    tid, cache_req->inst->getMemAddr());
            cache_req->setCompleted(false);
            cache_req->setMemStall(true);            
        }
        break;

      case CompleteSecondSplitRead:
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Trying to Complete Split Data Read "
                "Access\n", tid, inst->seqNum);

        if (cache_req->isMemAccComplete() ||
            inst->isDataPrefetch() ||
            inst->isInstPrefetch()) {
            removeAddrDependency(inst);
            cache_req->setMemStall(false);            
            cache_req->done();
        } else {
            DPRINTF(InOrderStall, "STALL: [tid:%i]: Data miss from %08p\n",
                    tid, cache_req->inst->split2ndAddr);
            cache_req->setCompleted(false);
            cache_req->setMemStall(true);            
        }
        break;

      case CompleteSecondSplitWrite:
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Trying to Complete Split Data Write "
                "Access\n", tid, inst->seqNum);

        if (cache_req->isMemAccComplete() ||
            inst->isDataPrefetch() ||
            inst->isInstPrefetch()) {
            removeAddrDependency(inst);
            cache_req->setMemStall(false);            
            cache_req->done();
        } else {
            DPRINTF(InOrderStall, "STALL: [tid:%i]: Data miss from %08p\n",
                    tid, cache_req->inst->split2ndAddr);
            cache_req->setCompleted(false);
            cache_req->setMemStall(true);            
        }
        break;
        
      default:
        fatal("Unrecognized command to %s", resName);
    }
}

// @TODO: Split into doCacheRead() and doCacheWrite()
void
CacheUnit::doCacheAccess(DynInstPtr inst, uint64_t *write_res,
                         CacheReqPtr split_req)
{
    Fault fault = NoFault;
#if TRACING_ON
    ThreadID tid = inst->readTid();
#endif

    CacheReqPtr cache_req;
    
    if (split_req == NULL) {        
        cache_req = dynamic_cast<CacheReqPtr>(reqs[inst->getCurResSlot()]);
    } else{
        cache_req = split_req;
    }        

    assert(cache_req);

    // Check for LL/SC and if so change command
    if (cache_req->memReq->isLLSC() && cache_req->pktCmd == MemCmd::ReadReq) {
        cache_req->pktCmd = MemCmd::LoadLockedReq;
    }

    if (cache_req->pktCmd == MemCmd::WriteReq) {
        cache_req->pktCmd =
            cache_req->memReq->isSwap() ? MemCmd::SwapReq :
            (cache_req->memReq->isLLSC() ? MemCmd::StoreCondReq 
             : MemCmd::WriteReq);
    }

    cache_req->dataPkt = new CacheReqPacket(cache_req,
                                            cache_req->pktCmd,
                                            Packet::Broadcast,
                                            cache_req->instIdx);

    if (cache_req->dataPkt->isRead()) {
        cache_req->dataPkt->dataStatic(cache_req->reqData);
    } else if (cache_req->dataPkt->isWrite()) {        
        if (inst->split2ndAccess) {            
            cache_req->dataPkt->dataStatic(inst->split2ndStoreDataPtr);
        } else {
            cache_req->dataPkt->dataStatic(&cache_req->inst->storeData);            
        }
        
        if (cache_req->memReq->isCondSwap()) {
            assert(write_res);
            cache_req->memReq->setExtraData(*write_res);
        }
    }

    bool do_access = true;  // flag to suppress cache access

    Request *memReq = cache_req->dataPkt->req;

    if (cache_req->dataPkt->isWrite() && cache_req->memReq->isLLSC()) {
        assert(cache_req->inst->isStoreConditional());
        DPRINTF(InOrderCachePort, "Evaluating Store Conditional access\n");
        do_access = TheISA::handleLockedWrite(cpu, memReq);
    }

    DPRINTF(InOrderCachePort,
            "[tid:%i] [sn:%i] attempting to access cache for addr %08p\n",
            tid, inst->seqNum, cache_req->dataPkt->getAddr());

    if (do_access) {
        if (!cachePort->sendTiming(cache_req->dataPkt)) {
            DPRINTF(InOrderCachePort,
                    "[tid:%i] [sn:%i] cannot access cache, because port "
                    "is blocked. now waiting to retry request\n", tid, 
                    inst->seqNum);
            delete cache_req->dataPkt;
            cache_req->done(false);
            cachePortBlocked = true;
        } else {
            DPRINTF(InOrderCachePort,
                    "[tid:%i] [sn:%i] is now waiting for cache response\n",
                    tid, inst->seqNum);
            cache_req->setCompleted();
            cache_req->setMemAccPending();
            cachePortBlocked = false;
        }
    } else if (!do_access && memReq->isLLSC()){
        // Store-Conditional instructions complete even if they "failed"
        assert(cache_req->inst->isStoreConditional());
        cache_req->setCompleted(true);

        DPRINTF(LLSC,
                "[tid:%i]: T%i Ignoring Failed Store Conditional Access\n",
                tid, tid);

        processCacheCompletion(cache_req->dataPkt);
    } else {
        delete cache_req->dataPkt;
        // Make cache request again since access due to
        // inability to access
        DPRINTF(InOrderStall, "STALL: \n");
        cache_req->done(false);
    }

}

void
CacheUnit::processCacheCompletion(PacketPtr pkt)
{
    // Cast to correct packet type
    CacheReqPacket* cache_pkt = dynamic_cast<CacheReqPacket*>(pkt);
             
    assert(cache_pkt);

    if (cache_pkt->cacheReq->isSquashed()) {
        DPRINTF(InOrderCachePort,
                "Ignoring completion of squashed access, [tid:%i] [sn:%i]\n",
                cache_pkt->cacheReq->getInst()->readTid(),
                cache_pkt->cacheReq->getInst()->seqNum);
        DPRINTF(RefCount,
                "Ignoring completion of squashed access, [tid:%i] [sn:%i]\n",
                cache_pkt->cacheReq->getTid(),
                cache_pkt->cacheReq->seqNum);

        cache_pkt->cacheReq->freeSlot();
        delete cache_pkt;

        cpu->wakeCPU();

        return;
    }

    DPRINTF(InOrderCachePort,
            "[tid:%u]: [sn:%i]: Waking from cache access to addr. %08p\n",
            cache_pkt->cacheReq->getInst()->readTid(),
            cache_pkt->cacheReq->getInst()->seqNum,
            cache_pkt->cacheReq->getInst()->getMemAddr());

    // Cast to correct request type
    CacheRequest *cache_req = dynamic_cast<CacheReqPtr>(
        findRequest(cache_pkt->cacheReq->getInst(), cache_pkt->instIdx));

    if (!cache_req) {
        panic("[tid:%u]: [sn:%i]: Can't find slot for cache access to "
              "addr. %08p\n", cache_pkt->cacheReq->getInst()->readTid(),
              cache_pkt->cacheReq->getInst()->seqNum,
              cache_pkt->cacheReq->getInst()->getMemAddr());
    }
    
    assert(cache_req);


    // Get resource request info
    unsigned stage_num = cache_req->getStageNum();
    DynInstPtr inst = cache_req->inst;
    ThreadID tid = cache_req->inst->readTid();

    assert(!cache_req->isSquashed());
    assert(inst->staticInst && inst->isMemRef());


    DPRINTF(InOrderCachePort,
            "[tid:%u]: [sn:%i]: Processing cache access\n",
            tid, inst->seqNum);
    PacketPtr dataPkt = NULL;

    if (inst->splitInst) {
        inst->splitFinishCnt++;

        if (inst->splitFinishCnt == 2) {
            cache_req->memReq->setVirt(0/*inst->tid*/,
                                       inst->getMemAddr(),
                                       inst->totalSize,
                                       0,
                                       0);

            Packet split_pkt(cache_req->memReq, cache_req->pktCmd,
                             Packet::Broadcast);


            if (inst->isLoad()) {
                split_pkt.dataStatic(inst->splitMemData);
            } else  {
                split_pkt.dataStatic(&inst->storeData);
            }

            dataPkt = &split_pkt;
        }
    } else {
        dataPkt = pkt;
    }
    inst->completeAcc(dataPkt);

    if (inst->isLoad()) {
        assert(cache_pkt->isRead());

        if (cache_pkt->req->isLLSC()) {
            DPRINTF(InOrderCachePort,
                    "[tid:%u]: Handling Load-Linked for [sn:%u]\n",
                    tid, inst->seqNum);
            TheISA::handleLockedRead(cpu, cache_pkt->req);
        }

        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i]: Bytes loaded were: %s\n",
                tid, inst->seqNum,
                printMemData(dataPkt->getPtr<uint8_t>(),
                             dataPkt->getSize()));
    } else if(inst->isStore()) {
        assert(cache_pkt->isWrite());

        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i]: Bytes stored were: %s\n",
                tid, inst->seqNum,
                printMemData(dataPkt->getPtr<uint8_t>(),
                             dataPkt->getSize()));
    }

    delete cache_pkt;
    cache_req->setMemAccPending(false);
    cache_req->setMemAccCompleted();

    if (cache_req->isMemStall() &&
        cpu->threadModel == InOrderCPU::SwitchOnCacheMiss) {
        DPRINTF(InOrderCachePort, "[tid:%u] Waking up from Cache Miss.\n",
                tid);
            
        cpu->activateContext(tid);
            
        DPRINTF(ThreadModel, "Activating [tid:%i] after return from cache"
                "miss.\n", tid);
    }
        
    // Wake up the CPU (if it went to sleep and was waiting on this
    // completion event).
    cpu->wakeCPU();

    DPRINTF(Activity, "[tid:%u] Activating %s due to cache completion\n",
            tid, cpu->pipelineStage[stage_num]->name());

    cpu->switchToActive(stage_num);
}

void
CacheUnit::recvRetry()
{
    DPRINTF(InOrderCachePort, "Unblocking Cache Port. \n");
    
    assert(cachePortBlocked);

    // Clear the cache port for use again
    cachePortBlocked = false;

    cpu->wakeCPU();
}

CacheUnitEvent::CacheUnitEvent()
    : ResourceEvent()
{ }

void
CacheUnitEvent::process()
{
    DynInstPtr inst = resource->reqs[slotIdx]->inst;
    int stage_num = resource->reqs[slotIdx]->getStageNum();
    ThreadID tid = inst->threadNumber;
    CacheReqPtr req_ptr = dynamic_cast<CacheReqPtr>(resource->reqs[slotIdx]);

    DPRINTF(InOrderTLB, "Waking up from TLB Miss caused by [sn:%i].\n",
            inst->seqNum);

    CacheUnit* tlb_res = dynamic_cast<CacheUnit*>(resource);
    assert(tlb_res);

    tlb_res->tlbBlocked[tid] = false;

    tlb_res->cpu->pipelineStage[stage_num]->
        unsetResStall(tlb_res->reqs[slotIdx], tid);

    req_ptr->tlbStall = false;

    if (req_ptr->isSquashed()) {
        req_ptr->freeSlot();
    }

    tlb_res->cpu->wakeCPU();
}

void
CacheUnit::squashDueToMemStall(DynInstPtr inst, int stage_num,
                               InstSeqNum squash_seq_num, ThreadID tid)
{
    // If squashing due to memory stall, then we do NOT want to 
    // squash the instruction that caused the stall so we
    // increment the sequence number here to prevent that.
    //
    // NOTE: This is only for the SwitchOnCacheMiss Model
    // NOTE: If you have multiple outstanding misses from the same
    //       thread then you need to reevaluate this code
    // NOTE: squash should originate from 
    //       pipeline_stage.cc:processInstSchedule
    DPRINTF(InOrderCachePort, "Squashing above [sn:%u]\n", 
            squash_seq_num + 1);
    
    squash(inst, stage_num, squash_seq_num + 1, tid);    
}

void
CacheUnit::squashCacheRequest(CacheReqPtr req_ptr)
{
    DynInstPtr inst =  req_ptr->getInst();

    req_ptr->setSquashed();
    inst->setSquashed();
    if (inst->validMemAddr()) {
        DPRINTF(AddrDep, "Squash of [tid:%i] [sn:%i], attempting to "
                "remove addr. %08p dependencies.\n",
                inst->readTid(),
                inst->seqNum,
                inst->getMemAddr());

        removeAddrDependency(inst);
    }
}


void
CacheUnit::squash(DynInstPtr inst, int stage_num,
                  InstSeqNum squash_seq_num, ThreadID tid)
{
    for (int i = 0; i < width; i++) {
        ResReqPtr req_ptr = reqs[i];

        if (req_ptr->valid &&
            req_ptr->getInst()->readTid() == tid &&
            req_ptr->getInst()->seqNum > squash_seq_num) {

            DPRINTF(InOrderCachePort,
                    "[tid:%i] Squashing request from [sn:%i]\n",
                    req_ptr->getInst()->readTid(), req_ptr->getInst()->seqNum);

            if (req_ptr->isSquashed()) {
                DPRINTF(AddrDep, "Request for [tid:%i] [sn:%i] already "
                        "squashed, ignoring squash process.\n",
                        req_ptr->getInst()->readTid(),
                        req_ptr->getInst()->seqNum);
                continue;                
            }

            CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(req_ptr);
            assert(cache_req);

            squashCacheRequest(cache_req);

            int req_slot_num = req_ptr->getSlot();

            if (cache_req->tlbStall) {
                tlbBlocked[tid] = false;

                int stall_stage = reqs[req_slot_num]->getStageNum();

                cpu->pipelineStage[stall_stage]->
                    unsetResStall(reqs[req_slot_num], tid);
            }

            if (!cache_req->tlbStall && !cache_req->isMemAccPending()) {
                freeSlot(req_slot_num);
            } else {
                DPRINTF(InOrderCachePort,
                        "[tid:%i] Request from [sn:%i] squashed, but still "
                        "pending completion.\n",
                        req_ptr->getInst()->readTid(), req_ptr->getInst()->seqNum);
                DPRINTF(RefCount,
                        "[tid:%i] Request from [sn:%i] squashed (split:%i), but "
                        "still pending completion.\n",
                        req_ptr->getInst()->readTid(), req_ptr->getInst()->seqNum,
                        req_ptr->getInst()->splitInst);
            }
        }
    }

}

