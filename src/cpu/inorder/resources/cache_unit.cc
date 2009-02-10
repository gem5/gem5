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
#include "arch/mips/locked_mem.hh"
#include "arch/utility.hh"
#include "cpu/inorder/resources/cache_unit.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/cpu.hh"
#include "mem/request.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

Tick
CacheUnit::CachePort::recvAtomic(PacketPtr pkt)
{
    panic("DefaultFetch doesn't expect recvAtomic callback!");
    return curTick;
}

void
CacheUnit::CachePort::recvFunctional(PacketPtr pkt)
{
    panic("DefaultFetch doesn't expect recvFunctional callback!");
}

void
CacheUnit::CachePort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("DefaultFetch doesn't expect recvStatusChange callback!");
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
      retryPkt(NULL), retrySlot(-1)
{
    //cacheData.resize(res_width);
    //slotStatus = new CachePortStatus[width];
    //fetchPC = new Addr[width];
    cachePort = new CachePort(this);

    cacheBlocked = false;
}

Port *
CacheUnit::getPort(const string &if_name, int idx)
{
    if (if_name == resName)
        return cachePort;
    else
        return NULL;
}

int
CacheUnit::getSlot(DynInstPtr inst)
{
    if (!inst->validMemAddr()) {
        panic("Mem. Addr. must be set before requesting cache access\n");
    }

    Addr req_addr = inst->getMemAddr();

    if (resName == "icache_port" ||
        find(addrList.begin(), addrList.end(), req_addr) == addrList.end()) {

        int new_slot = Resource::getSlot(inst);

        if (new_slot == -1)
            return -1;

        inst->memTime = curTick;
        addrList.push_back(req_addr);
        addrMap[req_addr] = inst->seqNum;
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Address %08p added to dependency list\n",
                inst->readTid(), inst->seqNum, req_addr);
        return new_slot;
    } else {
        DPRINTF(InOrderCachePort,
                "Denying request because there is an outstanding"
                " request to/for addr. %08p. by [sn:%i] @ tick %i\n",
                req_addr, addrMap[req_addr], inst->memTime);
        return -1;
    }
}

void
CacheUnit::freeSlot(int slot_num)
{
    vector<Addr>::iterator vect_it = find(addrList.begin(), addrList.end(),
            reqMap[slot_num]->inst->getMemAddr());
    assert(vect_it != addrList.end());

    DPRINTF(InOrderCachePort,
            "[tid:%i]: Address %08p removed from dependency list\n",
            reqMap[slot_num]->inst->readTid(), (*vect_it));

    addrList.erase(vect_it);

    Resource::freeSlot(slot_num);
}

ResReqPtr
CacheUnit::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    ScheduleEntry* sched_entry = inst->resSched.top();

    if (!inst->validMemAddr()) {
        panic("Mem. Addr. must be set before requesting cache access\n");
    }

    int req_size = 0;
    MemCmd::Command pkt_cmd;

    if (sched_entry->cmd == InitiateReadData) {
        pkt_cmd = MemCmd::ReadReq;
        req_size = inst->getMemAccSize();

        DPRINTF(InOrderCachePort,
                "[tid:%i]: %i byte Read request from [sn:%i] for addr %08p\n",
                inst->readTid(), req_size, inst->seqNum, inst->getMemAddr());
    } else if (sched_entry->cmd == InitiateWriteData) {
        pkt_cmd = MemCmd::WriteReq;
        req_size = inst->getMemAccSize();

        DPRINTF(InOrderCachePort,
                "[tid:%i]: %i byte Write request from [sn:%i] for addr %08p\n",
                inst->readTid(), req_size, inst->seqNum, inst->getMemAddr());
    } else if (sched_entry->cmd == InitiateFetch){
        pkt_cmd = MemCmd::ReadReq;
        req_size = sizeof(MachInst); //@TODO: mips16e

        DPRINTF(InOrderCachePort,
                "[tid:%i]: %i byte Fetch request from [sn:%i] for addr %08p\n",
                inst->readTid(), req_size, inst->seqNum, inst->getMemAddr());
    } else {
        panic("%i: Unexpected request type (%i) to %s", curTick,
              sched_entry->cmd, name());
    }

    return new CacheRequest(this, inst, stage_num, id, slot_num,
                            sched_entry->cmd, req_size, pkt_cmd,
                            0/*flags*/, this->cpu->readCpuId());
}

void
CacheUnit::requestAgain(DynInstPtr inst, bool &service_request)
{
    //service_request = false;

    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(findRequest(inst));
    assert(cache_req);

    // Check to see if this instruction is requesting the same command
    // or a different one
    if (cache_req->cmd != inst->resSched.top()->cmd) {
        // If different, then update command in the request
        cache_req->cmd = inst->resSched.top()->cmd;
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: the command for this instruction\n",
                inst->readTid(), inst->seqNum);

        service_request = true;
    } else {
        // If same command, just check to see if memory access was completed
        // but dont try to re-execute
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: requesting this resource again\n",
                inst->readTid(), inst->seqNum);

        service_request = true;
    }
}

void
CacheUnit::execute(int slot_num)
{
    if (cacheBlocked) {
        DPRINTF(InOrderCachePort, "Cache Blocked. Cannot Access\n");
        return;
    }

    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(reqMap[slot_num]);
    assert(cache_req);

    DynInstPtr inst = cache_req->inst;
    int tid;
    tid = inst->readTid();
    int seq_num;
    seq_num = inst->seqNum;
    //int stage_num = cache_req->getStageNum();

    cache_req->fault = NoFault;

    switch (cache_req->cmd)
    {
      case InitiateFetch:
        DPRINTF(InOrderCachePort,
                "[tid:%u]: Initiating fetch access to %s for addr. %08p\n",
                tid, name(), cache_req->inst->getMemAddr());

        DPRINTF(InOrderCachePort,
                "[tid:%u]: Fetching new cache block from addr: %08p\n",
                tid, cache_req->memReq->getVaddr());

        inst->setCurResSlot(slot_num);
        doDataAccess(inst);
        break;

      case CompleteFetch:
        if (cache_req->isMemAccComplete()) {
            DPRINTF(InOrderCachePort,
                    "[tid:%i]: Completing Fetch Access for [sn:%i]\n",
                    tid, inst->seqNum);

            MachInst mach_inst = cache_req->dataPkt->get<MachInst>();

            /**
             * @TODO: May Need This Function for Endianness-Compatibility
             *  mach_inst =
             *    gtoh(*reinterpret_cast<MachInst *>(&cacheData[tid][offset]));
             */

            DPRINTF(InOrderCachePort,
                    "[tid:%i]: Fetched instruction is %08p\n",
                    tid, mach_inst);

            // ExtMachInst ext_inst = makeExtMI(mach_inst, cpu->tcBase(tid));

            inst->setMachInst(mach_inst);
            inst->setASID(tid);
            inst->setThreadState(cpu->thread[tid]);

            DPRINTF(InOrderStage, "[tid:%i]: Instruction [sn:%i] is: %s\n",
                    tid, seq_num, inst->staticInst->disassemble(inst->PC));

            // Set Up More TraceData info
            if (inst->traceData) {
                inst->traceData->setStaticInst(inst->staticInst);
                inst->traceData->setPC(inst->readPC());
            }

            cache_req->done();
        } else {
            DPRINTF(InOrderCachePort,
                    "[tid:%i]: [sn:%i]: Unable to Complete Fetch Access\n",
                    tid, inst->seqNum);
            DPRINTF(InOrderStall,
                    "STALL: [tid:%i]: Fetch miss from %08p\n",
                    tid, cache_req->inst->readPC());
            cache_req->setCompleted(false);
        }
        break;

      case InitiateReadData:
      case InitiateWriteData:
        DPRINTF(InOrderCachePort,
                "[tid:%u]: Initiating data access to %s for addr. %08p\n",
                tid, name(), cache_req->inst->getMemAddr());

        inst->setCurResSlot(slot_num);
        //inst->memAccess();
        inst->initiateAcc();
        break;

      case CompleteReadData:
      case CompleteWriteData:
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Trying to Complete Data Access\n",
                tid, inst->seqNum);
        if (cache_req->isMemAccComplete()) {
            cache_req->done();
        } else {
            DPRINTF(InOrderStall, "STALL: [tid:%i]: Data miss from %08p\n",
                    tid, cache_req->inst->getMemAddr());
            cache_req->setCompleted(false);
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

Fault
CacheUnit::doDataAccess(DynInstPtr inst)
{
    Fault fault = NoFault;
    int tid = 0;

    tid = inst->readTid();

    CacheReqPtr cache_req
        = dynamic_cast<CacheReqPtr>(reqMap[inst->getCurResSlot()]);
    assert(cache_req);

    cache_req->dataPkt = new CacheReqPacket(cache_req, cache_req->pktCmd,
                                            Packet::Broadcast);

    if (cache_req->dataPkt->isRead()) {
        cache_req->dataPkt->dataStatic(cache_req->reqData);
    } else if (cache_req->dataPkt->isWrite()) {
        cache_req->dataPkt->dataStatic(&cache_req->inst->storeData);

    }

    cache_req->dataPkt->time = curTick;

    bool do_access = true;  // flag to suppress cache access

    Request *memReq = cache_req->dataPkt->req;

    if (cache_req->dataPkt->isWrite() && memReq->isLocked()) {
        assert(cache_req->inst->isStoreConditional());
        DPRINTF(InOrderCachePort, "Evaluating Store Conditional access\n");
        do_access = TheISA::handleLockedWrite(cpu, memReq);
    }

    DPRINTF(InOrderCachePort,
            "[tid:%i] [sn:%i] attempting to access cache\n",
            tid, inst->seqNum);

    //@TODO: If you want to ignore failed store conditional accesses, then
    //       enable this. However, this might skew memory stats because
    //       the failed store conditional access will get ignored.
    // - Remove optionality here ...
    if (1/*do_access*/) {
        if (!cachePort->sendTiming(cache_req->dataPkt)) {
            DPRINTF(InOrderCachePort,
                    "[tid:%i] [sn:%i] is waiting to retry request\n",
                    tid, inst->seqNum);

            retrySlot = cache_req->getSlot();
            retryReq = cache_req;
            retryPkt = cache_req->dataPkt;

            cacheStatus = cacheWaitRetry;

            //cacheBlocked = true;

            DPRINTF(InOrderStall, "STALL: \n");

            cache_req->setCompleted(false);
        } else {
            DPRINTF(InOrderCachePort,
                    "[tid:%i] [sn:%i] is now waiting for cache response\n",
                    tid, inst->seqNum);
            cache_req->setCompleted();
            cache_req->setMemAccPending();
            cacheStatus = cacheWaitResponse;
            cacheBlocked = false;
        }
    } else if (!do_access && memReq->isLocked()){
        // Store-Conditional instructions complete even if they "failed"
        assert(cache_req->inst->isStoreConditional());
        cache_req->setCompleted(true);

        DPRINTF(LLSC,
                "[tid:%i]: T%i Ignoring Failed Store Conditional Access\n",
                tid, tid);

        cache_req->dataPkt->req->setExtraData(0);

        processCacheCompletion(cache_req->dataPkt);

        // Automatically set these since we ignored the memory access
        //cache_req->setMemAccPending(false);
        //cache_req->setMemAccCompleted();
    } else {
        // Make cache request again since access due to
        // inability to access
        DPRINTF(InOrderStall, "STALL: \n");
        cache_req->setCompleted(false);
    }

    return fault;
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

        cache_pkt->cacheReq->done();
        return;
    }

    DPRINTF(InOrderCachePort,
            "[tid:%u]: [sn:%i]: Waking from cache access to addr. %08p\n",
            cache_pkt->cacheReq->getInst()->readTid(),
            cache_pkt->cacheReq->getInst()->seqNum,
            cache_pkt->cacheReq->getInst()->getMemAddr());

    // Cast to correct request type
    CacheRequest *cache_req = dynamic_cast<CacheReqPtr>(
        findRequest(cache_pkt->cacheReq->getInst()));
    assert(cache_req);

#if TRACING_ON
    // Get resource request info
    unsigned tid = 0;
#endif

    //tid = pkt->req->getThreadNum();
    unsigned stage_num = cache_req->getStageNum();
    DynInstPtr inst = cache_req->inst;

    if (!cache_req->isSquashed()) {
        if (inst->resSched.top()->cmd == CompleteFetch) {
            DPRINTF(InOrderCachePort,
                    "[tid:%u]: [sn:%i]: Processing fetch access\n",
                    tid, inst->seqNum);
        } else if (inst->staticInst && inst->isMemRef()) {
            DPRINTF(InOrderCachePort,
                    "[tid:%u]: [sn:%i]: Processing cache access\n",
                    tid, inst->seqNum);

            inst->completeAcc(pkt);

            if (inst->isLoad()) {
                assert(cache_pkt->isRead());

                if (cache_pkt->req->isLocked()) {
                    DPRINTF(InOrderCachePort,
                            "[tid:%u]: Handling Load-Linked for [sn:%u]\n",
                            tid, inst->seqNum);
                    TheISA::handleLockedRead(cpu, cache_pkt->req);
                }

                // @TODO: Hardcoded to for load instructions. Assumes that
                // the dest. idx 0 is always where the data is loaded to.
                DPRINTF(InOrderCachePort,
                        "[tid:%u]: [sn:%i]: Data loaded was: %08p\n",
                        tid, inst->seqNum, inst->readIntResult(0));
            } else if(inst->isStore()) {
                assert(cache_pkt->isWrite());

                DPRINTF(InOrderCachePort,
                        "[tid:%u]: [sn:%i]: Data stored was: %08p\n",
                        tid, inst->seqNum,
                        getMemData(cache_pkt));

            }
        }

        cache_req->setMemAccPending(false);
        cache_req->setMemAccCompleted();

        // Wake up the CPU (if it went to sleep and was waiting on this
        // completion event).
        cpu->wakeCPU();

        DPRINTF(Activity, "[tid:%u] Activating %s due to cache completion\n",
            tid, cpu->pipelineStage[stage_num]->name());

        cpu->switchToActive(stage_num);
    } else {
        DPRINTF(InOrderCachePort,
                "[tid:%u] Miss on block @ %08p completed, but squashed\n",
                tid, cache_req->inst->readPC());
        cache_req->setMemAccCompleted();
    }

    inst->unsetMemAddr();
}

void
CacheUnit::recvRetry()
{
    DPRINTF(InOrderCachePort, "Retrying Request for [tid:%i] [sn:%i]\n",
            retryReq->inst->readTid(), retryReq->inst->seqNum);

    assert(retryPkt != NULL);
    assert(cacheBlocked);
    assert(cacheStatus == cacheWaitRetry);

    if (cachePort->sendTiming(retryPkt)) {
        cacheStatus = cacheWaitResponse;
        retryPkt = NULL;
        cacheBlocked = false;
    } else {
        DPRINTF(InOrderCachePort,
                "Retry Request for [tid:%i] [sn:%i] failed\n",
                retryReq->inst->readTid(), retryReq->inst->seqNum);
    }
}

void
CacheUnit::squash(DynInstPtr inst, int stage_num,
                  InstSeqNum squash_seq_num, unsigned tid)
{
    vector<int> slot_remove_list;

    map<int, ResReqPtr>::iterator map_it = reqMap.begin();
    map<int, ResReqPtr>::iterator map_end = reqMap.end();

    while (map_it != map_end) {
        ResReqPtr req_ptr = (*map_it).second;

        if (req_ptr &&
            req_ptr->getInst()->readTid() == tid &&
            req_ptr->getInst()->seqNum > squash_seq_num) {

            DPRINTF(InOrderCachePort,
                    "[tid:%i] Squashing request from [sn:%i]\n",
                    req_ptr->getInst()->readTid(), req_ptr->getInst()->seqNum);

            req_ptr->setSquashed();

            req_ptr->getInst()->setSquashed();

            CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(req_ptr);
            assert(cache_req);

            if (!cache_req->isMemAccPending()) {
                // Mark request for later removal
                cpu->reqRemoveList.push(req_ptr);

                // Mark slot for removal from resource
                slot_remove_list.push_back(req_ptr->getSlot());
            }
        }

        map_it++;
    }

    // Now Delete Slot Entry from Req. Map
    for (int i = 0; i < slot_remove_list.size(); i++)
        freeSlot(slot_remove_list[i]);
}

uint64_t
CacheUnit::getMemData(Packet *packet)
{
    switch (packet->getSize())
    {
      case 8:
        return packet->get<uint8_t>();

      case 16:
        return packet->get<uint16_t>();

      case 32:
        return packet->get<uint32_t>();

      case 864:
        return packet->get<uint64_t>();

      default:
        panic("bad store data size = %d\n", packet->getSize());
    }
}

