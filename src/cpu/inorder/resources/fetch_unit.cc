/*
 * Copyright (c) 2011 The Regents of The University of Michigan
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
#include "cpu/inorder/resources/fetch_unit.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/resource_pool.hh"
#include "mem/request.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

FetchUnit::FetchUnit(string res_name, int res_id, int res_width,
                     int res_latency, InOrderCPU *_cpu,
                     ThePipeline::Params *params)
    : CacheUnit(res_name, res_id, res_width, res_latency, _cpu,
                params)
{ }

int
FetchUnit::getSlot(DynInstPtr inst)
{
    if (tlbBlocked[inst->threadNumber]) {
        return -1;
    }

    if (!inst->validMemAddr()) {
        panic("[tid:%i][sn:%i] Mem. Addr. must be set before requesting "
              "cache access\n", inst->readTid(), inst->seqNum);
    }

    int new_slot = Resource::getSlot(inst);

    if (new_slot == -1)
        return -1;

    inst->memTime = curTick();
    return new_slot;
}

void
FetchUnit::removeAddrDependency(DynInstPtr inst)
{
    inst->unsetMemAddr();
}

ResReqPtr
FetchUnit::getRequest(DynInstPtr inst, int stage_num, int res_idx,
                     int slot_num, unsigned cmd)
{
    ScheduleEntry* sched_entry = inst->resSched.top();

    if (!inst->validMemAddr()) {
        panic("Mem. Addr. must be set before requesting cache access\n");
    }

    MemCmd::Command pkt_cmd;

    switch (sched_entry->cmd)
    {
      case InitiateFetch:
        pkt_cmd = MemCmd::ReadReq;

        DPRINTF(InOrderCachePort,
                "[tid:%i]: Fetch request from [sn:%i] for addr %08p\n",
                inst->readTid(), inst->seqNum, inst->getMemAddr());
        break;

      default:
        panic("%i: Unexpected request type (%i) to %s", curTick(),
              sched_entry->cmd, name());
    }

    return new CacheRequest(this, inst, stage_num, id, slot_num,
                            sched_entry->cmd, 0, pkt_cmd,
                            0/*flags*/, this->cpu->readCpuId(),
                            inst->resSched.top()->idx);
}

void
FetchUnit::setupMemRequest(DynInstPtr inst, CacheReqPtr cache_req,
                           int acc_size, int flags)
{
    ThreadID tid = inst->readTid();
    Addr aligned_addr = inst->getMemAddr();

    inst->fetchMemReq =
            new Request(inst->readTid(), aligned_addr, acc_size, flags,
                        inst->instAddr(), cpu->readCpuId(), inst->readTid());

    cache_req->memReq = inst->fetchMemReq;
}


void
FetchUnit::execute(int slot_num)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(reqMap[slot_num]);
    assert(cache_req);

    if (cachePortBlocked) {
        DPRINTF(InOrderCachePort, "Cache Port Blocked. Cannot Access\n");
        cache_req->setCompleted(false);
        return;
    }

    DynInstPtr inst = cache_req->inst;
#if TRACING_ON
    ThreadID tid = inst->readTid();
    int seq_num = inst->seqNum;
    std::string acc_type = "write";
#endif

    cache_req->fault = NoFault;

    switch (cache_req->cmd)
    {
      case InitiateFetch:
        {
            doTLBAccess(inst, cache_req, cacheBlkSize, 0, TheISA::TLB::Execute);

            if (cache_req->fault == NoFault) {
                DPRINTF(InOrderCachePort,
                    "[tid:%u]: Initiating fetch access to %s for addr. %08p\n",
                    tid, name(), cache_req->inst->getMemAddr());

                cache_req->reqData = new uint8_t[cacheBlksize];

                inst->setCurResSlot(slot_num);

                doCacheAccess(inst);
            }

            break;
        }

      case CompleteFetch:
        if (cache_req->isMemAccComplete()) {
            DPRINTF(InOrderCachePort,
                    "[tid:%i]: Completing Fetch Access for [sn:%i]\n",
                    tid, inst->seqNum);


            DPRINTF(InOrderCachePort, "[tid:%i]: Instruction [sn:%i] is: %s\n",
                    tid, seq_num,
                    inst->staticInst->disassemble(inst->instAddr()));

            removeAddrDependency(inst);

            delete cache_req->dataPkt;

            // Do not stall and switch threads for fetch... for now..
            // TODO: We need to detect cache misses for latencies > 1
            // cache_req->setMemStall(false);

            cache_req->done();
        } else {
            DPRINTF(InOrderCachePort,
                     "[tid:%i]: [sn:%i]: Unable to Complete Fetch Access\n",
                    tid, inst->seqNum);
            DPRINTF(InOrderStall,
                    "STALL: [tid:%i]: Fetch miss from %08p\n",
                    tid, cache_req->inst->instAddr());
            cache_req->setCompleted(false);
            //cache_req->setMemStall(true);
        }
        break;

      default:
        fatal("Unrecognized command to %s", resName);
    }
}

void
FetchUnit::processCacheCompletion(PacketPtr pkt)
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

        cache_pkt->cacheReq->done();
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

    if (!cache_req->isSquashed()) {
        assert(inst->resSched.top()->cmd == CompleteFetch);

        DPRINTF(InOrderCachePort,
                "[tid:%u]: [sn:%i]: Processing fetch access\n",
                tid, inst->seqNum);

        // NOTE: This is only allowing a thread to fetch one line
        //       at a time. Re-examine when/if prefetching
        //       gets implemented.
        // memcpy(fetchData[tid], cache_pkt->getPtr<uint8_t>(),
        //        cache_pkt->getSize());

        // Get the instruction from the array of the cache line.
        // @todo: update this
        ExtMachInst ext_inst;
        StaticInstPtr staticInst = NULL;
        TheISA::PCState instPC = inst->pcState();
        MachInst mach_inst =
            TheISA::gtoh(*reinterpret_cast<TheISA::MachInst *>
                         (cache_pkt->getPtr<uint8_t>()));

        predecoder.setTC(cpu->thread[tid]->getTC());
        predecoder.moreBytes(instPC, inst->instAddr(), mach_inst);
        ext_inst = predecoder.getExtMachInst(instPC);
        inst->pcState(instPC);

        inst->setMachInst(ext_inst);

        // Set Up More TraceData info
        if (inst->traceData) {
            inst->traceData->setStaticInst(inst->staticInst);
            inst->traceData->setPC(instPC);
        }

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
    } else {
        DPRINTF(InOrderCachePort,
                "[tid:%u] Miss on block @ %08p completed, but squashed\n",
                tid, cache_req->inst->instAddr());
        cache_req->setMemAccCompleted();
    }
}

void
FetchUnit::squash(DynInstPtr inst, int stage_num,
                  InstSeqNum squash_seq_num, ThreadID tid)
{
    CacheUnit::squash(inst, stage_num, squash_seq_num, tid);
}

