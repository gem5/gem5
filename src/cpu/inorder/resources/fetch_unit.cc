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

#include <list>
#include <vector>

#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/utility.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/resources/cache_unit.hh"
#include "cpu/inorder/resources/fetch_unit.hh"
#include "cpu/inorder/cpu.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource_pool.hh"
#include "debug/Activity.hh"
#include "debug/InOrderCachePort.hh"
#include "debug/InOrderStall.hh"
#include "debug/RefCount.hh"
#include "debug/ThreadModel.hh"
#include "mem/request.hh"

using namespace std;
using namespace TheISA;
using namespace ThePipeline;

FetchUnit::FetchUnit(string res_name, int res_id, int res_width,
                     Cycles res_latency, InOrderCPU *_cpu,
                     ThePipeline::Params *params)
    : CacheUnit(res_name, res_id, res_width, res_latency, _cpu, params),
      instSize(sizeof(TheISA::MachInst)), fetchBuffSize(params->fetchBuffSize)
{
    for (int tid = 0; tid < MaxThreads; tid++)
        decoder[tid] = new Decoder;
}

FetchUnit::~FetchUnit()
{
    std::list<FetchBlock*>::iterator fetch_it = fetchBuffer.begin();
    std::list<FetchBlock*>::iterator end_it = fetchBuffer.end();
    while (fetch_it != end_it) {
        delete (*fetch_it)->block;
        delete *fetch_it;
        fetch_it++;
    }
    fetchBuffer.clear();


    std::list<FetchBlock*>::iterator pend_it = pendingFetch.begin();
    std::list<FetchBlock*>::iterator pend_end = pendingFetch.end();
    while (pend_it != pend_end) {
        if ((*pend_it)->block) {
            delete (*pend_it)->block;
        }

        delete *pend_it;
        pend_it++;
    }
    pendingFetch.clear();
}

void
FetchUnit::createMachInst(std::list<FetchBlock*>::iterator fetch_it,
                          DynInstPtr inst)
{
    Addr block_addr = cacheBlockAlign(inst->getMemAddr());
    Addr fetch_addr = inst->getMemAddr();
    unsigned fetch_offset = (fetch_addr - block_addr) / instSize;
    ThreadID tid = inst->readTid();
    TheISA::PCState instPC = inst->pcState();


    DPRINTF(InOrderCachePort, "Creating instruction [sn:%i] w/fetch data @"
            "addr:%08p block:%08p\n", inst->seqNum, fetch_addr, block_addr);

    assert((*fetch_it)->valid);

    TheISA::MachInst *fetchInsts =
        reinterpret_cast<TheISA::MachInst *>((*fetch_it)->block);

    MachInst mach_inst =
        TheISA::gtoh(fetchInsts[fetch_offset]);

    decoder[tid]->moreBytes(instPC, inst->instAddr(), mach_inst);
    assert(decoder[tid]->instReady());
    inst->setStaticInst(decoder[tid]->decode(instPC));
    inst->pcState(instPC);
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
    ScheduleEntry* sched_entry = *inst->curSkedEntry;
    CacheRequest* cache_req = dynamic_cast<CacheRequest*>(reqs[slot_num]);

    if (!inst->validMemAddr()) {
        panic("Mem. Addr. must be set before requesting cache access\n");
    }

    assert(sched_entry->cmd == InitiateFetch);

    DPRINTF(InOrderCachePort,
            "[tid:%i]: Fetch request from [sn:%i] for addr %08p\n",
            inst->readTid(), inst->seqNum, inst->getMemAddr());

    cache_req->setRequest(inst, stage_num, id, slot_num,
                          sched_entry->cmd, MemCmd::ReadReq,
                          inst->curSkedEntry->idx);

    return cache_req;
}

void
FetchUnit::setupMemRequest(DynInstPtr inst, CacheReqPtr cache_req,
                           int acc_size, int flags)
{
    ThreadID tid = inst->readTid();
    Addr aligned_addr = cacheBlockAlign(inst->getMemAddr());
    if (cache_req->memReq == NULL) {
        cache_req->memReq =
            new Request(tid, aligned_addr, acc_size, flags,
                        cpu->instMasterId(), inst->instAddr(), cpu->readCpuId(),
                        tid);
        DPRINTF(InOrderCachePort, "[sn:%i] Created memReq @%x, ->%x\n",
                inst->seqNum, &cache_req->memReq, cache_req->memReq);
    }
}

std::list<FetchUnit::FetchBlock*>::iterator
FetchUnit::findBlock(std::list<FetchBlock*> &fetch_blocks, int asid,
                     Addr block_addr)
{
    std::list<FetchBlock*>::iterator fetch_it = fetch_blocks.begin();
    std::list<FetchBlock*>::iterator end_it = fetch_blocks.end();

    while (fetch_it != end_it) {
        if ((*fetch_it)->asid == asid &&
            (*fetch_it)->addr == block_addr) {
            return fetch_it;
        }

        fetch_it++;
    }

    return fetch_it;
}

std::list<FetchUnit::FetchBlock*>::iterator
FetchUnit::findReplacementBlock()
{
    std::list<FetchBlock*>::iterator fetch_it = fetchBuffer.begin();
    std::list<FetchBlock*>::iterator end_it = fetchBuffer.end();

    while (fetch_it != end_it) {
        if ((*fetch_it)->cnt == 0) {
            return fetch_it;
        } else {
            DPRINTF(InOrderCachePort, "Block %08p has %i insts pending.\n",
                    (*fetch_it)->addr, (*fetch_it)->cnt);
        }
        fetch_it++;
    }

    return fetch_it;
}

void
FetchUnit::markBlockUsed(std::list<FetchBlock*>::iterator block_it)
{
    // Move block from whatever location it is in fetch buffer
    // to the back (represents most-recently-used location)
    if (block_it != fetchBuffer.end()) {
        FetchBlock *mru_blk = *block_it;
        fetchBuffer.erase(block_it);
        fetchBuffer.push_back(mru_blk);
    }
}

int
FetchUnit::blocksInUse()
{
    std::list<FetchBlock*>::iterator fetch_it = fetchBuffer.begin();
    std::list<FetchBlock*>::iterator end_it = fetchBuffer.end();

    int cnt = 0;
    while (fetch_it != end_it) {
        if ((*fetch_it)->cnt > 0)
            cnt++;

        fetch_it++;
    }

    return cnt;
}

void
FetchUnit::clearFetchBuffer()
{
    std::list<FetchBlock*>::iterator fetch_it = fetchBuffer.begin();
    std::list<FetchBlock*>::iterator end_it = fetchBuffer.end();

    while (fetch_it != end_it) {
        if ((*fetch_it)->block) {
            delete [] (*fetch_it)->block;
        }
        delete *fetch_it;
        fetch_it++;
    }
    fetchBuffer.clear();
}

void
FetchUnit::execute(int slot_num)
{
    CacheReqPtr cache_req = dynamic_cast<CacheReqPtr>(reqs[slot_num]);
    assert(cache_req);

    if (cachePortBlocked && cache_req->cmd == InitiateFetch) {
        DPRINTF(InOrderCachePort, "Cache Port Blocked. Cannot Access\n");
        cache_req->done(false);
        return;
    }

    DynInstPtr inst = cache_req->inst;
    ThreadID tid = inst->readTid();
    Addr block_addr = cacheBlockAlign(inst->getMemAddr());
    int asid = cpu->asid[tid];

    if (inst->fault != NoFault) {
        DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", tid, inst->seqNum, inst->fault->name(),
                cacheBlockAlign(inst->getMemAddr()));
        finishCacheUnitReq(inst, cache_req);
        return;
    }

    switch (cache_req->cmd)
    {
      case InitiateFetch:
        {
            // Check to see if we've already got this request buffered
            // or pending to be buffered
            bool do_fetch = true;
            int total_pending = pendingFetch.size() + blocksInUse();

            std::list<FetchBlock*>::iterator pending_it;
            pending_it = findBlock(pendingFetch, asid, block_addr);
            if (pending_it != pendingFetch.end()) {
                (*pending_it)->cnt++;
                do_fetch = false;

                DPRINTF(InOrderCachePort, "%08p is a pending fetch block "
                        "(pending:%i).\n", block_addr,
                        (*pending_it)->cnt);
            } else if (total_pending < fetchBuffSize) {
                std::list<FetchBlock*>::iterator buff_it;
                buff_it = findBlock(fetchBuffer, asid, block_addr);
                if (buff_it != fetchBuffer.end()) {
                    (*buff_it)->cnt++;
                    do_fetch = false;

                    DPRINTF(InOrderCachePort, "%08p is in fetch buffer "
                            "(pending:%i).\n", block_addr, (*buff_it)->cnt);
                }
            }

            if (!do_fetch) {
                DPRINTF(InOrderCachePort, "Inst. [sn:%i] marked to be filled "
                        "through fetch buffer.\n", inst->seqNum);
                cache_req->fetchBufferFill = true;
                cache_req->setCompleted(true);
                return;
            }

            // Check to see if there is room in the fetchbuffer for this instruction.
            // If not, block this request.
            if (total_pending >= fetchBuffSize) {
                DPRINTF(InOrderCachePort, "No room available in fetch buffer.\n");
                cache_req->done(false);
                return;
            }

            doTLBAccess(inst, cache_req, cacheBlkSize, Request::INST_FETCH, TheISA::TLB::Execute);

            if (inst->fault == NoFault) {
                DPRINTF(InOrderCachePort,
                        "[tid:%u]: Initiating fetch access to %s for "
                        "addr:%#x (block:%#x)\n", tid, name(),
                        cache_req->inst->getMemAddr(), block_addr);

                cache_req->reqData = new uint8_t[cacheBlkSize];

                inst->setCurResSlot(slot_num);

                doCacheAccess(inst);

                if (cache_req->isMemAccPending()) {
                    pendingFetch.push_back(new FetchBlock(asid, block_addr));

                    // mark replacement block
                }
            }

            break;
        }

      case CompleteFetch:
        if (inst->fault != NoFault) {
            DPRINTF(InOrderCachePort,
                "[tid:%i]: [sn:%i]: Detected %s fault @ %x. Forwarding to "
                "next stage.\n", tid, inst->seqNum, inst->fault->name(),
                inst->getMemAddr());
            finishCacheUnitReq(inst, cache_req);
            return;
        }

        if (cache_req->fetchBufferFill) {
            // Block request if it's depending on a previous fetch, but it hasnt made it yet
            std::list<FetchBlock*>::iterator fetch_it = findBlock(fetchBuffer, asid, block_addr);
            if (fetch_it == fetchBuffer.end()) {
                DPRINTF(InOrderCachePort, "%#x not available yet\n",
                        block_addr);
                cache_req->setCompleted(false);
                return;
            }

            // Make New Instruction
            createMachInst(fetch_it, inst);
            if (inst->traceData) {
                inst->traceData->setStaticInst(inst->staticInst);
                inst->traceData->setPC(inst->pcState());
            }

            // FetchBuffer Book-Keeping
            (*fetch_it)->cnt--;
            assert((*fetch_it)->cnt >= 0);
            markBlockUsed(fetch_it);

            cache_req->done();
            return;
        }

        if (cache_req->isMemAccComplete()) {
            if (fetchBuffer.size() >= fetchBuffSize) {
                // If there is no replacement block, then we'll just have
                // to wait till that gets cleared before satisfying the fetch
                // for this instruction
                std::list<FetchBlock*>::iterator repl_it  =
                    findReplacementBlock();
                if (repl_it == fetchBuffer.end()) {
                    DPRINTF(InOrderCachePort, "Unable to find replacement block"
                            " and complete fetch.\n");
                    cache_req->setCompleted(false);
                    return;
                }

                delete [] (*repl_it)->block;
                delete *repl_it;
                fetchBuffer.erase(repl_it);
            }

            DPRINTF(InOrderCachePort,
                    "[tid:%i]: Completing Fetch Access for [sn:%i]\n",
                    tid, inst->seqNum);

            // Make New Instruction
            std::list<FetchBlock*>::iterator fetch_it  =
                findBlock(pendingFetch, asid, block_addr);

            assert(fetch_it != pendingFetch.end());
            assert((*fetch_it)->valid);

            createMachInst(fetch_it, inst);
            if (inst->traceData) {
                inst->traceData->setStaticInst(inst->staticInst);
                inst->traceData->setPC(inst->pcState());
            }


            // Update instructions waiting on new fetch block
            FetchBlock *new_block = (*fetch_it);
            new_block->cnt--;
            assert(new_block->cnt >= 0);

            // Finally, update FetchBuffer w/Pending Block into the
            // MRU location
            pendingFetch.erase(fetch_it);
            fetchBuffer.push_back(new_block);

            DPRINTF(InOrderCachePort, "[tid:%i]: Instruction [sn:%i] is: %s\n",
                    tid, inst->seqNum,
                    inst->staticInst->disassemble(inst->instAddr()));

            inst->unsetMemAddr();

            cache_req->done();
        } else {
            DPRINTF(InOrderCachePort,
                     "[tid:%i]: [sn:%i]: Unable to Complete Fetch Access\n",
                    tid, inst->seqNum);
            DPRINTF(InOrderStall,
                    "STALL: [tid:%i]: Fetch miss from %08p\n",
                    tid, cache_req->inst->instAddr());
            cache_req->setCompleted(false);
            // NOTE: For SwitchOnCacheMiss ThreadModel, we *don't* switch on
            //       fetch miss, but we could ...
            // cache_req->setMemStall(true);
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
    // @todo: use pkt Sender state here to be consistent with other
    // cpu models
    CacheReqPacket* cache_pkt = dynamic_cast<CacheReqPacket*>(pkt);
    assert(cache_pkt);

    DPRINTF(InOrderCachePort, "Finished request for %x\n",
            cache_pkt->getAddr());

    if (processSquash(cache_pkt))
        return;

    Addr block_addr = cacheBlockAlign(cache_pkt->cacheReq->
                                      getInst()->getMemAddr());

    DPRINTF(InOrderCachePort,
            "[tid:%u]: [sn:%i]: Waking from fetch access to addr:%#x(phys:%#x), size:%i\n",
            cache_pkt->cacheReq->getInst()->readTid(),
            cache_pkt->cacheReq->getInst()->seqNum,
            block_addr, cache_pkt->getAddr(), cache_pkt->getSize());

    // Cast to correct request type
    CacheRequest *cache_req = dynamic_cast<CacheReqPtr>(
        findRequest(cache_pkt->cacheReq->getInst(), cache_pkt->instIdx));

    if (!cache_req) {
        panic("[tid:%u]: [sn:%i]: Can't find slot for fetch access to "
              "addr. %08p\n", cache_pkt->cacheReq->getInst()->readTid(),
              cache_pkt->cacheReq->getInst()->seqNum,
              block_addr);
    }

    // Get resource request info
    unsigned stage_num = cache_req->getStageNum();
    DynInstPtr inst = cache_req->inst;
    ThreadID tid = cache_req->inst->readTid();
    short asid = cpu->asid[tid];

    assert(!cache_req->isSquashed());
    assert(inst->curSkedEntry->cmd == CompleteFetch);

    DPRINTF(InOrderCachePort,
            "[tid:%u]: [sn:%i]: Processing fetch access for block %#x\n",
            tid, inst->seqNum, block_addr);

    std::list<FetchBlock*>::iterator pend_it = findBlock(pendingFetch, asid,
                                                         block_addr);
    assert(pend_it != pendingFetch.end());

    // Copy Data to pendingFetch queue...
    (*pend_it)->block = new uint8_t[cacheBlkSize];
    memcpy((*pend_it)->block, cache_pkt->getPtr<uint8_t>(), cacheBlkSize);
    (*pend_it)->valid = true;

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
FetchUnit::squashCacheRequest(CacheReqPtr req_ptr)
{
    DynInstPtr inst = req_ptr->getInst();
    ThreadID tid = inst->readTid();
    Addr block_addr = cacheBlockAlign(inst->getMemAddr());
    int asid = cpu->asid[tid];

    // Check Fetch Buffer (or pending fetch) for this block and
    // update pending counts
    std::list<FetchBlock*>::iterator buff_it = findBlock(fetchBuffer,
                                                         asid,
                                                         block_addr);
    if (buff_it != fetchBuffer.end()) {
        (*buff_it)->cnt--;
        DPRINTF(InOrderCachePort, "[sn:%i] Removing Pending Access "
                "for Fetch Buffer block %08p (cnt=%i)\n", inst->seqNum,
                block_addr, (*buff_it)->cnt);
        assert((*buff_it)->cnt >= 0);
    } else {
        std::list<FetchBlock*>::iterator block_it = findBlock(pendingFetch,
                                                              asid,
                                                              block_addr);
        if (block_it != pendingFetch.end()) {
            (*block_it)->cnt--;
            DPRINTF(InOrderCachePort, "[sn:%i] Removing Pending Access "
                    "for Pending Buffer Block %08p (cnt=%i)\n",
                    inst->seqNum,
                    block_addr, (*block_it)->cnt);
            assert((*block_it)->cnt >= 0);
            if ((*block_it)->cnt == 0) {
                if ((*block_it)->block) {
                    delete [] (*block_it)->block;
                }
                delete *block_it;
                pendingFetch.erase(block_it);
            }
        }
    }

    CacheUnit::squashCacheRequest(req_ptr);
}

void
FetchUnit::trap(const Fault &fault, ThreadID tid, DynInstPtr inst)
{
    //@todo: per thread?
    decoder[tid]->reset();

    //@todo: squash using dummy inst seq num
    squash(NULL, NumStages - 1, 0, tid);

    //@todo: make sure no blocks are in use
    assert(blocksInUse() == 0);
    assert(pendingFetch.size() == 0);

    //@todo: clear pendingFetch and fetchBuffer
    clearFetchBuffer();
}
