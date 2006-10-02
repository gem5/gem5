/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#include "config/use_checker.hh"

#include "cpu/o3/lsq.hh"
#include "cpu/o3/lsq_unit.hh"
#include "base/str.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

#if USE_CHECKER
#include "cpu/checker/cpu.hh"
#endif

template<class Impl>
LSQUnit<Impl>::WritebackEvent::WritebackEvent(DynInstPtr &_inst, PacketPtr _pkt,
                                              LSQUnit *lsq_ptr)
    : Event(&mainEventQueue), inst(_inst), pkt(_pkt), lsqPtr(lsq_ptr)
{
    this->setFlags(Event::AutoDelete);
}

template<class Impl>
void
LSQUnit<Impl>::WritebackEvent::process()
{
    if (!lsqPtr->isSwitchedOut()) {
        lsqPtr->writeback(inst, pkt);
    }
    delete pkt;
}

template<class Impl>
const char *
LSQUnit<Impl>::WritebackEvent::description()
{
    return "Store writeback event";
}

template<class Impl>
void
LSQUnit<Impl>::completeDataAccess(PacketPtr pkt)
{
    LSQSenderState *state = dynamic_cast<LSQSenderState *>(pkt->senderState);
    DynInstPtr inst = state->inst;
    DPRINTF(IEW, "Writeback event [sn:%lli]\n", inst->seqNum);
    DPRINTF(Activity, "Activity: Writeback event [sn:%lli]\n", inst->seqNum);

    //iewStage->ldstQueue.removeMSHR(inst->threadNumber,inst->seqNum);

    if (isSwitchedOut() || inst->isSquashed()) {
        iewStage->decrWb(inst->seqNum);
        delete state;
        delete pkt;
        return;
    } else {
        if (!state->noWB) {
            writeback(inst, pkt);
        }

        if (inst->isStore()) {
            completeStore(state->idx);
        }
    }

    delete state;
    delete pkt;
}

template <class Impl>
LSQUnit<Impl>::LSQUnit()
    : loads(0), stores(0), storesToWB(0), stalled(false),
      isStoreBlocked(false), isLoadBlocked(false),
      loadBlockedHandled(false)
{
}

template<class Impl>
void
LSQUnit<Impl>::init(Params *params, LSQ *lsq_ptr, unsigned maxLQEntries,
                    unsigned maxSQEntries, unsigned id)
{
    DPRINTF(LSQUnit, "Creating LSQUnit%i object.\n",id);

    switchedOut = false;

    lsq = lsq_ptr;

    lsqID = id;

    // Add 1 for the sentinel entry (they are circular queues).
    LQEntries = maxLQEntries + 1;
    SQEntries = maxSQEntries + 1;

    loadQueue.resize(LQEntries);
    storeQueue.resize(SQEntries);

    loadHead = loadTail = 0;

    storeHead = storeWBIdx = storeTail = 0;

    usedPorts = 0;
    cachePorts = params->cachePorts;

    memDepViolator = NULL;

    blockedLoadSeqNum = 0;
}

template<class Impl>
void
LSQUnit<Impl>::setCPU(O3CPU *cpu_ptr)
{
    cpu = cpu_ptr;

#if USE_CHECKER
    if (cpu->checker) {
        cpu->checker->setDcachePort(dcachePort);
    }
#endif
}

template<class Impl>
std::string
LSQUnit<Impl>::name() const
{
    if (Impl::MaxThreads == 1) {
        return iewStage->name() + ".lsq";
    } else {
        return iewStage->name() + ".lsq.thread." + to_string(lsqID);
    }
}

template<class Impl>
void
LSQUnit<Impl>::regStats()
{
    lsqForwLoads
        .name(name() + ".forwLoads")
        .desc("Number of loads that had data forwarded from stores");

    invAddrLoads
        .name(name() + ".invAddrLoads")
        .desc("Number of loads ignored due to an invalid address");

    lsqSquashedLoads
        .name(name() + ".squashedLoads")
        .desc("Number of loads squashed");

    lsqIgnoredResponses
        .name(name() + ".ignoredResponses")
        .desc("Number of memory responses ignored because the instruction is squashed");

    lsqMemOrderViolation
        .name(name() + ".memOrderViolation")
        .desc("Number of memory ordering violations");

    lsqSquashedStores
        .name(name() + ".squashedStores")
        .desc("Number of stores squashed");

    invAddrSwpfs
        .name(name() + ".invAddrSwpfs")
        .desc("Number of software prefetches ignored due to an invalid address");

    lsqBlockedLoads
        .name(name() + ".blockedLoads")
        .desc("Number of blocked loads due to partial load-store forwarding");

    lsqRescheduledLoads
        .name(name() + ".rescheduledLoads")
        .desc("Number of loads that were rescheduled");

    lsqCacheBlocked
        .name(name() + ".cacheBlocked")
        .desc("Number of times an access to memory failed due to the cache being blocked");
}

template<class Impl>
void
LSQUnit<Impl>::clearLQ()
{
    loadQueue.clear();
}

template<class Impl>
void
LSQUnit<Impl>::clearSQ()
{
    storeQueue.clear();
}

template<class Impl>
void
LSQUnit<Impl>::switchOut()
{
    switchedOut = true;
    for (int i = 0; i < loadQueue.size(); ++i) {
        assert(!loadQueue[i]);
        loadQueue[i] = NULL;
    }

    assert(storesToWB == 0);
}

template<class Impl>
void
LSQUnit<Impl>::takeOverFrom()
{
    switchedOut = false;
    loads = stores = storesToWB = 0;

    loadHead = loadTail = 0;

    storeHead = storeWBIdx = storeTail = 0;

    usedPorts = 0;

    memDepViolator = NULL;

    blockedLoadSeqNum = 0;

    stalled = false;
    isLoadBlocked = false;
    loadBlockedHandled = false;
}

template<class Impl>
void
LSQUnit<Impl>::resizeLQ(unsigned size)
{
    unsigned size_plus_sentinel = size + 1;
    assert(size_plus_sentinel >= LQEntries);

    if (size_plus_sentinel > LQEntries) {
        while (size_plus_sentinel > loadQueue.size()) {
            DynInstPtr dummy;
            loadQueue.push_back(dummy);
            LQEntries++;
        }
    } else {
        LQEntries = size_plus_sentinel;
    }

}

template<class Impl>
void
LSQUnit<Impl>::resizeSQ(unsigned size)
{
    unsigned size_plus_sentinel = size + 1;
    if (size_plus_sentinel > SQEntries) {
        while (size_plus_sentinel > storeQueue.size()) {
            SQEntry dummy;
            storeQueue.push_back(dummy);
            SQEntries++;
        }
    } else {
        SQEntries = size_plus_sentinel;
    }
}

template <class Impl>
void
LSQUnit<Impl>::insert(DynInstPtr &inst)
{
    assert(inst->isMemRef());

    assert(inst->isLoad() || inst->isStore());

    if (inst->isLoad()) {
        insertLoad(inst);
    } else {
        insertStore(inst);
    }

    inst->setInLSQ();
}

template <class Impl>
void
LSQUnit<Impl>::insertLoad(DynInstPtr &load_inst)
{
    assert((loadTail + 1) % LQEntries != loadHead);
    assert(loads < LQEntries);

    DPRINTF(LSQUnit, "Inserting load PC %#x, idx:%i [sn:%lli]\n",
            load_inst->readPC(), loadTail, load_inst->seqNum);

    load_inst->lqIdx = loadTail;

    if (stores == 0) {
        load_inst->sqIdx = -1;
    } else {
        load_inst->sqIdx = storeTail;
    }

    loadQueue[loadTail] = load_inst;

    incrLdIdx(loadTail);

    ++loads;
}

template <class Impl>
void
LSQUnit<Impl>::insertStore(DynInstPtr &store_inst)
{
    // Make sure it is not full before inserting an instruction.
    assert((storeTail + 1) % SQEntries != storeHead);
    assert(stores < SQEntries);

    DPRINTF(LSQUnit, "Inserting store PC %#x, idx:%i [sn:%lli]\n",
            store_inst->readPC(), storeTail, store_inst->seqNum);

    store_inst->sqIdx = storeTail;
    store_inst->lqIdx = loadTail;

    storeQueue[storeTail] = SQEntry(store_inst);

    incrStIdx(storeTail);

    ++stores;
}

template <class Impl>
typename Impl::DynInstPtr
LSQUnit<Impl>::getMemDepViolator()
{
    DynInstPtr temp = memDepViolator;

    memDepViolator = NULL;

    return temp;
}

template <class Impl>
unsigned
LSQUnit<Impl>::numFreeEntries()
{
    unsigned free_lq_entries = LQEntries - loads;
    unsigned free_sq_entries = SQEntries - stores;

    // Both the LQ and SQ entries have an extra dummy entry to differentiate
    // empty/full conditions.  Subtract 1 from the free entries.
    if (free_lq_entries < free_sq_entries) {
        return free_lq_entries - 1;
    } else {
        return free_sq_entries - 1;
    }
}

template <class Impl>
int
LSQUnit<Impl>::numLoadsReady()
{
    int load_idx = loadHead;
    int retval = 0;

    while (load_idx != loadTail) {
        assert(loadQueue[load_idx]);

        if (loadQueue[load_idx]->readyToIssue()) {
            ++retval;
        }
    }

    return retval;
}

template <class Impl>
Fault
LSQUnit<Impl>::executeLoad(DynInstPtr &inst)
{
    // Execute a specific load.
    Fault load_fault = NoFault;

    DPRINTF(LSQUnit, "Executing load PC %#x, [sn:%lli]\n",
            inst->readPC(),inst->seqNum);

    load_fault = inst->initiateAcc();

    // If the instruction faulted, then we need to send it along to commit
    // without the instruction completing.
    if (load_fault != NoFault) {
        // Send this instruction to commit, also make sure iew stage
        // realizes there is activity.
        // Mark it as executed unless it is an uncached load that
        // needs to hit the head of commit.
        if (!(inst->req->getFlags() & UNCACHEABLE) || inst->isAtCommit()) {
            inst->setExecuted();
        }
        iewStage->instToCommit(inst);
        iewStage->activityThisCycle();
    }

    return load_fault;
}

template <class Impl>
Fault
LSQUnit<Impl>::executeStore(DynInstPtr &store_inst)
{
    using namespace TheISA;
    // Make sure that a store exists.
    assert(stores != 0);

    int store_idx = store_inst->sqIdx;

    DPRINTF(LSQUnit, "Executing store PC %#x [sn:%lli]\n",
            store_inst->readPC(), store_inst->seqNum);

    // Check the recently completed loads to see if any match this store's
    // address.  If so, then we have a memory ordering violation.
    int load_idx = store_inst->lqIdx;

    Fault store_fault = store_inst->initiateAcc();

    if (storeQueue[store_idx].size == 0) {
        DPRINTF(LSQUnit,"Fault on Store PC %#x, [sn:%lli],Size = 0\n",
                store_inst->readPC(),store_inst->seqNum);

        return store_fault;
    }

    assert(store_fault == NoFault);

    if (store_inst->isStoreConditional()) {
        // Store conditionals need to set themselves as able to
        // writeback if we haven't had a fault by here.
        storeQueue[store_idx].canWB = true;

        ++storesToWB;
    }

    if (!memDepViolator) {
        while (load_idx != loadTail) {
            // Really only need to check loads that have actually executed
            // It's safe to check all loads because effAddr is set to
            // InvalAddr when the dyn inst is created.

            // @todo: For now this is extra conservative, detecting a
            // violation if the addresses match assuming all accesses
            // are quad word accesses.

            // @todo: Fix this, magic number being used here
            if ((loadQueue[load_idx]->effAddr >> 8) ==
                (store_inst->effAddr >> 8)) {
                // A load incorrectly passed this store.  Squash and refetch.
                // For now return a fault to show that it was unsuccessful.
                memDepViolator = loadQueue[load_idx];
                ++lsqMemOrderViolation;

                return genMachineCheckFault();
            }

            incrLdIdx(load_idx);
        }

        // If we've reached this point, there was no violation.
        memDepViolator = NULL;
    }

    return store_fault;
}

template <class Impl>
void
LSQUnit<Impl>::commitLoad()
{
    assert(loadQueue[loadHead]);

    DPRINTF(LSQUnit, "Committing head load instruction, PC %#x\n",
            loadQueue[loadHead]->readPC());

    loadQueue[loadHead] = NULL;

    incrLdIdx(loadHead);

    --loads;
}

template <class Impl>
void
LSQUnit<Impl>::commitLoads(InstSeqNum &youngest_inst)
{
    assert(loads == 0 || loadQueue[loadHead]);

    while (loads != 0 && loadQueue[loadHead]->seqNum <= youngest_inst) {
        commitLoad();
    }
}

template <class Impl>
void
LSQUnit<Impl>::commitStores(InstSeqNum &youngest_inst)
{
    assert(stores == 0 || storeQueue[storeHead].inst);

    int store_idx = storeHead;

    while (store_idx != storeTail) {
        assert(storeQueue[store_idx].inst);
        // Mark any stores that are now committed and have not yet
        // been marked as able to write back.
        if (!storeQueue[store_idx].canWB) {
            if (storeQueue[store_idx].inst->seqNum > youngest_inst) {
                break;
            }
            DPRINTF(LSQUnit, "Marking store as able to write back, PC "
                    "%#x [sn:%lli]\n",
                    storeQueue[store_idx].inst->readPC(),
                    storeQueue[store_idx].inst->seqNum);

            storeQueue[store_idx].canWB = true;

            ++storesToWB;
        }

        incrStIdx(store_idx);
    }
}

template <class Impl>
void
LSQUnit<Impl>::writebackStores()
{
    while (storesToWB > 0 &&
           storeWBIdx != storeTail &&
           storeQueue[storeWBIdx].inst &&
           storeQueue[storeWBIdx].canWB &&
           usedPorts < cachePorts) {

        if (isStoreBlocked || lsq->cacheBlocked()) {
            DPRINTF(LSQUnit, "Unable to write back any more stores, cache"
                    " is blocked!\n");
            break;
        }

        // Store didn't write any data so no need to write it back to
        // memory.
        if (storeQueue[storeWBIdx].size == 0) {
            completeStore(storeWBIdx);

            incrStIdx(storeWBIdx);

            continue;
        }

        ++usedPorts;

        if (storeQueue[storeWBIdx].inst->isDataPrefetch()) {
            incrStIdx(storeWBIdx);

            continue;
        }

        assert(storeQueue[storeWBIdx].req);
        assert(!storeQueue[storeWBIdx].committed);

        DynInstPtr inst = storeQueue[storeWBIdx].inst;

        Request *req = storeQueue[storeWBIdx].req;
        storeQueue[storeWBIdx].committed = true;

        assert(!inst->memData);
        inst->memData = new uint8_t[64];
        memcpy(inst->memData, (uint8_t *)&storeQueue[storeWBIdx].data,
               req->getSize());

        PacketPtr data_pkt = new Packet(req, Packet::WriteReq, Packet::Broadcast);
        data_pkt->dataStatic(inst->memData);

        LSQSenderState *state = new LSQSenderState;
        state->isLoad = false;
        state->idx = storeWBIdx;
        state->inst = inst;
        data_pkt->senderState = state;

        DPRINTF(LSQUnit, "D-Cache: Writing back store idx:%i PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                storeWBIdx, storeQueue[storeWBIdx].inst->readPC(),
                req->getPaddr(), *(inst->memData),
                storeQueue[storeWBIdx].inst->seqNum);

        // @todo: Remove this SC hack once the memory system handles it.
        if (req->getFlags() & LOCKED) {
            if (req->getFlags() & UNCACHEABLE) {
                req->setScResult(2);
            } else {
                if (cpu->lockFlag) {
                    req->setScResult(1);
                } else {
                    req->setScResult(0);
                    // Hack: Instantly complete this store.
                    completeDataAccess(data_pkt);
                    incrStIdx(storeWBIdx);
                    continue;
                }
            }
        } else {
            // Non-store conditionals do not need a writeback.
            state->noWB = true;
        }

        if (!dcachePort->sendTiming(data_pkt)) {
            // Need to handle becoming blocked on a store.
            isStoreBlocked = true;
            ++lsqCacheBlocked;
            assert(retryPkt == NULL);
            retryPkt = data_pkt;
            lsq->setRetryTid(lsqID);
        } else {
            storePostSend(data_pkt);
        }
    }

    // Not sure this should set it to 0.
    usedPorts = 0;

    assert(stores >= 0 && storesToWB >= 0);
}

/*template <class Impl>
void
LSQUnit<Impl>::removeMSHR(InstSeqNum seqNum)
{
    list<InstSeqNum>::iterator mshr_it = find(mshrSeqNums.begin(),
                                              mshrSeqNums.end(),
                                              seqNum);

    if (mshr_it != mshrSeqNums.end()) {
        mshrSeqNums.erase(mshr_it);
        DPRINTF(LSQUnit, "Removing MSHR. count = %i\n",mshrSeqNums.size());
    }
}*/

template <class Impl>
void
LSQUnit<Impl>::squash(const InstSeqNum &squashed_num)
{
    DPRINTF(LSQUnit, "Squashing until [sn:%lli]!"
            "(Loads:%i Stores:%i)\n", squashed_num, loads, stores);

    int load_idx = loadTail;
    decrLdIdx(load_idx);

    while (loads != 0 && loadQueue[load_idx]->seqNum > squashed_num) {
        DPRINTF(LSQUnit,"Load Instruction PC %#x squashed, "
                "[sn:%lli]\n",
                loadQueue[load_idx]->readPC(),
                loadQueue[load_idx]->seqNum);

        if (isStalled() && load_idx == stallingLoadIdx) {
            stalled = false;
            stallingStoreIsn = 0;
            stallingLoadIdx = 0;
        }

        // Clear the smart pointer to make sure it is decremented.
        loadQueue[load_idx]->setSquashed();
        loadQueue[load_idx] = NULL;
        --loads;

        // Inefficient!
        loadTail = load_idx;

        decrLdIdx(load_idx);
        ++lsqSquashedLoads;
    }

    if (isLoadBlocked) {
        if (squashed_num < blockedLoadSeqNum) {
            isLoadBlocked = false;
            loadBlockedHandled = false;
            blockedLoadSeqNum = 0;
        }
    }

    int store_idx = storeTail;
    decrStIdx(store_idx);

    while (stores != 0 &&
           storeQueue[store_idx].inst->seqNum > squashed_num) {
        // Instructions marked as can WB are already committed.
        if (storeQueue[store_idx].canWB) {
            break;
        }

        DPRINTF(LSQUnit,"Store Instruction PC %#x squashed, "
                "idx:%i [sn:%lli]\n",
                storeQueue[store_idx].inst->readPC(),
                store_idx, storeQueue[store_idx].inst->seqNum);

        // I don't think this can happen.  It should have been cleared
        // by the stalling load.
        if (isStalled() &&
            storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
            panic("Is stalled should have been cleared by stalling load!\n");
            stalled = false;
            stallingStoreIsn = 0;
        }

        // Clear the smart pointer to make sure it is decremented.
        storeQueue[store_idx].inst->setSquashed();
        storeQueue[store_idx].inst = NULL;
        storeQueue[store_idx].canWB = 0;

        storeQueue[store_idx].req = NULL;
        --stores;

        // Inefficient!
        storeTail = store_idx;

        decrStIdx(store_idx);
        ++lsqSquashedStores;
    }
}

template <class Impl>
void
LSQUnit<Impl>::storePostSend(Packet *pkt)
{
    if (isStalled() &&
        storeQueue[storeWBIdx].inst->seqNum == stallingStoreIsn) {
        DPRINTF(LSQUnit, "Unstalling, stalling store [sn:%lli] "
                "load idx:%i\n",
                stallingStoreIsn, stallingLoadIdx);
        stalled = false;
        stallingStoreIsn = 0;
        iewStage->replayMemInst(loadQueue[stallingLoadIdx]);
    }

    if (!storeQueue[storeWBIdx].inst->isStoreConditional()) {
        // The store is basically completed at this time. This
        // only works so long as the checker doesn't try to
        // verify the value in memory for stores.
        storeQueue[storeWBIdx].inst->setCompleted();
#if USE_CHECKER
        if (cpu->checker) {
            cpu->checker->verify(storeQueue[storeWBIdx].inst);
        }
#endif
    }

    if (pkt->result != Packet::Success) {
        DPRINTF(LSQUnit,"D-Cache Write Miss on idx:%i!\n",
                storeWBIdx);

        DPRINTF(Activity, "Active st accessing mem miss [sn:%lli]\n",
                storeQueue[storeWBIdx].inst->seqNum);

        //mshrSeqNums.push_back(storeQueue[storeWBIdx].inst->seqNum);

        //DPRINTF(LSQUnit, "Added MSHR. count = %i\n",mshrSeqNums.size());

        // @todo: Increment stat here.
    } else {
        DPRINTF(LSQUnit,"D-Cache: Write Hit on idx:%i !\n",
                storeWBIdx);

        DPRINTF(Activity, "Active st accessing mem hit [sn:%lli]\n",
                storeQueue[storeWBIdx].inst->seqNum);
    }

    incrStIdx(storeWBIdx);
}

template <class Impl>
void
LSQUnit<Impl>::writeback(DynInstPtr &inst, PacketPtr pkt)
{
    iewStage->wakeCPU();

    // Squashed instructions do not need to complete their access.
    if (inst->isSquashed()) {
        iewStage->decrWb(inst->seqNum);
        assert(!inst->isStore());
        ++lsqIgnoredResponses;
        return;
    }

    if (!inst->isExecuted()) {
        inst->setExecuted();

        // Complete access to copy data to proper place.
        inst->completeAcc(pkt);
    }

    // Need to insert instruction into queue to commit
    iewStage->instToCommit(inst);

    iewStage->activityThisCycle();
}

template <class Impl>
void
LSQUnit<Impl>::completeStore(int store_idx)
{
    assert(storeQueue[store_idx].inst);
    storeQueue[store_idx].completed = true;
    --storesToWB;
    // A bit conservative because a store completion may not free up entries,
    // but hopefully avoids two store completions in one cycle from making
    // the CPU tick twice.
    cpu->wakeCPU();
    cpu->activityThisCycle();

    if (store_idx == storeHead) {
        do {
            incrStIdx(storeHead);

            --stores;
        } while (storeQueue[storeHead].completed &&
                 storeHead != storeTail);

        iewStage->updateLSQNextCycle = true;
    }

    DPRINTF(LSQUnit, "Completing store [sn:%lli], idx:%i, store head "
            "idx:%i\n",
            storeQueue[store_idx].inst->seqNum, store_idx, storeHead);

    if (isStalled() &&
        storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
        DPRINTF(LSQUnit, "Unstalling, stalling store [sn:%lli] "
                "load idx:%i\n",
                stallingStoreIsn, stallingLoadIdx);
        stalled = false;
        stallingStoreIsn = 0;
        iewStage->replayMemInst(loadQueue[stallingLoadIdx]);
    }

    storeQueue[store_idx].inst->setCompleted();

    // Tell the checker we've completed this instruction.  Some stores
    // may get reported twice to the checker, but the checker can
    // handle that case.
#if USE_CHECKER
    if (cpu->checker) {
        cpu->checker->verify(storeQueue[store_idx].inst);
    }
#endif
}

template <class Impl>
void
LSQUnit<Impl>::recvRetry()
{
    if (isStoreBlocked) {
        assert(retryPkt != NULL);

        if (dcachePort->sendTiming(retryPkt)) {
            storePostSend(retryPkt);
            retryPkt = NULL;
            isStoreBlocked = false;
            lsq->setRetryTid(-1);
        } else {
            // Still blocked!
            ++lsqCacheBlocked;
            lsq->setRetryTid(lsqID);
        }
    } else if (isLoadBlocked) {
        DPRINTF(LSQUnit, "Loads squash themselves and all younger insts, "
                "no need to resend packet.\n");
    } else {
        DPRINTF(LSQUnit, "Retry received but LSQ is no longer blocked.\n");
    }
}

template <class Impl>
inline void
LSQUnit<Impl>::incrStIdx(int &store_idx)
{
    if (++store_idx >= SQEntries)
        store_idx = 0;
}

template <class Impl>
inline void
LSQUnit<Impl>::decrStIdx(int &store_idx)
{
    if (--store_idx < 0)
        store_idx += SQEntries;
}

template <class Impl>
inline void
LSQUnit<Impl>::incrLdIdx(int &load_idx)
{
    if (++load_idx >= LQEntries)
        load_idx = 0;
}

template <class Impl>
inline void
LSQUnit<Impl>::decrLdIdx(int &load_idx)
{
    if (--load_idx < 0)
        load_idx += LQEntries;
}

template <class Impl>
void
LSQUnit<Impl>::dumpInsts()
{
    cprintf("Load store queue: Dumping instructions.\n");
    cprintf("Load queue size: %i\n", loads);
    cprintf("Load queue: ");

    int load_idx = loadHead;

    while (load_idx != loadTail && loadQueue[load_idx]) {
        cprintf("%#x ", loadQueue[load_idx]->readPC());

        incrLdIdx(load_idx);
    }

    cprintf("Store queue size: %i\n", stores);
    cprintf("Store queue: ");

    int store_idx = storeHead;

    while (store_idx != storeTail && storeQueue[store_idx].inst) {
        cprintf("%#x ", storeQueue[store_idx].inst->readPC());

        incrStIdx(store_idx);
    }

    cprintf("\n");
}
