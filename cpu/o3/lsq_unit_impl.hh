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
 */

#include "cpu/checker/cpu.hh"
#include "cpu/o3/lsq_unit.hh"
#include "base/str.hh"

template <class Impl>
LSQUnit<Impl>::StoreCompletionEvent::StoreCompletionEvent(int store_idx,
                                                          Event *wb_event,
                                                          LSQUnit<Impl> *lsq_ptr)
    : Event(&mainEventQueue),
      wbEvent(wb_event),
      storeIdx(store_idx),
      lsqPtr(lsq_ptr)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
LSQUnit<Impl>::StoreCompletionEvent::process()
{
    DPRINTF(LSQ, "Cache miss complete for store idx:%i\n", storeIdx);
    DPRINTF(Activity, "Activity: st writeback event idx:%i\n", storeIdx);

    //lsqPtr->removeMSHR(lsqPtr->storeQueue[storeIdx].inst->seqNum);

    if (lsqPtr->isSwitchedOut()) {
        if (wbEvent)
            delete wbEvent;

        return;
    }

    lsqPtr->cpu->wakeCPU();
    if (wbEvent) {
        wbEvent->process();
        delete wbEvent;
    }
    lsqPtr->completeStore(storeIdx);
}

template <class Impl>
const char *
LSQUnit<Impl>::StoreCompletionEvent::description()
{
    return "LSQ store completion event";
}

template <class Impl>
LSQUnit<Impl>::LSQUnit()
    : loads(0), stores(0), storesToWB(0), stalled(false), isLoadBlocked(false),
      loadBlockedHandled(false)
{
}

template<class Impl>
void
LSQUnit<Impl>::init(Params *params, unsigned maxLQEntries,
                    unsigned maxSQEntries, unsigned id)

{
    DPRINTF(LSQUnit, "Creating LSQUnit%i object.\n",id);

    switchedOut = false;

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

    dcacheInterface = params->dcacheInterface;

    memDepViolator = NULL;

    blockedLoadSeqNum = 0;
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

#if 0
template<class Impl>
void
LSQUnit<Impl>::setPageTable(PageTable *pt_ptr)
{
    DPRINTF(LSQUnit, "Setting the page table pointer.\n");
    pTable = pt_ptr;
}
#endif

template<class Impl>
void
LSQUnit<Impl>::switchOut()
{
    switchedOut = true;
    for (int i = 0; i < loadQueue.size(); ++i)
        loadQueue[i] = NULL;

    assert(storesToWB == 0);

    while (storesToWB > 0 &&
           storeWBIdx != storeTail &&
           storeQueue[storeWBIdx].inst &&
           storeQueue[storeWBIdx].canWB) {

        if (storeQueue[storeWBIdx].size == 0 ||
            storeQueue[storeWBIdx].inst->isDataPrefetch() ||
            storeQueue[storeWBIdx].committed ||
            storeQueue[storeWBIdx].req->flags & LOCKED) {
            incrStIdx(storeWBIdx);

            continue;
        }

        assert(storeQueue[storeWBIdx].req);
        assert(!storeQueue[storeWBIdx].committed);

        MemReqPtr req = storeQueue[storeWBIdx].req;
        storeQueue[storeWBIdx].committed = true;

        req->cmd = Write;
        req->completionEvent = NULL;
        req->time = curTick;
        assert(!req->data);
        req->data = new uint8_t[64];
        memcpy(req->data, (uint8_t *)&storeQueue[storeWBIdx].data, req->size);

        DPRINTF(LSQUnit, "D-Cache: Writing back store idx:%i PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                storeWBIdx,storeQueue[storeWBIdx].inst->readPC(),
                req->paddr, *(req->data),
                storeQueue[storeWBIdx].inst->seqNum);

        switch(storeQueue[storeWBIdx].size) {
          case 1:
            cpu->write(req, (uint8_t &)storeQueue[storeWBIdx].data);
            break;
          case 2:
            cpu->write(req, (uint16_t &)storeQueue[storeWBIdx].data);
            break;
          case 4:
            cpu->write(req, (uint32_t &)storeQueue[storeWBIdx].data);
            break;
          case 8:
            cpu->write(req, (uint64_t &)storeQueue[storeWBIdx].data);
            break;
          default:
            panic("Unexpected store size!\n");
        }
        incrStIdx(storeWBIdx);
    }
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

//    load_fault = inst->initiateAcc();
    load_fault = inst->execute();

    // If the instruction faulted, then we need to send it along to commit
    // without the instruction completing.
    if (load_fault != NoFault) {
        // Send this instruction to commit, also make sure iew stage
        // realizes there is activity.
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
//    Fault store_fault = store_inst->execute();

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

        // Store didn't write any data so no need to write it back to
        // memory.
        if (storeQueue[storeWBIdx].size == 0) {
            completeStore(storeWBIdx);

            incrStIdx(storeWBIdx);

            continue;
        }

        if (dcacheInterface && dcacheInterface->isBlocked()) {
            DPRINTF(LSQUnit, "Unable to write back any more stores, cache"
                    " is blocked!\n");
            break;
        }

        ++usedPorts;

        if (storeQueue[storeWBIdx].inst->isDataPrefetch()) {
            incrStIdx(storeWBIdx);

            continue;
        }

        assert(storeQueue[storeWBIdx].req);
        assert(!storeQueue[storeWBIdx].committed);

        MemReqPtr req = storeQueue[storeWBIdx].req;
        storeQueue[storeWBIdx].committed = true;

        req->cmd = Write;
        req->completionEvent = NULL;
        req->time = curTick;
        assert(!req->data);
        req->data = new uint8_t[64];
        memcpy(req->data, (uint8_t *)&storeQueue[storeWBIdx].data, req->size);

        DPRINTF(LSQUnit, "D-Cache: Writing back store idx:%i PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                storeWBIdx,storeQueue[storeWBIdx].inst->readPC(),
                req->paddr, *(req->data),
                storeQueue[storeWBIdx].inst->seqNum);

        switch(storeQueue[storeWBIdx].size) {
          case 1:
            cpu->write(req, (uint8_t &)storeQueue[storeWBIdx].data);
            break;
          case 2:
            cpu->write(req, (uint16_t &)storeQueue[storeWBIdx].data);
            break;
          case 4:
            cpu->write(req, (uint32_t &)storeQueue[storeWBIdx].data);
            break;
          case 8:
            cpu->write(req, (uint64_t &)storeQueue[storeWBIdx].data);
            break;
          default:
            panic("Unexpected store size!\n");
        }

        // Stores other than store conditionals are completed at this
        // time.  Mark them as completed and, if we have a checker,
        // tell it that the instruction is completed.
        // @todo: Figure out what time I can say stores are complete in
        // the timing memory.
        if (!(req->flags & LOCKED)) {
            storeQueue[storeWBIdx].inst->setCompleted();
            if (cpu->checker) {
                cpu->checker->tick(storeQueue[storeWBIdx].inst);
            }
        }

        if (dcacheInterface) {
            assert(!req->completionEvent);
            StoreCompletionEvent *store_event = new
                StoreCompletionEvent(storeWBIdx, NULL, this);
            req->completionEvent = store_event;

            MemAccessResult result = dcacheInterface->access(req);

            if (isStalled() &&
                storeQueue[storeWBIdx].inst->seqNum == stallingStoreIsn) {
                DPRINTF(LSQUnit, "Unstalling, stalling store [sn:%lli] "
                        "load idx:%i\n",
                        stallingStoreIsn, stallingLoadIdx);
                stalled = false;
                stallingStoreIsn = 0;
                iewStage->replayMemInst(loadQueue[stallingLoadIdx]);
            }

            typename IEW::LdWritebackEvent *wb = NULL;
            if (req->flags & LOCKED) {
                // Stx_C should not generate a system port transaction
                // if it misses in the cache, but that might be hard
                // to accomplish without explicit cache support.
                wb = new typename
                    IEW::LdWritebackEvent(storeQueue[storeWBIdx].inst,
                                              iewStage);
                store_event->wbEvent = wb;
            }

            if (result != MA_HIT && dcacheInterface->doEvents()) {
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
        } else {
            panic("Must HAVE DCACHE!!!!!\n");
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
        loadQueue[load_idx]->squashed = true;
        loadQueue[load_idx] = NULL;
        --loads;

        // Inefficient!
        loadTail = load_idx;

        decrLdIdx(load_idx);
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
        storeQueue[store_idx].inst->squashed = true;
        storeQueue[store_idx].inst = NULL;
        storeQueue[store_idx].canWB = 0;

        if (storeQueue[store_idx].req) {
            // There should not be a completion event if the store has
            // not yet committed.
            assert(!storeQueue[store_idx].req->completionEvent);
        }

        storeQueue[store_idx].req = NULL;
        --stores;

        // Inefficient!
        storeTail = store_idx;

        decrStIdx(store_idx);
    }
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
    if (cpu->checker) {
        cpu->checker->tick(storeQueue[store_idx].inst);
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
