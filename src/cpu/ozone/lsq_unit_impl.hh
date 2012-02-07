/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 */

#include "base/str.hh"
#include "config/the_isa.hh"
#include "cpu/ozone/lsq_unit.hh"
#include "sim/fault_fwd.hh"

template <class Impl>
OzoneLSQ<Impl>::StoreCompletionEvent::StoreCompletionEvent(int store_idx,
                                                          Event *wb_event,
                                                          OzoneLSQ<Impl> *lsq_ptr)
    : Event(&mainEventQueue),
      storeIdx(store_idx),
      wbEvent(wb_event),
      lsqPtr(lsq_ptr)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
OzoneLSQ<Impl>::StoreCompletionEvent::process()
{
    DPRINTF(OzoneLSQ, "Cache miss complete for store idx:%i\n", storeIdx);

    //lsqPtr->removeMSHR(lsqPtr->storeQueue[storeIdx].inst->seqNum);

//    lsqPtr->cpu->wakeCPU();
    if (wbEvent)
        wbEvent->process();
    lsqPtr->completeStore(storeIdx);
}

template <class Impl>
const char *
OzoneLSQ<Impl>::StoreCompletionEvent::description() const
{
    return "LSQ store completion";
}

template <class Impl>
OzoneLSQ<Impl>::OzoneLSQ()
    : loads(0), stores(0), storesToWB(0), stalled(false), isLoadBlocked(false)
{
}

template<class Impl>
void
OzoneLSQ<Impl>::init(Params *params, unsigned maxLQEntries,
                     unsigned maxSQEntries, unsigned id)

{
    DPRINTF(OzoneLSQ, "Creating OzoneLSQ%i object.\n",id);

    lsqID = id;

    LQEntries = maxLQEntries;
    SQEntries = maxSQEntries;

    loadQueue.resize(LQEntries);
    storeQueue.resize(SQEntries);


    // May want to initialize these entries to NULL

    loadHead = loadTail = 0;

    storeHead = storeWBIdx = storeTail = 0;

    usedPorts = 0;
    cachePorts = params->cachePorts;

    dcacheInterface = params->dcacheInterface;

    loadFaultInst = storeFaultInst = memDepViolator = NULL;
}

template<class Impl>
std::string
OzoneLSQ<Impl>::name() const
{
    return "lsqunit";
}

template<class Impl>
void
OzoneLSQ<Impl>::clearLQ()
{
    loadQueue.clear();
}

template<class Impl>
void
OzoneLSQ<Impl>::clearSQ()
{
    storeQueue.clear();
}

template<class Impl>
void
OzoneLSQ<Impl>::resizeLQ(unsigned size)
{
    assert( size >= LQEntries);

    if (size > LQEntries) {
        while (size > loadQueue.size()) {
            DynInstPtr dummy;
            loadQueue.push_back(dummy);
            LQEntries++;
        }
    } else {
        LQEntries = size;
    }

}

template<class Impl>
void
OzoneLSQ<Impl>::resizeSQ(unsigned size)
{
    if (size > SQEntries) {
        while (size > storeQueue.size()) {
            SQEntry dummy;
            storeQueue.push_back(dummy);
            SQEntries++;
        }
    } else {
        SQEntries = size;
    }
}

template <class Impl>
void
OzoneLSQ<Impl>::insert(DynInstPtr &inst)
{
    // Make sure we really have a memory reference.
    assert(inst->isMemRef());

    // Make sure it's one of the two classes of memory references.
    assert(inst->isLoad() || inst->isStore());

    if (inst->isLoad()) {
        insertLoad(inst);
    } else {
        insertStore(inst);
    }

//    inst->setInLSQ();
}

template <class Impl>
void
OzoneLSQ<Impl>::insertLoad(DynInstPtr &load_inst)
{
    assert((loadTail + 1) % LQEntries != loadHead && loads < LQEntries);

    DPRINTF(OzoneLSQ, "Inserting load PC %#x, idx:%i [sn:%lli]\n",
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
OzoneLSQ<Impl>::insertStore(DynInstPtr &store_inst)
{
    // Make sure it is not full before inserting an instruction.
    assert((storeTail + 1) % SQEntries != storeHead);
    assert(stores < SQEntries);

    DPRINTF(OzoneLSQ, "Inserting store PC %#x, idx:%i [sn:%lli]\n",
            store_inst->readPC(), storeTail, store_inst->seqNum);

    store_inst->sqIdx = storeTail;
    store_inst->lqIdx = loadTail;

    storeQueue[storeTail] = SQEntry(store_inst);

    incrStIdx(storeTail);

    ++stores;

}

template <class Impl>
typename Impl::DynInstPtr
OzoneLSQ<Impl>::getMemDepViolator()
{
    DynInstPtr temp = memDepViolator;

    memDepViolator = NULL;

    return temp;
}

template <class Impl>
unsigned
OzoneLSQ<Impl>::numFreeEntries()
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
OzoneLSQ<Impl>::numLoadsReady()
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

#if 0
template <class Impl>
Fault
OzoneLSQ<Impl>::executeLoad()
{
    Fault load_fault = NoFault;
    DynInstPtr load_inst;

    assert(readyLoads.size() != 0);

    // Execute a ready load.
    LdMapIt ready_it = readyLoads.begin();

    load_inst = (*ready_it).second;

    // Execute the instruction, which is held in the data portion of the
    // iterator.
    load_fault = load_inst->execute();

    // If it executed successfully, then switch it over to the executed
    // loads list.
    if (load_fault == NoFault) {
        executedLoads[load_inst->seqNum] = load_inst;

        readyLoads.erase(ready_it);
    } else {
        loadFaultInst = load_inst;
    }

    return load_fault;
}
#endif

template <class Impl>
Fault
OzoneLSQ<Impl>::executeLoad(DynInstPtr &inst)
{
    // Execute a specific load.
    Fault load_fault = NoFault;

    DPRINTF(OzoneLSQ, "Executing load PC %#x, [sn:%lli]\n",
            inst->readPC(),inst->seqNum);

    // Make sure it's really in the list.
    // Normally it should always be in the list.  However,
    /* due to a syscall it may not be the list.
#ifdef DEBUG
    int i = loadHead;
    while (1) {
        if (i == loadTail && !find(inst)) {
            assert(0 && "Load not in the queue!");
        } else if (loadQueue[i] == inst) {
            break;
        }

        i = i + 1;
        if (i >= LQEntries) {
            i = 0;
        }
    }
#endif // DEBUG*/

    load_fault = inst->initiateAcc();

    // Might want to make sure that I'm not overwriting a previously faulting
    // instruction that hasn't been checked yet.
    // Actually probably want the oldest faulting load
    if (load_fault != NoFault) {
        // Maybe just set it as can commit here, although that might cause
        // some other problems with sending traps to the ROB too quickly.
//        iewStage->instToCommit(inst);
//        iewStage->activityThisCycle();
    }

    return load_fault;
}

template <class Impl>
Fault
OzoneLSQ<Impl>::executeLoad(int lq_idx)
{
    // Very hackish.  Not sure the best way to check that this
    // instruction is at the head of the ROB.  I should have some sort
    // of extra information here so that I'm not overloading the
    // canCommit signal for 15 different things.
    loadQueue[lq_idx]->setCanCommit();
    Fault ret_fault = executeLoad(loadQueue[lq_idx]);
    loadQueue[lq_idx]->clearCanCommit();
    return ret_fault;
}

template <class Impl>
Fault
OzoneLSQ<Impl>::executeStore(DynInstPtr &store_inst)
{
    // Make sure that a store exists.
    assert(stores != 0);

    int store_idx = store_inst->sqIdx;

    DPRINTF(OzoneLSQ, "Executing store PC %#x [sn:%lli]\n",
            store_inst->readPC(), store_inst->seqNum);

    // Check the recently completed loads to see if any match this store's
    // address.  If so, then we have a memory ordering violation.
    int load_idx = store_inst->lqIdx;

    Fault store_fault = store_inst->initiateAcc();

    // Store size should now be available.  Use it to get proper offset for
    // addr comparisons.
    int size = storeQueue[store_idx].size;

    if (size == 0) {
        DPRINTF(OzoneLSQ,"Fault on Store PC %#x, [sn:%lli],Size = 0\n",
                store_inst->readPC(),store_inst->seqNum);

        return store_fault;
    }

    assert(store_fault == NoFault);

    if (!storeFaultInst) {
        if (store_fault != NoFault) {
            panic("Fault in a store instruction!");
            storeFaultInst = store_inst;
        } else if (store_inst->isNonSpeculative()) {
            // Nonspeculative accesses (namely store conditionals)
            // need to set themselves as able to writeback if we
            // haven't had a fault by here.
            storeQueue[store_idx].canWB = true;

            ++storesToWB;
        }
    }

    if (!memDepViolator) {
        while (load_idx != loadTail) {
            // Actually should only check loads that have actually executed
            // Might be safe because effAddr is set to InvalAddr when the
            // dyn inst is created.

            // Must actually check all addrs in the proper size range
            // Which is more correct than needs to be.  What if for now we just
            // assume all loads are quad-word loads, and do the addr based
            // on that.
            // @todo: Fix this, magic number being used here
            if ((loadQueue[load_idx]->effAddr >> 8) ==
                (store_inst->effAddr >> 8)) {
                // A load incorrectly passed this store.  Squash and refetch.
                // For now return a fault to show that it was unsuccessful.
                memDepViolator = loadQueue[load_idx];

                return TheISA::genMachineCheckFault();
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
OzoneLSQ<Impl>::commitLoad()
{
    assert(loadQueue[loadHead]);

    DPRINTF(OzoneLSQ, "[sn:%lli] Committing head load instruction, PC %#x\n",
            loadQueue[loadHead]->seqNum, loadQueue[loadHead]->readPC());


    loadQueue[loadHead] = NULL;

    incrLdIdx(loadHead);

    --loads;
}

template <class Impl>
void
OzoneLSQ<Impl>::commitLoad(InstSeqNum &inst)
{
    // Hopefully I don't use this function too much
    panic("Don't use this function!");

    int i = loadHead;
    while (1) {
        if (i == loadTail) {
            assert(0 && "Load not in the queue!");
        } else if (loadQueue[i]->seqNum == inst) {
            break;
        }

        ++i;
        if (i >= LQEntries) {
            i = 0;
        }
    }

//    loadQueue[i]->removeInLSQ();
    loadQueue[i] = NULL;
    --loads;
}

template <class Impl>
void
OzoneLSQ<Impl>::commitLoads(InstSeqNum &youngest_inst)
{
    assert(loads == 0 || loadQueue[loadHead]);

    while (loads != 0 && loadQueue[loadHead]->seqNum <= youngest_inst) {
        commitLoad();
    }
}

template <class Impl>
void
OzoneLSQ<Impl>::commitStores(InstSeqNum &youngest_inst)
{
    assert(stores == 0 || storeQueue[storeHead].inst);

    int store_idx = storeHead;

    while (store_idx != storeTail) {
        assert(storeQueue[store_idx].inst);
        if (!storeQueue[store_idx].canWB) {
            if (storeQueue[store_idx].inst->seqNum > youngest_inst) {
                break;
            }
            DPRINTF(OzoneLSQ, "Marking store as able to write back, PC "
                    "%#x [sn:%lli]\n",
                    storeQueue[store_idx].inst->readPC(),
                    storeQueue[store_idx].inst->seqNum);

            storeQueue[store_idx].canWB = true;

//            --stores;
            ++storesToWB;
        }

        incrStIdx(store_idx);
    }
}

template <class Impl>
void
OzoneLSQ<Impl>::writebackStores()
{
    while (storesToWB > 0 &&
           storeWBIdx != storeTail &&
           storeQueue[storeWBIdx].inst &&
           storeQueue[storeWBIdx].canWB &&
           usedPorts < cachePorts) {

        if (storeQueue[storeWBIdx].size == 0) {
            completeStore(storeWBIdx);

            incrStIdx(storeWBIdx);

            continue;
        }

        if (dcacheInterface && dcacheInterface->isBlocked()) {
            DPRINTF(OzoneLSQ, "Unable to write back any more stores, cache"
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

//      Fault fault = cpu->translateDataReadReq(req);
        req->cmd = Write;
        req->completionEvent = NULL;
        req->time = curTick();
        assert(!req->data);
        req->data = new uint8_t[64];
        memcpy(req->data, (uint8_t *)&storeQueue[storeWBIdx].data, req->size);

        DPRINTF(OzoneLSQ, "D-Cache: Writing back store idx:%i PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                storeWBIdx,storeQueue[storeWBIdx].inst->readPC(),
                req->paddr, *(req->data),
                storeQueue[storeWBIdx].inst->seqNum);

//        if (fault != NoFault) {
            //What should we do if there is a fault???
            //for now panic
//            panic("Page Table Fault!!!!!\n");
//        }

        if (dcacheInterface) {
            MemAccessResult result = dcacheInterface->access(req);

            //@todo temp fix for LL/SC (works fine for 1 CPU)
            if (req->isLLSC()) {
                req->result=1;
                panic("LL/SC! oh no no support!!!");
            }

            if (isStalled() &&
                storeQueue[storeWBIdx].inst->seqNum == stallingStoreIsn) {
                DPRINTF(OzoneLSQ, "Unstalling, stalling store [sn:%lli] "
                        "load idx:%i\n",
                        stallingStoreIsn, stallingLoadIdx);
                stalled = false;
                stallingStoreIsn = 0;
                be->replayMemInst(loadQueue[stallingLoadIdx]);
            }

            if (result != MA_HIT && dcacheInterface->doEvents()) {
                Event *wb = NULL;
/*
                typename IEW::LdWritebackEvent *wb = NULL;
                if (req->isLLSC()) {
                    // Stx_C does not generate a system port transaction.
                    req->result=0;
                    wb = new typename IEW::LdWritebackEvent(storeQueue[storeWBIdx].inst,
                                                            iewStage);
                }
*/
                DPRINTF(OzoneLSQ,"D-Cache Write Miss!\n");

//                DPRINTF(Activity, "Active st accessing mem miss [sn:%lli]\n",
//                        storeQueue[storeWBIdx].inst->seqNum);

                // Will stores need their own kind of writeback events?
                // Do stores even need writeback events?
                assert(!req->completionEvent);
                req->completionEvent = new
                    StoreCompletionEvent(storeWBIdx, wb, this);

                lastDcacheStall = curTick();

                _status = DcacheMissStall;

                //mshrSeqNums.push_back(storeQueue[storeWBIdx].inst->seqNum);

                //DPRINTF(OzoneLSQ, "Added MSHR. count = %i\n",mshrSeqNums.size());

                // Increment stat here or something
            } else {
                DPRINTF(OzoneLSQ,"D-Cache: Write Hit on idx:%i !\n",
                        storeWBIdx);

//                DPRINTF(Activity, "Active st accessing mem hit [sn:%lli]\n",
//                        storeQueue[storeWBIdx].inst->seqNum);

                if (req->isLLSC()) {
                    // Stx_C does not generate a system port transaction.
                    req->result=1;
                    typename BackEnd::LdWritebackEvent *wb =
                        new typename BackEnd::LdWritebackEvent(storeQueue[storeWBIdx].inst,
                                                               be);
                    wb->schedule(curTick());
                }

                completeStore(storeWBIdx);
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
OzoneLSQ<Impl>::removeMSHR(InstSeqNum seqNum)
{
    list<InstSeqNum>::iterator mshr_it = find(mshrSeqNums.begin(),
                                              mshrSeqNums.end(),
                                              seqNum);

    if (mshr_it != mshrSeqNums.end()) {
        mshrSeqNums.erase(mshr_it);
        DPRINTF(OzoneLSQ, "Removing MSHR. count = %i\n",mshrSeqNums.size());
    }
}*/

template <class Impl>
void
OzoneLSQ<Impl>::squash(const InstSeqNum &squashed_num)
{
    DPRINTF(OzoneLSQ, "Squashing until [sn:%lli]!"
            "(Loads:%i Stores:%i)\n",squashed_num,loads,stores);

    int load_idx = loadTail;
    decrLdIdx(load_idx);

    while (loads != 0 && loadQueue[load_idx]->seqNum > squashed_num) {

        // Clear the smart pointer to make sure it is decremented.
        DPRINTF(OzoneLSQ,"Load Instruction PC %#x squashed, "
                "[sn:%lli]\n",
                loadQueue[load_idx]->readPC(),
                loadQueue[load_idx]->seqNum);

        if (isStalled() && load_idx == stallingLoadIdx) {
            stalled = false;
            stallingStoreIsn = 0;
            stallingLoadIdx = 0;
        }

//        loadQueue[load_idx]->squashed = true;
        loadQueue[load_idx] = NULL;
        --loads;

        // Inefficient!
        loadTail = load_idx;

        decrLdIdx(load_idx);
    }

    int store_idx = storeTail;
    decrStIdx(store_idx);

    while (stores != 0 && storeQueue[store_idx].inst->seqNum > squashed_num) {

        // Clear the smart pointer to make sure it is decremented.
        DPRINTF(OzoneLSQ,"Store Instruction PC %#x squashed, "
                "idx:%i [sn:%lli]\n",
                storeQueue[store_idx].inst->readPC(),
                store_idx, storeQueue[store_idx].inst->seqNum);

        // I don't think this can happen.  It should have been cleared by the
        // stalling load.
        if (isStalled() &&
            storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
            panic("Is stalled should have been cleared by stalling load!\n");
            stalled = false;
            stallingStoreIsn = 0;
        }

//        storeQueue[store_idx].inst->squashed = true;
        storeQueue[store_idx].inst = NULL;
        storeQueue[store_idx].canWB = 0;

        if (storeQueue[store_idx].req) {
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
OzoneLSQ<Impl>::dumpInsts()
{
    cprintf("Load store queue: Dumping instructions.\n");
    cprintf("Load queue size: %i\n", loads);
    cprintf("Load queue: ");

    int load_idx = loadHead;

    while (load_idx != loadTail && loadQueue[load_idx]) {
        cprintf("[sn:%lli] %#x ", loadQueue[load_idx]->seqNum,
                loadQueue[load_idx]->readPC());

        incrLdIdx(load_idx);
    }

    cprintf("\nStore queue size: %i\n", stores);
    cprintf("Store queue: ");

    int store_idx = storeHead;

    while (store_idx != storeTail && storeQueue[store_idx].inst) {
        cprintf("[sn:%lli] %#x ", storeQueue[store_idx].inst->seqNum,
                storeQueue[store_idx].inst->readPC());

        incrStIdx(store_idx);
    }

    cprintf("\n");
}

template <class Impl>
void
OzoneLSQ<Impl>::completeStore(int store_idx)
{
    assert(storeQueue[store_idx].inst);
    storeQueue[store_idx].completed = true;
    --storesToWB;
    // A bit conservative because a store completion may not free up entries,
    // but hopefully avoids two store completions in one cycle from making
    // the CPU tick twice.
//    cpu->activityThisCycle();

    if (store_idx == storeHead) {
        do {
            incrStIdx(storeHead);

            --stores;
        } while (storeQueue[storeHead].completed &&
                 storeHead != storeTail);

//        be->updateLSQNextCycle = true;
    }

    DPRINTF(OzoneLSQ, "Store head idx:%i\n", storeHead);

    if (isStalled() &&
        storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
        DPRINTF(OzoneLSQ, "Unstalling, stalling store [sn:%lli] "
                "load idx:%i\n",
                stallingStoreIsn, stallingLoadIdx);
        stalled = false;
        stallingStoreIsn = 0;
        be->replayMemInst(loadQueue[stallingLoadIdx]);
    }
}

template <class Impl>
inline void
OzoneLSQ<Impl>::incrStIdx(int &store_idx)
{
    if (++store_idx >= SQEntries)
        store_idx = 0;
}

template <class Impl>
inline void
OzoneLSQ<Impl>::decrStIdx(int &store_idx)
{
    if (--store_idx < 0)
        store_idx += SQEntries;
}

template <class Impl>
inline void
OzoneLSQ<Impl>::incrLdIdx(int &load_idx)
{
    if (++load_idx >= LQEntries)
        load_idx = 0;
}

template <class Impl>
inline void
OzoneLSQ<Impl>::decrLdIdx(int &load_idx)
{
    if (--load_idx < 0)
        load_idx += LQEntries;
}
