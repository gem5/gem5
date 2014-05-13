/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __CPU_OZONE_LW_LSQ_IMPL_HH__
#define __CPU_OZONE_LW_LSQ_IMPL_HH__

#include "base/str.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/ozone/lw_lsq.hh"
#include "sim/fault_fwd.hh"

template<class Impl>
OzoneLWLSQ<Impl>::WritebackEvent::WritebackEvent(DynInstPtr &_inst, PacketPtr _pkt,
                                                 OzoneLWLSQ *lsq_ptr)
    : Event(&mainEventQueue), inst(_inst), pkt(_pkt), lsqPtr(lsq_ptr)
{
    this->setFlags(Event::AutoDelete);
}

template<class Impl>
void
OzoneLWLSQ<Impl>::WritebackEvent::process()
{
    if (!lsqPtr->isSwitchedOut()) {
        lsqPtr->writeback(inst, pkt);
    }
    delete pkt;
}

template<class Impl>
const char *
OzoneLWLSQ<Impl>::WritebackEvent::description() const
{
    return "Store writeback";
}

template <class Impl>
Tick
OzoneLWLSQ<Impl>::DcachePort::recvAtomic(PacketPtr pkt)
{
    panic("O3CPU model does not work with atomic mode!");
    return curTick();
}

template <class Impl>
void
OzoneLWLSQ<Impl>::DcachePort::recvFunctional(PacketPtr pkt)
{
    warn("O3CPU doesn't update things on a recvFunctional");
}

template <class Impl>
bool
OzoneLWLSQ<Impl>::DcachePort::recvTiming(PacketPtr pkt)
{
    lsq->completeDataAccess(pkt);
    return true;
}

template <class Impl>
void
OzoneLWLSQ<Impl>::DcachePort::recvRetry()
{
    lsq->recvRetry();
}

template<class Impl>
void
OzoneLWLSQ<Impl>::completeDataAccess(PacketPtr pkt)
{
    LSQSenderState *state = dynamic_cast<LSQSenderState *>(pkt->senderState);
    DynInstPtr inst = state->inst;
    DPRINTF(IEW, "Writeback event [sn:%lli]\n", inst->seqNum);
    DPRINTF(Activity, "Activity: Writeback event [sn:%lli]\n", inst->seqNum);

    //iewStage->ldstQueue.removeMSHR(inst->threadNumber,inst->seqNum);

    if (isSwitchedOut() || inst->isSquashed()) {
        delete state;
        delete pkt;
        return;
    } else {
        if (!state->noWB) {
            writeback(inst, pkt);
        }

        if (inst->isStore()) {
            completeStore(inst);
        }
    }

    delete state;
    delete pkt;
}

template <class Impl>
OzoneLWLSQ<Impl>::OzoneLWLSQ()
    : switchedOut(false), dcachePort(this), loads(0), stores(0),
      storesToWB(0), storesInFlight(0), stalled(false), isStoreBlocked(false),
      isLoadBlocked(false), loadBlockedHandled(false)
{
}

template<class Impl>
void
OzoneLWLSQ<Impl>::init(Params *params, unsigned maxLQEntries,
                       unsigned maxSQEntries, unsigned id)
{
    DPRINTF(OzoneLSQ, "Creating OzoneLWLSQ%i object.\n",id);

    lsqID = id;

    LQEntries = maxLQEntries;
    SQEntries = maxSQEntries;

    for (int i = 0; i < LQEntries * 2; i++) {
        LQIndices.push(i);
        SQIndices.push(i);
    }

    usedPorts = 0;
    cachePorts = params->cachePorts;

    loadFaultInst = storeFaultInst = memDepViolator = NULL;

    blockedLoadSeqNum = 0;
}

template<class Impl>
std::string
OzoneLWLSQ<Impl>::name() const
{
    return "lsqunit";
}

template<class Impl>
void
OzoneLWLSQ<Impl>::regStats()
{
    lsqMemOrderViolation
        .name(name() + ".memOrderViolation")
        .desc("Number of memory ordering violations");
}

template<class Impl>
void
OzoneLWLSQ<Impl>::setCPU(OzoneCPU *cpu_ptr)
{
    cpu = cpu_ptr;
    dcachePort.setName(this->name() + "-dport");

    if (cpu->checker) {
        cpu->checker->setDcachePort(&dcachePort);
    }
}

template<class Impl>
void
OzoneLWLSQ<Impl>::clearLQ()
{
    loadQueue.clear();
}

template<class Impl>
void
OzoneLWLSQ<Impl>::clearSQ()
{
    storeQueue.clear();
}
/*
template<class Impl>
void
OzoneLWLSQ<Impl>::setPageTable(PageTable *pt_ptr)
{
    DPRINTF(OzoneLSQ, "Setting the page table pointer.\n");
    pTable = pt_ptr;
}
*/
template<class Impl>
void
OzoneLWLSQ<Impl>::resizeLQ(unsigned size)
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
OzoneLWLSQ<Impl>::resizeSQ(unsigned size)
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
OzoneLWLSQ<Impl>::insert(DynInstPtr &inst)
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
}

template <class Impl>
void
OzoneLWLSQ<Impl>::insertLoad(DynInstPtr &load_inst)
{
    assert(loads < LQEntries * 2);
    assert(!LQIndices.empty());
    int load_index = LQIndices.front();
    LQIndices.pop();

    DPRINTF(OzoneLSQ, "Inserting load PC %#x, idx:%i [sn:%lli]\n",
            load_inst->readPC(), load_index, load_inst->seqNum);

    load_inst->lqIdx = load_index;

    loadQueue.push_front(load_inst);
    LQItHash[load_index] = loadQueue.begin();

    ++loads;
}

template <class Impl>
void
OzoneLWLSQ<Impl>::insertStore(DynInstPtr &store_inst)
{
    // Make sure it is not full before inserting an instruction.
    assert(stores - storesToWB < SQEntries);

    assert(!SQIndices.empty());
    int store_index = SQIndices.front();
    SQIndices.pop();

    DPRINTF(OzoneLSQ, "Inserting store PC %#x, idx:%i [sn:%lli]\n",
            store_inst->readPC(), store_index, store_inst->seqNum);

    store_inst->sqIdx = store_index;
    SQEntry entry(store_inst);
    if (loadQueue.empty()) {
        entry.lqIt = loadQueue.end();
    } else {
        entry.lqIt = loadQueue.begin();
    }
    storeQueue.push_front(entry);

    SQItHash[store_index] = storeQueue.begin();

    ++stores;
}

template <class Impl>
typename Impl::DynInstPtr
OzoneLWLSQ<Impl>::getMemDepViolator()
{
    DynInstPtr temp = memDepViolator;

    memDepViolator = NULL;

    return temp;
}

template <class Impl>
unsigned
OzoneLWLSQ<Impl>::numFreeEntries()
{
    unsigned free_lq_entries = LQEntries - loads;
    unsigned free_sq_entries = SQEntries - (stores + storesInFlight);

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
OzoneLWLSQ<Impl>::numLoadsReady()
{
    int retval = 0;
    LQIt lq_it = loadQueue.begin();
    LQIt end_it = loadQueue.end();

    while (lq_it != end_it) {
        if ((*lq_it)->readyToIssue()) {
            ++retval;
        }
    }

    return retval;
}

template <class Impl>
Fault
OzoneLWLSQ<Impl>::executeLoad(DynInstPtr &inst)
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
        DPRINTF(OzoneLSQ, "Load [sn:%lli] has a fault\n", inst->seqNum);
        if (!(inst->req->isUncacheable() && !inst->isAtCommit())) {
            inst->setExecuted();
        }
        // Maybe just set it as can commit here, although that might cause
        // some other problems with sending traps to the ROB too quickly.
        be->instToCommit(inst);
//        iewStage->activityThisCycle();
    }

    return load_fault;
}

template <class Impl>
Fault
OzoneLWLSQ<Impl>::executeStore(DynInstPtr &store_inst)
{
    // Make sure that a store exists.
    assert(stores != 0);

    int store_idx = store_inst->sqIdx;
    SQHashIt sq_hash_it = SQItHash.find(store_idx);
    assert(sq_hash_it != SQItHash.end());
    DPRINTF(OzoneLSQ, "Executing store PC %#x [sn:%lli]\n",
            store_inst->readPC(), store_inst->seqNum);

    SQIt sq_it = (*sq_hash_it).second;

    Fault store_fault = store_inst->initiateAcc();

    // Store size should now be available.  Use it to get proper offset for
    // addr comparisons.
    int size = (*sq_it).size;

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
        } else if (store_inst->isStoreConditional()) {
            // Store conditionals need to set themselves as able to
            // writeback if we haven't had a fault by here.
            (*sq_it).canWB = true;

            ++storesToWB;
            DPRINTF(OzoneLSQ, "Nonspeculative store! storesToWB:%i\n",
                    storesToWB);
        }
    }

    LQIt lq_it = --(loadQueue.end());

    if (!memDepViolator) {
        while (lq_it != loadQueue.end()) {
            if ((*lq_it)->seqNum < store_inst->seqNum) {
                lq_it--;
                continue;
            }
            // Actually should only check loads that have actually executed
            // Might be safe because effAddr is set to InvalAddr when the
            // dyn inst is created.

            // Must actually check all addrs in the proper size range
            // Which is more correct than needs to be.  What if for now we just
            // assume all loads are quad-word loads, and do the addr based
            // on that.
            // @todo: Fix this, magic number being used here
            if (((*lq_it)->effAddr >> 8) ==
                (store_inst->effAddr >> 8)) {
                // A load incorrectly passed this store.  Squash and refetch.
                // For now return a fault to show that it was unsuccessful.
                memDepViolator = (*lq_it);
                ++lsqMemOrderViolation;

                return TheISA::genMachineCheckFault();
            }

            lq_it--;
        }

        // If we've reached this point, there was no violation.
        memDepViolator = NULL;
    }

    return store_fault;
}

template <class Impl>
void
OzoneLWLSQ<Impl>::commitLoad()
{
    assert(!loadQueue.empty());

    DPRINTF(OzoneLSQ, "[sn:%lli] Committing head load instruction, PC %#x\n",
            loadQueue.back()->seqNum, loadQueue.back()->readPC());

    LQIndices.push(loadQueue.back()->lqIdx);
    LQItHash.erase(loadQueue.back()->lqIdx);

    loadQueue.pop_back();

    --loads;
}

template <class Impl>
void
OzoneLWLSQ<Impl>::commitLoads(InstSeqNum &youngest_inst)
{
    assert(loads == 0 || !loadQueue.empty());

    while (loads != 0 &&
           loadQueue.back()->seqNum <= youngest_inst) {
        commitLoad();
    }
}

template <class Impl>
void
OzoneLWLSQ<Impl>::commitStores(InstSeqNum &youngest_inst)
{
    assert(stores == 0 || !storeQueue.empty());

    SQIt sq_it = --(storeQueue.end());
    while (!storeQueue.empty() && sq_it != storeQueue.end()) {
        assert((*sq_it).inst);
        if (!(*sq_it).canWB) {
            if ((*sq_it).inst->seqNum > youngest_inst) {
                break;
            }
            ++storesToWB;

            DPRINTF(OzoneLSQ, "Marking store as able to write back, PC "
                    "%#x [sn:%lli], storesToWB:%i\n",
                    (*sq_it).inst->readPC(),
                    (*sq_it).inst->seqNum,
                    storesToWB);

            (*sq_it).canWB = true;
        }

        sq_it--;
    }
}

template <class Impl>
void
OzoneLWLSQ<Impl>::writebackStores()
{
    SQIt sq_it = --(storeQueue.end());
    while (storesToWB > 0 &&
           sq_it != storeQueue.end() &&
           (*sq_it).inst &&
           (*sq_it).canWB &&
           usedPorts < cachePorts) {

        if (isStoreBlocked) {
            DPRINTF(OzoneLSQ, "Unable to write back any more stores, cache"
                    " is blocked!\n");
            break;
        }

        DynInstPtr inst = (*sq_it).inst;

        if ((*sq_it).size == 0 && !(*sq_it).completed) {
            sq_it--;
            removeStore(inst->sqIdx);
            completeStore(inst);
            continue;
        }

        if (inst->isDataPrefetch() || (*sq_it).committed) {
            sq_it--;
            continue;
        }

        ++usedPorts;

        assert((*sq_it).req);
        assert(!(*sq_it).committed);

        Request *req = (*sq_it).req;
        (*sq_it).committed = true;

        assert(!inst->memData);
        inst->memData = new uint8_t[64];
        memcpy(inst->memData, (uint8_t *)&(*sq_it).data,
               req->getSize());

        PacketPtr data_pkt = Packet::createWrite(req);
        data_pkt->dataStatic(inst->memData);

        LSQSenderState *state = new LSQSenderState;
        state->isLoad = false;
        state->idx = inst->sqIdx;
        state->inst = inst;
        data_pkt->senderState = state;

        DPRINTF(OzoneLSQ, "D-Cache: Writing back store PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                (*sq_it).inst->readPC(),
                req->getPaddr(), *(inst->memData),
                inst->seqNum);

        // @todo: Remove this SC hack once the memory system handles it.
        if (req->isLLSC()) {
            if (req->isUncacheable()) {
                req->setExtraData(2);
            } else {
                if (cpu->lockFlag) {
                    req->setExtraData(1);
                } else {
                    req->setExtraData(0);
                    // Hack: Instantly complete this store.
                    completeDataAccess(data_pkt);
                    --sq_it;
                    continue;
                }
            }
        } else {
            // Non-store conditionals do not need a writeback.
            state->noWB = true;
        }

        if (!dcachePort.sendTiming(data_pkt)) {
            // Need to handle becoming blocked on a store.
            isStoreBlocked = true;
            assert(retryPkt == NULL);
            retryPkt = data_pkt;
        } else {
            storePostSend(data_pkt, inst);
            --sq_it;
        }
/*
        DPRINTF(OzoneLSQ, "D-Cache: Writing back store idx:%i PC:%#x "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                inst->sqIdx,inst->readPC(),
                req->paddr, *(req->data),
                inst->seqNum);
        DPRINTF(OzoneLSQ, "StoresInFlight: %i\n",
                storesInFlight + 1);

        if (dcacheInterface) {
            assert(!req->completionEvent);
            StoreCompletionEvent *store_event = new
                StoreCompletionEvent(inst, be, NULL, this);
            req->completionEvent = store_event;

            MemAccessResult result = dcacheInterface->access(req);

            if (isStalled() &&
                inst->seqNum == stallingStoreIsn) {
                DPRINTF(OzoneLSQ, "Unstalling, stalling store [sn:%lli] "
                        "load [sn:%lli]\n",
                        stallingStoreIsn, (*stallingLoad)->seqNum);
                stalled = false;
                stallingStoreIsn = 0;
                be->replayMemInst((*stallingLoad));
            }

            if (result != MA_HIT && dcacheInterface->doEvents()) {
                store_event->miss = true;
                typename BackEnd::LdWritebackEvent *wb = NULL;
                if (req->isLLSC()) {
                    wb = new typename BackEnd::LdWritebackEvent(inst,
                                                            be);
                    store_event->wbEvent = wb;
                }

                DPRINTF(OzoneLSQ,"D-Cache Write Miss!\n");

//                DPRINTF(Activity, "Active st accessing mem miss [sn:%lli]\n",
//                        inst->seqNum);

                be->addDcacheMiss(inst);

                lastDcacheStall = curTick();

                _status = DcacheMissStall;

                // Increment stat here or something

                sq_it--;
            } else {
                DPRINTF(OzoneLSQ,"D-Cache: Write Hit on idx:%i !\n",
                        inst->sqIdx);

//                DPRINTF(Activity, "Active st accessing mem hit [sn:%lli]\n",
//                        inst->seqNum);

                if (req->isLLSC()) {
                    // Stx_C does not generate a system port
                    // transaction in the 21264, but that might be
                    // hard to accomplish in this model.

                    typename BackEnd::LdWritebackEvent *wb =
                        new typename BackEnd::LdWritebackEvent(inst,
                                                               be);
                    store_event->wbEvent = wb;
                }
                sq_it--;
            }
            ++storesInFlight;
//            removeStore(inst->sqIdx);
        } else {
            panic("Must HAVE DCACHE!!!!!\n");
        }
*/
    }

    // Not sure this should set it to 0.
    usedPorts = 0;

    assert(stores >= 0 && storesToWB >= 0);
}

template <class Impl>
void
OzoneLWLSQ<Impl>::squash(const InstSeqNum &squashed_num)
{
    DPRINTF(OzoneLSQ, "Squashing until [sn:%lli]!"
            "(Loads:%i Stores:%i)\n",squashed_num,loads,stores+storesInFlight);


    LQIt lq_it = loadQueue.begin();

    while (loads != 0 && (*lq_it)->seqNum > squashed_num) {
        assert(!loadQueue.empty());
        // Clear the smart pointer to make sure it is decremented.
        DPRINTF(OzoneLSQ,"Load Instruction PC %#x squashed, "
                "[sn:%lli]\n",
                (*lq_it)->readPC(),
                (*lq_it)->seqNum);

        if (isStalled() && lq_it == stallingLoad) {
            stalled = false;
            stallingStoreIsn = 0;
            stallingLoad = NULL;
        }

        --loads;

        // Inefficient!
        LQHashIt lq_hash_it = LQItHash.find((*lq_it)->lqIdx);
        assert(lq_hash_it != LQItHash.end());
        LQItHash.erase(lq_hash_it);
        LQIndices.push((*lq_it)->lqIdx);
        loadQueue.erase(lq_it++);
    }

    if (isLoadBlocked) {
        if (squashed_num < blockedLoadSeqNum) {
            isLoadBlocked = false;
            loadBlockedHandled = false;
            blockedLoadSeqNum = 0;
        }
    }

    SQIt sq_it = storeQueue.begin();

    while (stores != 0 && (*sq_it).inst->seqNum > squashed_num) {
        assert(!storeQueue.empty());

        if ((*sq_it).canWB) {
            break;
        }

        // Clear the smart pointer to make sure it is decremented.
        DPRINTF(OzoneLSQ,"Store Instruction PC %#x idx:%i squashed [sn:%lli]\n",
                (*sq_it).inst->readPC(), (*sq_it).inst->sqIdx,
                (*sq_it).inst->seqNum);

        // I don't think this can happen.  It should have been cleared by the
        // stalling load.
        if (isStalled() &&
            (*sq_it).inst->seqNum == stallingStoreIsn) {
            panic("Is stalled should have been cleared by stalling load!\n");
            stalled = false;
            stallingStoreIsn = 0;
        }

        SQHashIt sq_hash_it = SQItHash.find((*sq_it).inst->sqIdx);
        assert(sq_hash_it != SQItHash.end());
        SQItHash.erase(sq_hash_it);
        SQIndices.push((*sq_it).inst->sqIdx);
        (*sq_it).inst = NULL;
        (*sq_it).canWB = 0;
        (*sq_it).req = NULL;
        --stores;
        storeQueue.erase(sq_it++);
    }
}

template <class Impl>
void
OzoneLWLSQ<Impl>::dumpInsts()
{
    cprintf("Load store queue: Dumping instructions.\n");
    cprintf("Load queue size: %i\n", loads);
    cprintf("Load queue: ");

    LQIt lq_it = --(loadQueue.end());

    while (lq_it != loadQueue.end() && (*lq_it)) {
        cprintf("[sn:%lli] %#x ", (*lq_it)->seqNum,
                (*lq_it)->readPC());

        lq_it--;
    }

    cprintf("\nStore queue size: %i\n", stores);
    cprintf("Store queue: ");

    SQIt sq_it = --(storeQueue.end());

    while (sq_it != storeQueue.end() && (*sq_it).inst) {
        cprintf("[sn:%lli]\nPC:%#x\nSize:%i\nCommitted:%i\nCompleted:%i\ncanWB:%i\n",
                (*sq_it).inst->seqNum,
                (*sq_it).inst->readPC(),
                (*sq_it).size,
                (*sq_it).committed,
                (*sq_it).completed,
                (*sq_it).canWB);

        sq_it--;
    }

    cprintf("\n");
}

template <class Impl>
void
OzoneLWLSQ<Impl>::storePostSend(PacketPtr pkt, DynInstPtr &inst)
{
    if (isStalled() &&
        inst->seqNum == stallingStoreIsn) {
        DPRINTF(OzoneLSQ, "Unstalling, stalling store [sn:%lli] "
                "load [sn:%lli]\n",
                stallingStoreIsn, (*stallingLoad)->seqNum);
        stalled = false;
        stallingStoreIsn = 0;
        be->replayMemInst((*stallingLoad));
    }

    if (!inst->isStoreConditional()) {
        // The store is basically completed at this time. This
        // only works so long as the checker doesn't try to
        // verify the value in memory for stores.
        inst->setCompleted();
        if (cpu->checker) {
            cpu->checker->verify(inst);
        }
    }
}

template <class Impl>
void
OzoneLWLSQ<Impl>::writeback(DynInstPtr &inst, PacketPtr pkt)
{
    // Squashed instructions do not need to complete their access.
    if (inst->isSquashed()) {
        assert(!inst->isStore());
        return;
    }

    if (!inst->isExecuted()) {
        inst->setExecuted();

        // Complete access to copy data to proper place.
        inst->completeAcc(pkt);
    }

    // Need to insert instruction into queue to commit
    be->instToCommit(inst);
}

template <class Impl>
void
OzoneLWLSQ<Impl>::removeStore(int store_idx)
{
    SQHashIt sq_hash_it = SQItHash.find(store_idx);
    assert(sq_hash_it != SQItHash.end());
    SQIt sq_it = (*sq_hash_it).second;

    assert((*sq_it).inst);
    (*sq_it).completed = true;
    DynInstPtr inst = (*sq_it).inst;

    if (isStalled() &&
        inst->seqNum == stallingStoreIsn) {
        DPRINTF(OzoneLSQ, "Unstalling, stalling store [sn:%lli] "
                "load [sn:%lli]\n",
                stallingStoreIsn, (*stallingLoad)->seqNum);
        stalled = false;
        stallingStoreIsn = 0;
        be->replayMemInst((*stallingLoad));
    }

    DPRINTF(OzoneLSQ, "Completing store idx:%i [sn:%lli], storesToWB:%i\n",
            inst->sqIdx, inst->seqNum, storesToWB);

    assert(!storeQueue.empty());
    SQItHash.erase(sq_hash_it);
    SQIndices.push(inst->sqIdx);
    storeQueue.erase(sq_it);
}

template <class Impl>
void
OzoneLWLSQ<Impl>::completeStore(DynInstPtr &inst)
{
    --storesToWB;
    --stores;

    inst->setCompleted();
    if (cpu->checker) {
        cpu->checker->verify(inst);
    }
}

template <class Impl>
void
OzoneLWLSQ<Impl>::recvRetry()
{
    panic("Unimplemented!");
}

template <class Impl>
void
OzoneLWLSQ<Impl>::switchOut()
{
    assert(storesToWB == 0);
    switchedOut = true;

    // Clear the queue to free up resources
    assert(stores == 0);
    assert(storeQueue.empty());
    assert(loads == 0);
    assert(loadQueue.empty());
    assert(storesInFlight == 0);
    storeQueue.clear();
    loadQueue.clear();
    loads = stores = storesToWB = storesInFlight = 0;
}

template <class Impl>
void
OzoneLWLSQ<Impl>::takeOverFrom(ThreadContext *old_tc)
{
    // Clear out any old state. May be redundant if this is the first time
    // the CPU is being used.
    stalled = false;
    isLoadBlocked = false;
    loadBlockedHandled = false;
    switchedOut = false;

    // Could do simple checks here to see if indices are on twice
    while (!LQIndices.empty())
        LQIndices.pop();
    while (!SQIndices.empty())
        SQIndices.pop();

    for (int i = 0; i < LQEntries * 2; i++) {
        LQIndices.push(i);
        SQIndices.push(i);
    }

    usedPorts = 0;

    loadFaultInst = storeFaultInst = memDepViolator = NULL;

    blockedLoadSeqNum = 0;
}

#endif//__CPU_OZONE_LW_LSQ_IMPL_HH__
