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

#ifndef __CPU_OZONE_INORDER_BACK_END_IMPL_HH__
#define __CPU_OZONE_INORDER_BACK_END_IMPL_HH__

#include "arch/types.hh"
#include "config/the_isa.hh"
#include "cpu/ozone/inorder_back_end.hh"
#include "cpu/ozone/thread_state.hh"
#include "sim/faults.hh"

template <class Impl>
InorderBackEnd<Impl>::InorderBackEnd(Params *params)
    : squashPending(false),
      squashSeqNum(0),
      squashNextPC(0),
      faultFromFetch(NoFault),
      interruptBlocked(false),
      cacheCompletionEvent(this),
      dcacheInterface(params->dcacheInterface),
      width(params->backEndWidth),
      latency(params->backEndLatency),
      squashLatency(params->backEndSquashLatency),
      numInstsToWB(0, latency + 1)
{
    instsAdded = numInstsToWB.getWire(latency);
    instsToExecute = numInstsToWB.getWire(0);

    memReq = new MemReq;
    memReq->data = new uint8_t[64];
    status = Running;
}

template <class Impl>
std::string
InorderBackEnd<Impl>::name() const
{
    return cpu->name() + ".inorderbackend";
}

template <class Impl>
void
InorderBackEnd<Impl>::setXC(ExecContext *xc_ptr)
{
    xc = xc_ptr;
    memReq->xc = xc;
}

template <class Impl>
void
InorderBackEnd<Impl>::setThreadState(OzoneThreadState<Impl> *thread_ptr)
{
    thread = thread_ptr;
    thread->setFuncExeInst(0);
}

template <class Impl>
void
InorderBackEnd<Impl>::checkInterrupts()
{
    //Check if there are any outstanding interrupts
    //Handle the interrupts
    int ipl = 0;
    int summary = 0;


    if (thread->readMiscRegNoEffect(IPR_ASTRR))
        panic("asynchronous traps not implemented\n");

    if (thread->readMiscRegNoEffect(IPR_SIRR)) {
        for (int i = INTLEVEL_SOFTWARE_MIN;
             i < INTLEVEL_SOFTWARE_MAX; i++) {
            if (thread->readMiscRegNoEffect(IPR_SIRR) & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                summary |= (ULL(1) << i);
            }
        }
    }

    uint64_t interrupts = cpu->intr_status();

    if (interrupts) {
        for (int i = INTLEVEL_EXTERNAL_MIN;
             i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }
    }

    if (ipl && ipl > thread->readMiscRegNoEffect(IPR_IPLR)) {
        thread->noSquashFromTC = true;

        thread->setMiscRegNoEffect(IPR_ISR, summary);
        thread->setMiscRegNoEffect(IPR_INTID, ipl);
        Fault(new InterruptFault)->invoke(xc);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                thread->readMiscRegNoEffect(IPR_IPLR), ipl, summary);

        // May need to go 1 inst prior
        squashPending = true;

        thread->noSquashFromTC = false;

        setSquashInfoFromXC();
    }
}

template <class Impl>
void
InorderBackEnd<Impl>::tick()
{
    // Squash due to an external source
    // Not sure if this or an interrupt has higher priority
    if (squashPending) {
        squash(squashSeqNum, squashNextPC);
        return;
    }

    // if (interrupt) then set thread PC, stall front end, record that
    // I'm waiting for it to drain.  (for now just squash)
    if (FullSystem && (interruptBlocked || cpu->checkInterrupts(tc))) {
        if (!robEmpty()) {
            interruptBlocked = true;
        //AlphaDep
        } else if (robEmpty() && (PC & 0x3)) {
            // Will need to let the front end continue a bit until
            // we're out of pal mode.  Hopefully we never get into an
            // infinite loop...
            interruptBlocked = false;
        } else {
            interruptBlocked = false;
            checkInterrupts();
            return;
        }
    }

    if (status != DcacheMissLoadStall &&
        status != DcacheMissStoreStall) {
        for (int i = 0; i < width && (*instsAdded) < width; ++i) {
            DynInstPtr inst = frontEnd->getInst();

            if (!inst)
                break;

            instList.push_back(inst);

            (*instsAdded)++;
        }

        if (faultFromFetch && robEmpty() && frontEnd->isEmpty()) {
            handleFault();
        } else {
            executeInsts();
        }
    }
}

template <class Impl>
void
InorderBackEnd<Impl>::executeInsts()
{
    bool completed_last_inst = true;
    int insts_to_execute = *instsToExecute;
    int freed_regs = 0;

    while (insts_to_execute > 0) {
        assert(!instList.empty());
        DynInstPtr inst = instList.front();

        commitPC = inst->readPC();

        thread->setPC(commitPC);
        thread->setNextPC(inst->readNextPC());

        if (FullSystem) {
            int count = 0;
            Addr oldpc;
            do {
                if (count == 0)
                    assert(!thread->noSquashFromTC && !thread->trapPending);
                oldpc = thread->readPC();
                cpu->system->pcEventQueue.service(
                    thread->getXCProxy());
                count++;
            } while (oldpc != thread->readPC());
            if (count > 1) {
                DPRINTF(IBE, "PC skip function event, stopping commit\n");
                completed_last_inst = false;
                squashPending = true;
                break;
            }
        }

        Fault inst_fault = NoFault;

        if (status == DcacheMissComplete) {
            DPRINTF(IBE, "Completing inst [sn:%lli]\n", inst->seqNum);
            status = Running;
        } else if (inst->isMemRef() && status != DcacheMissComplete &&
            (!inst->isDataPrefetch() && !inst->isInstPrefetch())) {
            DPRINTF(IBE, "Initiating mem op inst [sn:%lli] PC: %#x\n",
                    inst->seqNum, inst->readPC());

            cacheCompletionEvent.inst = inst;
            inst_fault = inst->initiateAcc();
            if (inst_fault == NoFault &&
                status != DcacheMissLoadStall &&
                status != DcacheMissStoreStall) {
                inst_fault = inst->completeAcc();
            }
            ++thread->funcExeInst;
        } else {
            DPRINTF(IBE, "Executing inst [sn:%lli] PC: %#x\n",
                    inst->seqNum, inst->readPC());
            inst_fault = inst->execute();
            ++thread->funcExeInst;
        }

        // Will need to be able to break this loop in case the load
        // misses.  Split access/complete ops would be useful here
        // with writeback events.
        if (status == DcacheMissLoadStall) {
            *instsToExecute = insts_to_execute;

            completed_last_inst = false;
            break;
        } else if (status == DcacheMissStoreStall) {
            // Figure out how to fix this hack.  Probably have DcacheMissLoad
            // vs DcacheMissStore.
            *instsToExecute = insts_to_execute;
            completed_last_inst = false;
/*
            instList.pop_front();
            --insts_to_execute;
            if (inst->traceData) {
                inst->traceData->finalize();
            }
*/

            // Don't really need to stop for a store stall as long as
            // the memory system is able to handle store forwarding
            // and such.  Breaking out might help avoid the cache
            // interface becoming blocked.
            break;
        }

        inst->setExecuted();
        inst->setResultReady();
        inst->setCanCommit();

        instList.pop_front();

        --insts_to_execute;
        --(*instsToExecute);

        if (inst->traceData) {
            inst->traceData->finalize();
            inst->traceData = NULL;
        }

        if (inst_fault != NoFault) {
            DPRINTF(IBE, "Inst [sn:%lli] PC %#x has a fault\n",
                    inst->seqNum, inst->readPC());

            assert(!thread->noSquashFromTC);

            thread->noSquashFromTC = true;

            // Consider holding onto the trap and waiting until the trap event
            // happens for this to be executed.
            inst_fault->invoke(xc);

            // Exit state update mode to avoid accidental updating.
            thread->noSquashFromTC = false;

            squashPending = true;

            completed_last_inst = false;
            break;
        }

        for (int i = 0; i < inst->numDestRegs(); ++i) {
            renameTable[inst->destRegIdx(i)] = inst;
            thread->renameTable[inst->destRegIdx(i)] = inst;
            ++freed_regs;
        }

        inst->clearDependents();

        comm->access(0)->doneSeqNum = inst->seqNum;

        if (inst->mispredicted()) {
            squash(inst->seqNum, inst->readNextPC());

            thread->setNextPC(inst->readNextPC());

            break;
        } else if (squashPending) {
            // Something external happened that caused the CPU to squash.
            // Break out of commit and handle the squash next cycle.
            break;
        }
        // If it didn't mispredict, then it executed fine.  Send back its
        // registers and BP info?  What about insts that may still have
        // latency, like loads?  Probably can send back the information after
        // it is completed.

        // keep an instruction count
        cpu->numInst++;
        thread->numInsts++;
    }

    frontEnd->addFreeRegs(freed_regs);

    assert(insts_to_execute >= 0);

    // Should only advance this if I have executed all instructions.
    if (insts_to_execute == 0) {
        numInstsToWB.advance();
    }

    // Should I set the PC to the next PC here?  What do I set next PC to?
    if (completed_last_inst) {
        thread->setPC(thread->readNextPC());
        thread->setNextPC(thread->readPC() + sizeof(MachInst));
    }

    if (squashPending) {
        setSquashInfoFromXC();
    }
}

template <class Impl>
void
InorderBackEnd<Impl>::handleFault()
{
    DPRINTF(Commit, "Handling fault from fetch\n");

    assert(!thread->noSquashFromTC);

    thread->noSquashFromTC = true;

    // Consider holding onto the trap and waiting until the trap event
    // happens for this to be executed.
    faultFromFetch->invoke(xc);

    // Exit state update mode to avoid accidental updating.
    thread->noSquashFromTC = false;

    squashPending = true;

    setSquashInfoFromXC();
}

template <class Impl>
void
InorderBackEnd<Impl>::squash(const InstSeqNum &squash_num, const Addr &next_PC)
{
    DPRINTF(IBE, "Squashing from [sn:%lli], setting PC to %#x\n",
            squash_num, next_PC);

    InstListIt squash_it = --(instList.end());

    int freed_regs = 0;

    while (!instList.empty() && (*squash_it)->seqNum > squash_num) {
        DynInstPtr inst = *squash_it;

        DPRINTF(IBE, "Squashing instruction PC %#x, [sn:%lli].\n",
                inst->readPC(),
                inst->seqNum);

        // May cause problems with misc regs
        freed_regs+= inst->numDestRegs();
        inst->clearDependents();
        squash_it--;
        instList.pop_back();
    }

    frontEnd->addFreeRegs(freed_regs);

    for (int i = 0; i < latency+1; ++i) {
        numInstsToWB.advance();
    }

    squashPending = false;

    // Probably want to make sure that this squash is the one that set the
    // thread into noSquashFromTC mode.
    thread->noSquashFromTC = false;

    // Tell front end to squash, reset PC to new one.
    frontEnd->squash(squash_num, next_PC);

    faultFromFetch = NULL;
}

template <class Impl>
void
InorderBackEnd<Impl>::squashFromXC()
{
    // Record that I need to squash
    squashPending = true;

    thread->noSquashFromTC = true;
}

template <class Impl>
void
InorderBackEnd<Impl>::setSquashInfoFromXC()
{
    // Need to handle the case of the instList being empty.  In that case
    // probably any number works, except maybe with stores in the store buffer.
    squashSeqNum = instList.empty() ? 0 : instList.front()->seqNum - 1;

    squashNextPC = thread->PC;
}

template <class Impl>
void
InorderBackEnd<Impl>::fetchFault(Fault &fault)
{
    faultFromFetch = fault;
}

template <class Impl>
void
InorderBackEnd<Impl>::dumpInsts()
{
    int num = 0;
    int valid_num = 0;

    InstListIt inst_list_it = instList.begin();

    cprintf("Inst list size: %i\n", instList.size());

    while (inst_list_it != instList.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone) {
                // Loads that have not been marked as executed still count
                // towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->readPC(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
        }

        cprintf("\n");

        inst_list_it++;
        ++num;
    }
}

template <class Impl>
InorderBackEnd<Impl>::DCacheCompletionEvent::DCacheCompletionEvent(
    InorderBackEnd *_be)
    : Event(&mainEventQueue, CPU_Tick_Pri), be(_be)
{
//    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
InorderBackEnd<Impl>::DCacheCompletionEvent::process()
{
    inst->completeAcc();
    be->status = DcacheMissComplete;
}

template <class Impl>
const char *
InorderBackEnd<Impl>::DCacheCompletionEvent::description() const
{
    return "DCache completion";
}
#endif//__CPU_OZONE_INORDER_BACK_END_IMPL_HH__
