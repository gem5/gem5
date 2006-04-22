
#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "cpu/ozone/inorder_back_end.hh"
#include "cpu/ozone/thread_state.hh"

using namespace TheISA;

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

#if FULL_SYSTEM
template <class Impl>
void
InorderBackEnd<Impl>::checkInterrupts()
{
    //Check if there are any outstanding interrupts
    //Handle the interrupts
    int ipl = 0;
    int summary = 0;

    cpu->checkInterrupts = false;

    if (thread->readMiscReg(IPR_ASTRR))
        panic("asynchronous traps not implemented\n");

    if (thread->readMiscReg(IPR_SIRR)) {
        for (int i = INTLEVEL_SOFTWARE_MIN;
             i < INTLEVEL_SOFTWARE_MAX; i++) {
            if (thread->readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
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

    if (ipl && ipl > thread->readMiscReg(IPR_IPLR)) {
        thread->inSyscall = true;

        thread->setMiscReg(IPR_ISR, summary);
        thread->setMiscReg(IPR_INTID, ipl);
        Fault(new InterruptFault)->invoke(xc);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                thread->readMiscReg(IPR_IPLR), ipl, summary);

        // May need to go 1 inst prior
        squashPending = true;

        thread->inSyscall = false;

        setSquashInfoFromXC();
    }
}
#endif

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
#if FULL_SYSTEM
    if (interruptBlocked ||
        (cpu->checkInterrupts &&
        cpu->check_interrupts() &&
        !cpu->inPalMode())) {
        if (!robEmpty()) {
            interruptBlocked = true;
        } else if (robEmpty() && cpu->inPalMode()) {
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
#endif

    if (status != DcacheMissLoadStall &&
        status != DcacheMissStoreStall) {
        for (int i = 0; i < width && (*instsAdded) < width; ++i) {
            DynInstPtr inst = frontEnd->getInst();

            if (!inst)
                break;

            instList.push_back(inst);

            (*instsAdded)++;
        }

#if FULL_SYSTEM
        if (faultFromFetch && robEmpty() && frontEnd->isEmpty()) {
            handleFault();
        } else {
            executeInsts();
        }
#else
        executeInsts();
#endif
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

#if FULL_SYSTEM
        int count = 0;
        Addr oldpc;
        do {
            if (count == 0)
                assert(!thread->inSyscall && !thread->trapPending);
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
#endif

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
        inst->setCompleted();
        inst->setCanCommit();

        instList.pop_front();

        --insts_to_execute;
        --(*instsToExecute);

        if (inst->traceData) {
            inst->traceData->finalize();
            inst->traceData = NULL;
        }

        if (inst_fault != NoFault) {
#if FULL_SYSTEM
            DPRINTF(IBE, "Inst [sn:%lli] PC %#x has a fault\n",
                    inst->seqNum, inst->readPC());

            assert(!thread->inSyscall);

            thread->inSyscall = true;

            // Hack for now; DTB will sometimes need the machine instruction
            // for when faults happen.  So we will set it here, prior to the
            // DTB possibly needing it for this translation.
            thread->setInst(
                static_cast<TheISA::MachInst>(inst->staticInst->machInst));

            // Consider holding onto the trap and waiting until the trap event
            // happens for this to be executed.
            inst_fault->invoke(xc);

            // Exit state update mode to avoid accidental updating.
            thread->inSyscall = false;

            squashPending = true;

            // Generate trap squash event.
//            generateTrapEvent(tid);
            completed_last_inst = false;
            break;
#else // !FULL_SYSTEM
            panic("fault (%d) detected @ PC %08p", inst_fault,
                  inst->PC);
#endif // FULL_SYSTEM
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

    assert(!thread->inSyscall);

    thread->inSyscall = true;

    // Consider holding onto the trap and waiting until the trap event
    // happens for this to be executed.
    faultFromFetch->invoke(xc);

    // Exit state update mode to avoid accidental updating.
    thread->inSyscall = false;

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
    // thread into inSyscall mode.
    thread->inSyscall = false;

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

    thread->inSyscall = true;
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
InorderBackEnd<Impl>::DCacheCompletionEvent::description()
{
    return "DCache completion event";
}
