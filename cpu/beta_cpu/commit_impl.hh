// @todo: Bug when something reaches execute, and mispredicts, but is never
// put into the ROB because the ROB is full.  Need rename stage to predict
// the free ROB entries better.

#ifndef __COMMIT_IMPL_HH__
#define __COMMIT_IMPL_HH__

#include "base/timebuf.hh"
#include "cpu/beta_cpu/commit.hh"
#include "cpu/exetrace.hh"

template <class Impl>
SimpleCommit<Impl>::SimpleCommit(Params &params)
    : dcacheInterface(params.dcacheInterface),
      iewToCommitDelay(params.iewToCommitDelay),
      renameToROBDelay(params.renameToROBDelay),
      renameWidth(params.renameWidth),
      iewWidth(params.executeWidth),
      commitWidth(params.commitWidth)
{
    _status = Idle;
}

template <class Impl>
void
SimpleCommit<Impl>::regStats()
{
    commitCommittedInsts
        .name(name() + ".commitCommittedInsts")
        .desc("The number of committed instructions")
        .prereq(commitCommittedInsts);
    commitSquashedInsts
        .name(name() + ".commitSquashedInsts")
        .desc("The number of squashed insts skipped by commit")
        .prereq(commitSquashedInsts);
    commitSquashEvents
        .name(name() + ".commitSquashEvents")
        .desc("The number of times commit is told to squash")
        .prereq(commitSquashEvents);
    commitNonSpecStalls
        .name(name() + ".commitNonSpecStalls")
        .desc("The number of times commit has been forced to stall to "
              "communicate backwards")
        .prereq(commitNonSpecStalls);
    commitCommittedBranches
        .name(name() + ".commitCommittedBranches")
        .desc("The number of committed branches")
        .prereq(commitCommittedBranches);
    commitCommittedLoads
        .name(name() + ".commitCommittedLoads")
        .desc("The number of committed loads")
        .prereq(commitCommittedLoads);
    commitCommittedMemRefs
        .name(name() + ".commitCommittedMemRefs")
        .desc("The number of committed memory references")
        .prereq(commitCommittedMemRefs);
    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("The number of times a branch was mispredicted")
        .prereq(branchMispredicts);
    n_committed_dist
        .init(0,commitWidth,1)
        .name(name() + ".COM:committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(Stats::pdf)
        ;
}

template <class Impl>
void
SimpleCommit<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Commit, "Commit: Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template <class Impl>
void
SimpleCommit<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Commit, "Commit: Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to send information back to IEW.
    toIEW = timeBuffer->getWire(0);

    // Setup wire to read data from IEW (for the ROB).
    robInfoFromIEW = timeBuffer->getWire(-iewToCommitDelay);
}

template <class Impl>
void
SimpleCommit<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(Commit, "Commit: Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromRename = renameQueue->getWire(-renameToROBDelay);
}

template <class Impl>
void
SimpleCommit<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    DPRINTF(Commit, "Commit: Setting IEW queue pointer.\n");
    iewQueue = iq_ptr;

    // Setup wire to get instructions from IEW.
    fromIEW = iewQueue->getWire(-iewToCommitDelay);
}

template <class Impl>
void
SimpleCommit<Impl>::setROB(ROB *rob_ptr)
{
    DPRINTF(Commit, "Commit: Setting ROB pointer.\n");
    rob = rob_ptr;
}

template <class Impl>
void
SimpleCommit<Impl>::tick()
{
    // If the ROB is currently in its squash sequence, then continue
    // to squash.  In this case, commit does not do anything.  Otherwise
    // run commit.
    if (_status == ROBSquashing) {
        if (rob->isDoneSquashing()) {
            _status = Running;
        } else {
            rob->doSquash();

            // Send back sequence number of tail of ROB, so other stages
            // can squash younger instructions.  Note that really the only
            // stage that this is important for is the IEW stage; other
            // stages can just clear all their state as long as selective
            // replay isn't used.
            toIEW->commitInfo.doneSeqNum = rob->readTailSeqNum();
            toIEW->commitInfo.robSquashing = true;
        }
    } else {
        commit();
    }

    markCompletedInsts();

    // Writeback number of free ROB entries here.
    DPRINTF(Commit, "Commit: ROB has %d free entries.\n",
            rob->numFreeEntries());
    toIEW->commitInfo.freeROBEntries = rob->numFreeEntries();
}

template <class Impl>
void
SimpleCommit<Impl>::commit()
{
    //////////////////////////////////////
    // Check for interrupts
    //////////////////////////////////////

    // Process interrupts if interrupts are enabled and not in PAL mode.
    // Take the PC from commit and write it to the IPR, then squash.  The
    // interrupt completing will take care of restoring the PC from that value
    // in the IPR.  Look at IPR[EXC_ADDR];
    // hwrei() is what resets the PC to the place where instruction execution
    // beings again.
#ifdef FULL_SYSTEM
    if (//checkInterrupts &&
        cpu->check_interrupts() &&
        !cpu->inPalMode(readCommitPC())) {
        // Will need to squash all instructions currently in flight and have
        // the interrupt handler restart at the last non-committed inst.
        // Most of that can be handled through the trap() function.  The
        // processInterrupts() function really just checks for interrupts
        // and then calls trap() if there is an interrupt present.

        // CPU will handle implementation of the interrupt.
        cpu->processInterrupts();
    }
#endif // FULL_SYSTEM

    ////////////////////////////////////
    // Check for squash signal, handle that first
    ////////////////////////////////////

    // Want to mainly check if the IEW stage is telling the ROB to squash.
    // Should I also check if the commit stage is telling the ROB to squah?
    // This might be necessary to keep the same timing between the IQ and
    // the ROB...
    if (fromIEW->squash) {
        DPRINTF(Commit, "Commit: Squashing instructions in the ROB.\n");

        _status = ROBSquashing;

        InstSeqNum squashed_inst = fromIEW->squashedSeqNum;

        rob->squash(squashed_inst);

        // Send back the sequence number of the squashed instruction.
        toIEW->commitInfo.doneSeqNum = squashed_inst;

        // Send back the squash signal to tell stages that they should squash.
        toIEW->commitInfo.squash = true;

        // Send back the rob squashing signal so other stages know that the
        // ROB is in the process of squashing.
        toIEW->commitInfo.robSquashing = true;

        toIEW->commitInfo.branchMispredict = fromIEW->branchMispredict;

        toIEW->commitInfo.branchTaken = fromIEW->branchTaken;

        toIEW->commitInfo.nextPC = fromIEW->nextPC;

        toIEW->commitInfo.mispredPC = fromIEW->mispredPC;

        if (toIEW->commitInfo.branchMispredict) {
            ++branchMispredicts;
        }
    }

    if (_status != ROBSquashing) {
        // If we're not currently squashing, then get instructions.
        getInsts();

        // Try to commit any instructions.
        commitInsts();
    }

    // If the ROB is empty, we can set this stage to idle.  Use this
    // in the future when the Idle status will actually be utilized.
#if 0
    if (rob->isEmpty()) {
        DPRINTF(Commit, "Commit: ROB is empty.  Status changed to idle.\n");
        _status = Idle;
        // Schedule an event so that commit will actually wake up
        // once something gets put in the ROB.
    }
#endif
}

// Loop that goes through as many instructions in the ROB as possible and
// tries to commit them.  The actual work for committing is done by the
// commitHead() function.
template <class Impl>
void
SimpleCommit<Impl>::commitInsts()
{
    ////////////////////////////////////
    // Handle commit
    // Note that commit will be handled prior to the ROB so that the ROB
    // only tries to commit instructions it has in this current cycle, and
    // not instructions it is writing in during this cycle.
    // Can't commit and squash things at the same time...
    ////////////////////////////////////

    if (rob->isEmpty())
        return;

    DynInstPtr head_inst = rob->readHeadInst();

    unsigned num_committed = 0;

    // Commit as many instructions as possible until the commit bandwidth
    // limit is reached, or it becomes impossible to commit any more.
    while (!rob->isEmpty() &&
           head_inst->readyToCommit() &&
           num_committed < commitWidth)
    {
        DPRINTF(Commit, "Commit: Trying to commit head instruction.\n");

        // If the head instruction is squashed, it is ready to retire at any
        // time.  However, we need to avoid updating any other state
        // incorrectly if it's already been squashed.
        if (head_inst->isSquashed()) {
            // Hack to avoid the instruction being retired (and deleted) if
            // it hasn't been through the IEW stage yet.
/*
            if (!head_inst->isExecuted()) {
                break;
            }
*/

            DPRINTF(Commit, "Commit: Retiring squashed instruction from "
                    "ROB.\n");

            // Tell ROB to retire head instruction.  This retires the head
            // inst in the ROB without affecting any other stages.
            rob->retireHead();

            ++commitSquashedInsts;

        } else {
            // Increment the total number of non-speculative instructions
            // executed.
            // Hack for now: it really shouldn't happen until after the
            // commit is deemed to be successful, but this count is needed
            // for syscalls.
            cpu->funcExeInst++;

            // Try to commit the head instruction.
            bool commit_success = commitHead(head_inst, num_committed);

            // Update what instruction we are looking at if the commit worked.
            if (commit_success) {
                ++num_committed;

                // Send back which instruction has been committed.
                // @todo: Update this later when a wider pipeline is used.
                // Hmm, can't really give a pointer here...perhaps the
                // sequence number instead (copy).
                toIEW->commitInfo.doneSeqNum = head_inst->seqNum;

                ++commitCommittedInsts;

                if (!head_inst->isNop()) {
                    cpu->instDone();
                }
            } else {
                break;
            }
        }

        // Update the pointer to read the next instruction in the ROB.
        head_inst = rob->readHeadInst();
    }

    DPRINTF(CommitRate, "%i\n", num_committed);
    n_committed_dist.sample(num_committed);
}

template <class Impl>
bool
SimpleCommit<Impl>::commitHead(DynInstPtr &head_inst, unsigned inst_num)
{
    // Make sure instruction is valid
    assert(head_inst);

    // If the instruction is not executed yet, then it is a non-speculative
    // or store inst.  Signal backwards that it should be executed.
    if (!head_inst->isExecuted()) {
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
        cpu->funcExeInst--;

        if (head_inst->isNonSpeculative()) {
            DPRINTF(Commit, "Commit: Encountered a store or non-speculative "
                    "instruction at the head of the ROB, PC %#x.\n",
                    head_inst->readPC());

            toIEW->commitInfo.nonSpecSeqNum = head_inst->seqNum;

            // Change the instruction so it won't try to commit again until
            // it is executed.
            head_inst->clearCanCommit();

            ++commitNonSpecStalls;

            return false;
        } else {
            panic("Commit: Trying to commit un-executed instruction "
                  "of unknown type!\n");
        }
    }

    // Now check if it's one of the special trap or barrier or
    // serializing instructions.
    if (head_inst->isThreadSync()  ||
        head_inst->isSerializing() ||
        head_inst->isMemBarrier()  ||
        head_inst->isWriteBarrier() )
    {
        // Not handled for now.  Mem barriers and write barriers are safe
        // to simply let commit as memory accesses only happen once they
        // reach the head of commit.  Not sure about the other two.
        panic("Serializing or barrier instructions"
              " are not handled yet.\n");
    }

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = head_inst->getFault();

    if (inst_fault != No_Fault && inst_fault != Fake_Mem_Fault) {
        if (!head_inst->isNop()) {
#ifdef FULL_SYSTEM
            cpu->trap(inst_fault);
#else // !FULL_SYSTEM
            panic("fault (%d) detected @ PC %08p", inst_fault,
                  head_inst->PC);
#endif // FULL_SYSTEM
        }
    }

    // Check if we're really ready to commit.  If not then return false.
    // I'm pretty sure all instructions should be able to commit if they've
    // reached this far.  For now leave this in as a check.
    if (!rob->isHeadReady()) {
        panic("Commit: Unable to commit head instruction!\n");
        return false;
    }

    // If it's a branch, then send back branch prediction update info
    // to the fetch stage.
    // This should be handled in the iew stage if a mispredict happens...

    if (head_inst->isControl()) {

#if 0
        toIEW->nextPC = head_inst->readPC();
        //Maybe switch over to BTB incorrect.
        toIEW->btbMissed = head_inst->btbMiss();
        toIEW->target = head_inst->nextPC;
        //Maybe also include global history information.
        //This simple version will have no branch prediction however.
#endif

        ++commitCommittedBranches;
    }

#if 0
    // Explicit communication back to the LDSTQ that a load has been committed
    // and can be removed from the LDSTQ.  Stores don't need this because
    // the LDSTQ will already have been told that a store has reached the head
    // of the ROB.  Consider including communication if it's a store as well
    // to keep things orthagonal.
    if (head_inst->isMemRef()) {
        ++commitCommittedMemRefs;
        if (head_inst->isLoad()) {
            toIEW->commitInfo.commitIsLoad = true;
            ++commitCommittedLoads;
        }
    }
#endif

    // Now that the instruction is going to be committed, finalize its
    // trace data.
    if (head_inst->traceData) {
        head_inst->traceData->finalize();
    }

    //Finally clear the head ROB entry.
    rob->retireHead();

    // Return true to indicate that we have committed an instruction.
    return true;
}

template <class Impl>
void
SimpleCommit<Impl>::getInsts()
{
    //////////////////////////////////////
    // Handle ROB functions
    //////////////////////////////////////

    // Read any issued instructions and place them into the ROB.  Do this
    // prior to squashing to avoid having instructions in the ROB that
    // don't get squashed properly.
    int insts_to_process = min((int)renameWidth, fromRename->size);

    for (int inst_num = 0;
         inst_num < insts_to_process;
         ++inst_num)
    {
        if (!fromRename->insts[inst_num]->isSquashed()) {
            DPRINTF(Commit, "Commit: Inserting PC %#x into ROB.\n",
                    fromRename->insts[inst_num]->readPC());
            rob->insertInst(fromRename->insts[inst_num]);
        } else {
            DPRINTF(Commit, "Commit: Instruction %i PC %#x was "
                    "squashed, skipping.\n",
                    fromRename->insts[inst_num]->seqNum,
                    fromRename->insts[inst_num]->readPC());
        }
    }
}

template <class Impl>
void
SimpleCommit<Impl>::markCompletedInsts()
{
    // Grab completed insts out of the IEW instruction queue, and mark
    // instructions completed within the ROB.
    for (int inst_num = 0;
         inst_num < fromIEW->size && fromIEW->insts[inst_num];
         ++inst_num)
    {
        DPRINTF(Commit, "Commit: Marking PC %#x, SN %i ready within ROB.\n",
                fromIEW->insts[inst_num]->readPC(),
                fromIEW->insts[inst_num]->seqNum);

        // Mark the instruction as ready to commit.
        fromIEW->insts[inst_num]->setCanCommit();
    }
}

template <class Impl>
uint64_t
SimpleCommit<Impl>::readCommitPC()
{
    return rob->readHeadPC();
}

#endif // __COMMIT_IMPL_HH__
