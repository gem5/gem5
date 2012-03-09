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

#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/ozone/lw_back_end.hh"
#include "cpu/op_class.hh"

template <class Impl>
void
LWBackEnd<Impl>::generateTrapEvent(Tick latency)
{
    DPRINTF(BE, "Generating trap event\n");

    TrapEvent *trap = new TrapEvent(this);

    trap->schedule(curTick() + cpu->ticks(latency));

    thread->trapPending = true;
}

template <class Impl>
int
LWBackEnd<Impl>::wakeDependents(DynInstPtr &inst, bool memory_deps)
{
    assert(!inst->isSquashed());
    std::vector<DynInstPtr> &dependents = memory_deps ? inst->getMemDeps() :
        inst->getDependents();
    int num_outputs = dependents.size();

    DPRINTF(BE, "Waking instruction [sn:%lli] dependents in IQ\n", inst->seqNum);

    for (int i = 0; i < num_outputs; i++) {
        DynInstPtr dep_inst = dependents[i];
        if (!memory_deps) {
            dep_inst->markSrcRegReady();
        } else {
            if (!dep_inst->isSquashed())
                dep_inst->markMemInstReady(inst.get());
        }

        DPRINTF(BE, "Marking source reg ready [sn:%lli] in IQ\n", dep_inst->seqNum);

        if (dep_inst->readyToIssue() && dep_inst->isInROB() &&
            !dep_inst->isNonSpeculative() && !dep_inst->isStoreConditional() &&
            dep_inst->memDepReady() && !dep_inst->isMemBarrier() &&
            !dep_inst->isWriteBarrier()) {
            DPRINTF(BE, "Adding instruction to exeList [sn:%lli]\n",
                    dep_inst->seqNum);
            exeList.push(dep_inst);
            if (dep_inst->iqItValid) {
                DPRINTF(BE, "Removing instruction from waiting list\n");
                waitingList.erase(dep_inst->iqIt);
                waitingInsts--;
                dep_inst->iqItValid = false;
                assert(waitingInsts >= 0);
            }
            if (dep_inst->isMemRef()) {
                removeWaitingMemOp(dep_inst);
                DPRINTF(BE, "Issued a waiting mem op [sn:%lli]\n",
                        dep_inst->seqNum);
            }
        }
    }
    return num_outputs;
}

template <class Impl>
void
LWBackEnd<Impl>::rescheduleMemInst(DynInstPtr &inst)
{
    replayList.push_front(inst);
}

template <class Impl>
LWBackEnd<Impl>::TrapEvent::TrapEvent(LWBackEnd<Impl> *_be)
    : Event(&mainEventQueue, CPU_Tick_Pri), be(_be)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
LWBackEnd<Impl>::TrapEvent::process()
{
    be->trapSquash = true;
}

template <class Impl>
const char *
LWBackEnd<Impl>::TrapEvent::description() const
{
    return "Trap";
}

template <class Impl>
void
LWBackEnd<Impl>::replayMemInst(DynInstPtr &inst)
{
    bool found_inst = false;
    while (!replayList.empty()) {
        exeList.push(replayList.front());
        if (replayList.front() == inst) {
            found_inst = true;
        }
        replayList.pop_front();
    }
    assert(found_inst);
}

template <class Impl>
LWBackEnd<Impl>::LWBackEnd(Params *params)
    : d2i(5, 5), i2e(5, 5), e2c(5, 5), numInstsToWB(params->backEndLatency, 0),
      trapSquash(false), tcSquash(false),
      latency(params->backEndLatency),
      width(params->backEndWidth), lsqLimits(params->lsqLimits),
      exactFullStall(true)
{
    numROBEntries = params->numROBEntries;
    numInsts = 0;
    maxOutstandingMemOps = params->maxOutstandingMemOps;
    numWaitingMemOps = 0;
    waitingInsts = 0;
    switchedOut = false;
    switchPending = false;

    LSQ.setBE(this);

    // Setup IQ and LSQ with their parameters here.
    instsToDispatch = d2i.getWire(-1);

    instsToExecute = i2e.getWire(-1);

    dispatchWidth = params->dispatchWidth ? params->dispatchWidth : width;
    issueWidth = params->issueWidth ? params->issueWidth : width;
    wbWidth = params->wbWidth ? params->wbWidth : width;
    commitWidth = params->commitWidth ? params->commitWidth : width;

    LSQ.init(params, params->LQEntries, params->SQEntries, 0);

    dispatchStatus = Running;
    commitStatus = Running;
}

template <class Impl>
std::string
LWBackEnd<Impl>::name() const
{
    return cpu->name() + ".backend";
}

template <class Impl>
void
LWBackEnd<Impl>::regStats()
{
    using namespace Stats;
    LSQ.regStats();

    robCapEvents
        .init(cpu->numThreads)
        .name(name() + ".ROB:cap_events")
        .desc("number of cycles where ROB cap was active")
        .flags(total)
        ;

    robCapInstCount
        .init(cpu->numThreads)
        .name(name() + ".ROB:cap_inst")
        .desc("number of instructions held up by ROB cap")
        .flags(total)
        ;

    iqCapEvents
        .init(cpu->numThreads)
        .name(name() +".IQ:cap_events" )
        .desc("number of cycles where IQ cap was active")
        .flags(total)
        ;

    iqCapInstCount
        .init(cpu->numThreads)
        .name(name() + ".IQ:cap_inst")
        .desc("number of instructions held up by IQ cap")
        .flags(total)
        ;

    exeInst
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:count")
        .desc("number of insts issued")
        .flags(total)
        ;

    exeSwp
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:swp")
        .desc("number of swp insts issued")
        .flags(total)
        ;

    exeNop
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:nop")
        .desc("number of nop insts issued")
        .flags(total)
        ;

    exeRefs
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:refs")
        .desc("number of memory reference insts issued")
        .flags(total)
        ;

    exeLoads
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:loads")
        .desc("number of load insts issued")
        .flags(total)
        ;

    exeBranches
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:branches")
        .desc("Number of branches issued")
        .flags(total)
        ;

    issuedOps
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:op_count")
        .desc("number of insts issued")
        .flags(total)
        ;

/*
    for (int i=0; i<Num_OpClasses; ++i) {
        stringstream subname;
        subname << opClassStrings[i] << "_delay";
        issue_delay_dist.subname(i, subname.str());
    }
*/
    //
    //  Other stats
    //
    lsqForwLoads
        .init(cpu->numThreads)
        .name(name() + ".LSQ:forw_loads")
        .desc("number of loads forwarded via LSQ")
        .flags(total)
        ;

    invAddrLoads
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:addr_loads")
        .desc("number of invalid-address loads")
        .flags(total)
        ;

    invAddrSwpfs
        .init(cpu->numThreads)
        .name(name() + ".ISSUE:addr_swpfs")
        .desc("number of invalid-address SW prefetches")
        .flags(total)
        ;

    lsqBlockedLoads
        .init(cpu->numThreads)
        .name(name() + ".LSQ:blocked_loads")
        .desc("number of ready loads not issued due to memory disambiguation")
        .flags(total)
        ;

    lsqInversion
        .name(name() + ".ISSUE:lsq_invert")
        .desc("Number of times LSQ instruction issued early")
        ;

    nIssuedDist
        .init(issueWidth + 1)
        .name(name() + ".ISSUE:issued_per_cycle")
        .desc("Number of insts issued each cycle")
        .flags(total | pdf | dist)
        ;
/*
    issueDelayDist
        .init(Num_OpClasses,0,99,2)
        .name(name() + ".ISSUE:")
        .desc("cycles from operands ready to issue")
        .flags(pdf | cdf)
        ;

    queueResDist
        .init(Num_OpClasses, 0, 99, 2)
        .name(name() + ".IQ:residence:")
        .desc("cycles from dispatch to issue")
        .flags(total | pdf | cdf )
        ;
    for (int i = 0; i < Num_OpClasses; ++i) {
        queueResDist.subname(i, opClassStrings[i]);
    }
*/
    writebackCount
        .init(cpu->numThreads)
        .name(name() + ".WB:count")
        .desc("cumulative count of insts written-back")
        .flags(total)
        ;

    producerInst
        .init(cpu->numThreads)
        .name(name() + ".WB:producers")
        .desc("num instructions producing a value")
        .flags(total)
        ;

    consumerInst
        .init(cpu->numThreads)
        .name(name() + ".WB:consumers")
        .desc("num instructions consuming a value")
        .flags(total)
        ;

    wbPenalized
        .init(cpu->numThreads)
        .name(name() + ".WB:penalized")
        .desc("number of instrctions required to write to 'other' IQ")
        .flags(total)
        ;


    wbPenalizedRate
        .name(name() + ".WB:penalized_rate")
        .desc ("fraction of instructions written-back that wrote to 'other' IQ")
        .flags(total)
        ;

    wbPenalizedRate = wbPenalized / writebackCount;

    wbFanout
        .name(name() + ".WB:fanout")
        .desc("average fanout of values written-back")
        .flags(total)
        ;

    wbFanout = producerInst / consumerInst;

    wbRate
        .name(name() + ".WB:rate")
        .desc("insts written-back per cycle")
        .flags(total)
        ;
    wbRate = writebackCount / cpu->numCycles;

    statComInst
        .init(cpu->numThreads)
        .name(name() + ".COM:count")
        .desc("Number of instructions committed")
        .flags(total)
        ;

    statComSwp
        .init(cpu->numThreads)
        .name(name() + ".COM:swp_count")
        .desc("Number of s/w prefetches committed")
        .flags(total)
        ;

    statComRefs
        .init(cpu->numThreads)
        .name(name() +  ".COM:refs")
        .desc("Number of memory references committed")
        .flags(total)
        ;

    statComLoads
        .init(cpu->numThreads)
        .name(name() +  ".COM:loads")
        .desc("Number of loads committed")
        .flags(total)
        ;

    statComMembars
        .init(cpu->numThreads)
        .name(name() +  ".COM:membars")
        .desc("Number of memory barriers committed")
        .flags(total)
        ;

    statComBranches
        .init(cpu->numThreads)
        .name(name() + ".COM:branches")
        .desc("Number of branches committed")
        .flags(total)
        ;
    nCommittedDist
        .init(0,commitWidth,1)
        .name(name() + ".COM:committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(pdf)
        ;

    //
    //  Commit-Eligible instructions...
    //
    //  -> The number of instructions eligible to commit in those
    //  cycles where we reached our commit BW limit (less the number
    //  actually committed)
    //
    //  -> The average value is computed over ALL CYCLES... not just
    //  the BW limited cycles
    //
    //  -> The standard deviation is computed only over cycles where
    //  we reached the BW limit
    //
    commitEligible
        .init(cpu->numThreads)
        .name(name() + ".COM:bw_limited")
        .desc("number of insts not committed due to BW limits")
        .flags(total)
        ;

    commitEligibleSamples
        .name(name() + ".COM:bw_lim_events")
        .desc("number cycles where commit BW limit reached")
        ;

    squashedInsts
        .init(cpu->numThreads)
        .name(name() + ".COM:squashed_insts")
        .desc("Number of instructions removed from inst list")
        ;

    ROBSquashedInsts
        .init(cpu->numThreads)
        .name(name() + ".COM:rob_squashed_insts")
        .desc("Number of instructions removed from inst list when they reached the head of the ROB")
        ;

    ROBFcount
        .name(name() + ".ROB:full_count")
        .desc("number of cycles where ROB was full")
        ;

    ROBCount
        .init(cpu->numThreads)
        .name(name() + ".ROB:occupancy")
        .desc(name() + ".ROB occupancy (cumulative)")
        .flags(total)
        ;

    ROBFullRate
        .name(name() + ".ROB:full_rate")
        .desc("ROB full per cycle")
        ;
    ROBFullRate = ROBFcount / cpu->numCycles;

    ROBOccRate
        .name(name() + ".ROB:occ_rate")
        .desc("ROB occupancy rate")
        .flags(total)
        ;
    ROBOccRate = ROBCount / cpu->numCycles;
/*
    ROBOccDist
        .init(cpu->numThreads, 0, numROBEntries, 2)
        .name(name() + ".ROB:occ_dist")
        .desc("ROB Occupancy per cycle")
        .flags(total | cdf)
        ;
*/
}

template <class Impl>
void
LWBackEnd<Impl>::setCPU(OzoneCPU *cpu_ptr)
{
    cpu = cpu_ptr;
    LSQ.setCPU(cpu_ptr);
    checker = cpu->checker;
}

template <class Impl>
void
LWBackEnd<Impl>::setCommBuffer(TimeBuffer<CommStruct> *_comm)
{
    comm = _comm;
    toIEW = comm->getWire(0);
    fromCommit = comm->getWire(-1);
}

template <class Impl>
void
LWBackEnd<Impl>::checkInterrupts()
{
    if (cpu->checkInterrupts(tc) && !trapSquash && !tcSquash) {
        frontEnd->interruptPending = true;
        if (robEmpty() && !LSQ.hasStoresToWB()) {
            // Will need to squash all instructions currently in flight and have
            // the interrupt handler restart at the last non-committed inst.
            // Most of that can be handled through the trap() function.  The
            // processInterrupts() function really just checks for interrupts
            // and then calls trap() if there is an interrupt present.

            // Not sure which thread should be the one to interrupt.  For now
            // always do thread 0.
            assert(!thread->inSyscall);
            thread->inSyscall = true;

            // CPU will handle implementation of the interrupt.
            cpu->processInterrupts();

            // Now squash or record that I need to squash this cycle.
            commitStatus = TrapPending;

            // Exit state update mode to avoid accidental updating.
            thread->inSyscall = false;

            // Generate trap squash event.
            generateTrapEvent();

            DPRINTF(BE, "Interrupt detected.\n");
        } else {
            DPRINTF(BE, "Interrupt must wait for ROB to drain.\n");
        }
    }
}

template <class Impl>
void
LWBackEnd<Impl>::handleFault(Fault &fault, Tick latency)
{
    DPRINTF(BE, "Handling fault!\n");

    assert(!thread->inSyscall);

    thread->inSyscall = true;

    // Consider holding onto the trap and waiting until the trap event
    // happens for this to be executed.
    fault->invoke(thread->getTC());

    // Exit state update mode to avoid accidental updating.
    thread->inSyscall = false;

    commitStatus = TrapPending;

    // Generate trap squash event.
    generateTrapEvent(latency);
}

template <class Impl>
void
LWBackEnd<Impl>::tick()
{
    DPRINTF(BE, "Ticking back end\n");

    // Read in any done instruction information and update the IQ or LSQ.
    updateStructures();

    if (switchPending && robEmpty() && !LSQ.hasStoresToWB()) {
        cpu->signalSwitched();
        return;
    }

    readyInstsForCommit();

    numInstsToWB.advance();

    ROBCount[0]+= numInsts;

    wbCycle = 0;

    checkInterrupts();

    if (trapSquash) {
        assert(!tcSquash);
        squashFromTrap();
    } else if (tcSquash) {
        squashFromTC();
    }

    if (dispatchStatus != Blocked) {
        dispatchInsts();
    } else {
        checkDispatchStatus();
    }

    if (commitStatus != TrapPending) {
        executeInsts();

        commitInsts();
    }

    LSQ.writebackStores();

    DPRINTF(BE, "Waiting insts: %i, mem ops: %i, ROB entries in use: %i, "
            "LSQ loads: %i, LSQ stores: %i\n",
            waitingInsts, numWaitingMemOps, numInsts,
            LSQ.numLoads(), LSQ.numStores());

#ifdef DEBUG
    assert(numInsts == instList.size());
    assert(waitingInsts == waitingList.size());
    assert(numWaitingMemOps == waitingMemOps.size());
    assert(!switchedOut);
#endif
}

template <class Impl>
void
LWBackEnd<Impl>::updateStructures()
{
    if (fromCommit->doneSeqNum) {
        LSQ.commitLoads(fromCommit->doneSeqNum);
        LSQ.commitStores(fromCommit->doneSeqNum);
    }

    if (fromCommit->nonSpecSeqNum) {
        if (fromCommit->uncached) {
//            LSQ.executeLoad(fromCommit->lqIdx);
        } else {
//            IQ.scheduleNonSpec(
//                fromCommit->nonSpecSeqNum);
        }
    }
}

template <class Impl>
void
LWBackEnd<Impl>::addToLSQ(DynInstPtr &inst)
{
    // Do anything LSQ specific here?
    LSQ.insert(inst);
}

template <class Impl>
void
LWBackEnd<Impl>::dispatchInsts()
{
    DPRINTF(BE, "Trying to dispatch instructions.\n");

    while (numInsts < numROBEntries &&
           numWaitingMemOps < maxOutstandingMemOps) {
        // Get instruction from front of time buffer
        if (lsqLimits && LSQ.isFull()) {
            break;
        }

        DynInstPtr inst = frontEnd->getInst();
        if (!inst) {
            break;
        } else if (inst->isSquashed()) {
            continue;
        }

        ++numInsts;
        instList.push_front(inst);

        inst->setInROB();

        DPRINTF(BE, "Dispatching instruction [sn:%lli] PC:%#x\n",
                inst->seqNum, inst->readPC());

        for (int i = 0; i < inst->numDestRegs(); ++i)
            renameTable[inst->destRegIdx(i)] = inst;

        if (inst->isMemBarrier() || inst->isWriteBarrier()) {
            if (memBarrier) {
                DPRINTF(BE, "Instruction [sn:%lli] is waiting on "
                        "barrier [sn:%lli].\n",
                        inst->seqNum, memBarrier->seqNum);
                memBarrier->addMemDependent(inst);
                inst->addSrcMemInst(memBarrier);
            }
            memBarrier = inst;
            inst->setCanCommit();
        } else if (inst->readyToIssue() &&
                   !inst->isNonSpeculative() &&
                   !inst->isStoreConditional()) {
            if (inst->isMemRef()) {

                LSQ.insert(inst);
                if (memBarrier) {
                    DPRINTF(BE, "Instruction [sn:%lli] is waiting on "
                            "barrier [sn:%lli].\n",
                            inst->seqNum, memBarrier->seqNum);
                    memBarrier->addMemDependent(inst);
                    inst->addSrcMemInst(memBarrier);
                    addWaitingMemOp(inst);

                    waitingList.push_front(inst);
                    inst->iqIt = waitingList.begin();
                    inst->iqItValid = true;
                    waitingInsts++;
                } else {
                    DPRINTF(BE, "Instruction [sn:%lli] ready, addding to "
                            "exeList.\n",
                            inst->seqNum);
                    exeList.push(inst);
                }
            } else if (inst->isNop()) {
                DPRINTF(BE, "Nop encountered [sn:%lli], skipping exeList.\n",
                        inst->seqNum);
                inst->setIssued();
                inst->setExecuted();
                inst->setCanCommit();
                numInstsToWB[0]++;
            } else {
                DPRINTF(BE, "Instruction [sn:%lli] ready, addding to "
                        "exeList.\n",
                        inst->seqNum);
                exeList.push(inst);
            }
        } else {
            if (inst->isNonSpeculative() || inst->isStoreConditional()) {
                inst->setCanCommit();
                DPRINTF(BE, "Adding non speculative instruction\n");
            }

            if (inst->isMemRef()) {
                addWaitingMemOp(inst);
                LSQ.insert(inst);
                if (memBarrier) {
                    memBarrier->addMemDependent(inst);
                    inst->addSrcMemInst(memBarrier);

                    DPRINTF(BE, "Instruction [sn:%lli] is waiting on "
                            "barrier [sn:%lli].\n",
                            inst->seqNum, memBarrier->seqNum);
                }
            }

            DPRINTF(BE, "Instruction [sn:%lli] not ready, addding to "
                    "waitingList.\n",
                    inst->seqNum);
            waitingList.push_front(inst);
            inst->iqIt = waitingList.begin();
            inst->iqItValid = true;
            waitingInsts++;
        }
    }

    // Check if IQ or LSQ is full.  If so we'll need to break and stop
    // removing instructions.  Also update the number of insts to remove
    // from the queue.  Check here if we don't care about exact stall
    // conditions.
/*
    bool stall = false;
    if (IQ.isFull()) {
        DPRINTF(BE, "IQ is full!\n");
        stall = true;
    } else if (LSQ.isFull()) {
        DPRINTF(BE, "LSQ is full!\n");
        stall = true;
    } else if (isFull()) {
        DPRINTF(BE, "ROB is full!\n");
        stall = true;
        ROB_fcount++;
    }
    if (stall) {
        d2i.advance();
        dispatchStall();
        return;
    }
*/
}

template <class Impl>
void
LWBackEnd<Impl>::dispatchStall()
{
    dispatchStatus = Blocked;
    if (!cpu->decoupledFrontEnd) {
        // Tell front end to stall here through a timebuffer, or just tell
        // it directly.
    }
}

template <class Impl>
void
LWBackEnd<Impl>::checkDispatchStatus()
{
    DPRINTF(BE, "Checking dispatch status\n");
    assert(dispatchStatus == Blocked);
    if (!LSQ.isFull() && !isFull()) {
        DPRINTF(BE, "Dispatch no longer blocked\n");
        dispatchStatus = Running;
        dispatchInsts();
    }
}

template <class Impl>
void
LWBackEnd<Impl>::executeInsts()
{
    DPRINTF(BE, "Trying to execute instructions\n");

    int num_executed = 0;
    while (!exeList.empty() && num_executed < issueWidth) {
        DynInstPtr inst = exeList.top();

        DPRINTF(BE, "Executing inst [sn:%lli] PC: %#x\n",
                inst->seqNum, inst->readPC());

        // Check if the instruction is squashed; if so then skip it
        // and don't count it towards the FU usage.
        if (inst->isSquashed()) {
            DPRINTF(BE, "Execute: Instruction was squashed.\n");

            // Not sure how to handle this plus the method of sending # of
            // instructions to use.  Probably will just have to count it
            // towards the bandwidth usage, but not the FU usage.
            ++num_executed;

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();

//            ++iewExecSquashedInsts;
            exeList.pop();

            continue;
        }

        Fault fault = NoFault;

        // Execute instruction.
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef() &&
            (!inst->isDataPrefetch() && !inst->isInstPrefetch())) {
            DPRINTF(BE, "Execute: Initiating access for memory "
                    "reference.\n");

            if (inst->isLoad()) {
                LSQ.executeLoad(inst);
            } else if (inst->isStore()) {
                Fault fault = LSQ.executeStore(inst);

                if (!inst->isStoreConditional() && fault == NoFault) {
                    inst->setExecuted();

                    instToCommit(inst);
                } else if (fault != NoFault) {
                    // If the instruction faulted, then we need to send it along to commit
                    // without the instruction completing.
                    // Send this instruction to commit, also make sure iew stage
                    // realizes there is activity.
                    inst->setExecuted();

                    instToCommit(inst);
                }
            } else {
                panic("Unknown mem type!");
            }
        } else {
            inst->execute();

            inst->setExecuted();

            instToCommit(inst);
        }

        updateExeInstStats(inst);

        ++funcExeInst;
        ++num_executed;

        exeList.pop();

        if (inst->mispredicted()) {
            squashDueToBranch(inst);
            break;
        } else if (LSQ.violation()) {
            // Get the DynInst that caused the violation.  Note that this
            // clears the violation signal.
            DynInstPtr violator;
            violator = LSQ.getMemDepViolator();

            DPRINTF(BE, "LDSTQ detected a violation.  Violator PC: "
                    "%#x, inst PC: %#x.  Addr is: %#x.\n",
                    violator->readPC(), inst->readPC(), inst->physEffAddr);

            // Squash.
            squashDueToMemViolation(inst);
        }
    }

    issuedOps[0]+= num_executed;
    nIssuedDist[num_executed]++;
}

template<class Impl>
void
LWBackEnd<Impl>::instToCommit(DynInstPtr &inst)
{
    DPRINTF(BE, "Sending instructions to commit [sn:%lli] PC %#x.\n",
            inst->seqNum, inst->readPC());

    if (!inst->isSquashed()) {
        if (inst->isExecuted()) {
            inst->setResultReady();
            int dependents = wakeDependents(inst);
            if (dependents) {
                producerInst[0]++;
                consumerInst[0]+= dependents;
            }
        }
    }

    writeback.push_back(inst);

    numInstsToWB[0]++;

    writebackCount[0]++;
}

template <class Impl>
void
LWBackEnd<Impl>::readyInstsForCommit()
{
    for (int i = numInstsToWB[-latency];
         !writeback.empty() && i;
         --i)
    {
        DynInstPtr inst = writeback.front();
        writeback.pop_front();
        if (!inst->isSquashed()) {
            DPRINTF(BE, "Writing back instruction [sn:%lli] PC %#x.\n",
                    inst->seqNum, inst->readPC());

            inst->setCanCommit();
        }
    }
}

#if 0
template <class Impl>
void
LWBackEnd<Impl>::writebackInsts()
{
    int wb_width = wbWidth;
    // Using this method I'm not quite sure how to prevent an
    // instruction from waking its own dependents multiple times,
    // without the guarantee that commit always has enough bandwidth
    // to accept all instructions being written back.  This guarantee
    // might not be too unrealistic.
    InstListIt wb_inst_it = writeback.begin();
    InstListIt wb_end_it = writeback.end();
    int inst_num = 0;
    int consumer_insts = 0;

    for (; inst_num < wb_width &&
             wb_inst_it != wb_end_it; inst_num++) {
        DynInstPtr inst = (*wb_inst_it);

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Uncached loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the uncached load.
        if (!inst->isSquashed()) {
            DPRINTF(BE, "Writing back instruction [sn:%lli] PC %#x.\n",
                    inst->seqNum, inst->readPC());

            inst->setCanCommit();
            inst->setResultReady();

            if (inst->isExecuted()) {
                int dependents = wakeDependents(inst);
                if (dependents) {
                    producer_inst[0]++;
                    consumer_insts+= dependents;
                }
            }
        }

        writeback.erase(wb_inst_it++);
    }
    LSQ.writebackStores();
    consumer_inst[0]+= consumer_insts;
    writeback_count[0]+= inst_num;
}
#endif
template <class Impl>
bool
LWBackEnd<Impl>::commitInst(int inst_num)
{
    // Read instruction from the head of the ROB
    DynInstPtr inst = instList.back();

    // Make sure instruction is valid
    assert(inst);

    if (!inst->readyToCommit())
        return false;

    DPRINTF(BE, "Trying to commit instruction [sn:%lli] PC:%#x\n",
            inst->seqNum, inst->readPC());

    thread->setPC(inst->readPC());
    thread->setNextPC(inst->readNextPC());
    inst->setAtCommit();

    // If the instruction is not executed yet, then it is a non-speculative
    // or store inst.  Signal backwards that it should be executed.
    if (!inst->isExecuted()) {
        if (inst->isNonSpeculative() ||
            (inst->isStoreConditional() && inst->getFault() == NoFault) ||
            inst->isMemBarrier() ||
            inst->isWriteBarrier()) {
            if ((inst->isMemBarrier() || inst->isWriteBarrier() ||
                    inst->isQuiesce()) && LSQ.hasStoresToWB())
            {
                DPRINTF(BE, "Waiting for all stores to writeback.\n");
                return false;
            }

            DPRINTF(BE, "Encountered a store or non-speculative "
                    "instruction at the head of the ROB, PC %#x.\n",
                    inst->readPC());

            if (inst->isMemBarrier() || inst->isWriteBarrier()) {
                DPRINTF(BE, "Waking dependents on barrier [sn:%lli]\n",
                        inst->seqNum);
                assert(memBarrier);
                wakeDependents(inst, true);
                if (memBarrier == inst)
                    memBarrier = NULL;
                inst->clearMemDependents();
            }

            // Send back the non-speculative instruction's sequence number.
            if (inst->iqItValid) {
                DPRINTF(BE, "Removing instruction from waiting list\n");
                waitingList.erase(inst->iqIt);
                inst->iqItValid = false;
                waitingInsts--;
                assert(waitingInsts >= 0);
                if (inst->isStore())
                    removeWaitingMemOp(inst);
            }

            exeList.push(inst);

            // Change the instruction so it won't try to commit again until
            // it is executed.
            inst->clearCanCommit();

//            ++commitNonSpecStalls;

            return false;
        } else if (inst->isLoad()) {
            DPRINTF(BE, "[sn:%lli]: Uncached load, PC %#x.\n",
                    inst->seqNum, inst->readPC());

            // Send back the non-speculative instruction's sequence
            // number.  Maybe just tell the lsq to re-execute the load.

            // Send back the non-speculative instruction's sequence number.
            if (inst->iqItValid) {
                DPRINTF(BE, "Removing instruction from waiting list\n");
                waitingList.erase(inst->iqIt);
                inst->iqItValid = false;
                waitingInsts--;
                assert(waitingInsts >= 0);
                removeWaitingMemOp(inst);
            }
            replayMemInst(inst);

            inst->clearCanCommit();

            return false;
        } else {
            panic("Trying to commit un-executed instruction "
                  "of unknown type!\n");
        }
    }

    // Not handled for now.
    assert(!inst->isThreadSync());
    assert(inst->memDepReady());
    // Stores will mark themselves as totally completed as they need
    // to wait to writeback to memory.  @todo: Hack...attempt to fix
    // having the checker be forced to wait until a store completes in
    // order to check all of the instructions.  If the store at the
    // head of the check list misses, but a later store hits, then
    // loads in the checker may see the younger store values instead
    // of the store they should see.  Either the checker needs its own
    // memory (annoying to update), its own store buffer (how to tell
    // which value is correct?), or something else...
    if (!inst->isStore()) {
        inst->setCompleted();
    }
    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = inst->getFault();

    // Use checker prior to updating anything due to traps or PC
    // based events.
    if (checker) {
        checker->verify(inst);
    }

    if (inst_fault != NoFault) {
        DPRINTF(BE, "Inst [sn:%lli] PC %#x has a fault\n",
                inst->seqNum, inst->readPC());

        // Instruction is completed as it has a fault.
        inst->setCompleted();

        if (LSQ.hasStoresToWB()) {
            DPRINTF(BE, "Stores still in flight, will wait until drained.\n");
            return false;
        } else if (inst_num != 0) {
            DPRINTF(BE, "Will wait until instruction is head of commit group.\n");
            return false;
        }
        else if (checker && inst->isStore()) {
            checker->verify(inst);
        }

        handleFault(inst_fault);
        return false;
    }

    int freed_regs = 0;

    for (int i = 0; i < inst->numDestRegs(); ++i) {
        DPRINTF(BE, "Commit rename map setting reg %i to [sn:%lli]\n",
                (int)inst->destRegIdx(i), inst->seqNum);
        thread->renameTable[inst->destRegIdx(i)] = inst;
        ++freed_regs;
    }

    if (FullSystem && thread->profile) {
        thread->profilePC = inst->readPC();
        ProfileNode *node = thread->profile->consume(thread->getTC(),
                                                     inst->staticInst);

        if (node)
            thread->profileNode = node;
    }

    if (inst->traceData) {
        inst->traceData->setFetchSeq(inst->seqNum);
        inst->traceData->setCPSeq(thread->numInst);
        inst->traceData->finalize();
        inst->traceData = NULL;
    }

    inst->clearDependents();

    frontEnd->addFreeRegs(freed_regs);

    instList.pop_back();

    --numInsts;
    ++thread->funcExeInst;
    // Maybe move this to where the fault is handled; if the fault is
    // handled, don't try to set this myself as the fault will set it.
    // If not, then I set thread->PC = thread->nextPC and
    // thread->nextPC = thread->nextPC + 4.
    thread->setPC(thread->readNextPC());
    thread->setNextPC(thread->readNextPC() + sizeof(TheISA::MachInst));
    updateComInstStats(inst);

    // Write the done sequence number here.
    toIEW->doneSeqNum = inst->seqNum;
    lastCommitCycle = curTick();

    if (FullSystem) {
        int count = 0;
        Addr oldpc;
        do {
            if (count == 0)
                assert(!thread->inSyscall && !thread->trapPending);
            oldpc = thread->readPC();
            cpu->system->pcEventQueue.service(
                thread->getTC());
            count++;
        } while (oldpc != thread->readPC());
        if (count > 1) {
            DPRINTF(BE, "PC skip function event, stopping commit\n");
            tcSquash = true;
            return false;
        }
    }
    return true;
}

template <class Impl>
void
LWBackEnd<Impl>::commitInsts()
{
    // Not sure this should be a loop or not.
    int inst_num = 0;
    while (!instList.empty() && inst_num < commitWidth) {
        if (instList.back()->isSquashed()) {
            instList.back()->clearDependents();
            ROBSquashedInsts[instList.back()->threadNumber]++;
            instList.pop_back();
            --numInsts;
            continue;
        }

        if (!commitInst(inst_num++)) {
            DPRINTF(BE, "Can't commit, Instruction [sn:%lli] PC "
                    "%#x is head of ROB and not ready\n",
                    instList.back()->seqNum, instList.back()->readPC());
            --inst_num;
            break;
        }
    }
    nCommittedDist.sample(inst_num);
}

template <class Impl>
void
LWBackEnd<Impl>::squash(const InstSeqNum &sn)
{
    LSQ.squash(sn);

    int freed_regs = 0;
    InstListIt insts_end_it = waitingList.end();
    InstListIt insts_it = waitingList.begin();

    while (insts_it != insts_end_it && (*insts_it)->seqNum > sn)
    {
        if ((*insts_it)->isSquashed()) {
            ++insts_it;
            continue;
        }
        DPRINTF(BE, "Squashing instruction on waitingList PC %#x, [sn:%lli].\n",
                (*insts_it)->readPC(),
                (*insts_it)->seqNum);

        if ((*insts_it)->isMemRef()) {
            DPRINTF(BE, "Squashing a waiting mem op [sn:%lli]\n",
                    (*insts_it)->seqNum);
            removeWaitingMemOp((*insts_it));
        }

        waitingList.erase(insts_it++);
        waitingInsts--;
    }
    assert(waitingInsts >= 0);

    insts_it = instList.begin();

    while (!instList.empty() && (*insts_it)->seqNum > sn)
    {
        if ((*insts_it)->isSquashed()) {
            panic("Instruction should not be already squashed and on list!");
            ++insts_it;
            continue;
        }
        DPRINTF(BE, "Squashing instruction on inst list PC %#x, [sn:%lli].\n",
                (*insts_it)->readPC(),
                (*insts_it)->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*insts_it)->setSquashed();

        (*insts_it)->setCanCommit();

        (*insts_it)->clearInROB();

        for (int i = 0; i < (*insts_it)->numDestRegs(); ++i) {
            DynInstPtr prev_dest = (*insts_it)->getPrevDestInst(i);
            DPRINTF(BE, "Commit rename map setting reg %i to [sn:%lli]\n",
                    (int)(*insts_it)->destRegIdx(i), prev_dest->seqNum);
            renameTable[(*insts_it)->destRegIdx(i)] = prev_dest;
            ++freed_regs;
        }

        (*insts_it)->clearDependents();

        squashedInsts[(*insts_it)->threadNumber]++;

        instList.erase(insts_it++);
        --numInsts;
    }

    while (memBarrier && memBarrier->seqNum > sn) {
        DPRINTF(BE, "[sn:%lli] Memory barrier squashed (or previously "
                "squashed)\n", memBarrier->seqNum);
        memBarrier->clearMemDependents();
        if (memBarrier->memDepReady()) {
            DPRINTF(BE, "No previous barrier\n");
            memBarrier = NULL;
        } else {
            std::list<DynInstPtr> &srcs = memBarrier->getMemSrcs();
            memBarrier = srcs.front();
            srcs.pop_front();
            assert(srcs.empty());
            DPRINTF(BE, "Previous barrier: [sn:%lli]\n",
                    memBarrier->seqNum);
        }
    }

    insts_it = replayList.begin();
    insts_end_it = replayList.end();
    while (!replayList.empty() && insts_it != insts_end_it) {
        if ((*insts_it)->seqNum < sn) {
            ++insts_it;
            continue;
        }
        assert((*insts_it)->isSquashed());

        replayList.erase(insts_it++);
    }

    frontEnd->addFreeRegs(freed_regs);
}

template <class Impl>
void
LWBackEnd<Impl>::squashFromTC()
{
    InstSeqNum squashed_inst = robEmpty() ? 0 : instList.back()->seqNum - 1;
    squash(squashed_inst);
    frontEnd->squash(squashed_inst, thread->readPC(),
                     false, false);
    frontEnd->interruptPending = false;

    thread->trapPending = false;
    thread->inSyscall = false;
    tcSquash = false;
    commitStatus = Running;
}

template <class Impl>
void
LWBackEnd<Impl>::squashFromTrap()
{
    InstSeqNum squashed_inst = robEmpty() ? 0 : instList.back()->seqNum - 1;
    squash(squashed_inst);
    frontEnd->squash(squashed_inst, thread->readPC(),
                     false, false);
    frontEnd->interruptPending = false;

    thread->trapPending = false;
    thread->inSyscall = false;
    trapSquash = false;
    commitStatus = Running;
}

template <class Impl>
void
LWBackEnd<Impl>::squashDueToBranch(DynInstPtr &inst)
{
    // Update the branch predictor state I guess
    DPRINTF(BE, "Squashing due to branch [sn:%lli], will restart at PC %#x\n",
            inst->seqNum, inst->readNextPC());
    squash(inst->seqNum);
    frontEnd->squash(inst->seqNum, inst->readNextPC(),
                     true, inst->mispredicted());
}

template <class Impl>
void
LWBackEnd<Impl>::squashDueToMemViolation(DynInstPtr &inst)
{
    // Update the branch predictor state I guess
    DPRINTF(BE, "Squashing due to violation [sn:%lli], will restart at PC %#x\n",
            inst->seqNum, inst->readNextPC());
    squash(inst->seqNum);
    frontEnd->squash(inst->seqNum, inst->readNextPC(),
                     false, inst->mispredicted());
}

template <class Impl>
void
LWBackEnd<Impl>::squashDueToMemBlocked(DynInstPtr &inst)
{
    DPRINTF(IEW, "Memory blocked, squashing load and younger insts, "
            "PC: %#x [sn:%i].\n", inst->readPC(), inst->seqNum);

    squash(inst->seqNum - 1);
    frontEnd->squash(inst->seqNum - 1, inst->readPC());
}

template <class Impl>
void
LWBackEnd<Impl>::switchOut()
{
    switchPending = true;
}

template <class Impl>
void
LWBackEnd<Impl>::doSwitchOut()
{
    switchedOut = true;
    switchPending = false;
    // Need to get rid of all committed, non-speculative state and write it
    // to memory/TC.  In this case this is stores that have committed and not
    // yet written back.
    assert(robEmpty());
    assert(!LSQ.hasStoresToWB());
    writeback.clear();
    for (int i = 0; i < numInstsToWB.getSize() + 1; ++i)
        numInstsToWB.advance();

//    squash(0);
    assert(waitingList.empty());
    assert(instList.empty());
    assert(replayList.empty());
    assert(writeback.empty());
    LSQ.switchOut();
}

template <class Impl>
void
LWBackEnd<Impl>::takeOverFrom(ThreadContext *old_tc)
{
    assert(!squashPending);
    squashSeqNum = 0;
    squashNextPC = 0;
    tcSquash = false;
    trapSquash = false;

    numInsts = 0;
    numWaitingMemOps = 0;
    waitingMemOps.clear();
    waitingInsts = 0;
    switchedOut = false;
    dispatchStatus = Running;
    commitStatus = Running;
    LSQ.takeOverFrom(old_tc);
}

template <class Impl>
void
LWBackEnd<Impl>::updateExeInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    exeInst[tid]++;

    //
    //  Control operations
    //
    if (inst->isControl())
        exeBranches[tid]++;

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        exeRefs[tid]++;

        if (inst->isLoad())
            exeLoads[tid]++;
    }
}

template <class Impl>
void
LWBackEnd<Impl>::updateComInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    // keep an instruction count
    thread->numInst++;
    thread->numInsts++;

    cpu->numInst++;
    //
    //  Pick off the software prefetches
    //
#ifdef TARGET_ALPHA
    if (inst->isDataPrefetch()) {
        statComSwp[tid]++;
    } else {
        statComInst[tid]++;
    }
#else
    statComInst[tid]++;
#endif

    //
    //  Control Instructions
    //
    if (inst->isControl())
        statComBranches[tid]++;

    //
    //  Memory references
    //
    if (inst->isMemRef()) {
        statComRefs[tid]++;

        if (inst->isLoad()) {
            statComLoads[tid]++;
        }
    }

    if (inst->isMemBarrier()) {
        statComMembars[tid]++;
    }
}

template <class Impl>
void
LWBackEnd<Impl>::dumpInsts()
{
    int num = 0;
    int valid_num = 0;

    InstListIt inst_list_it = --(instList.end());

    cprintf("ExeList size: %i\n", exeList.size());

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

        inst_list_it--;
        ++num;
    }

    inst_list_it = --(writeback.end());

    cprintf("Writeback list size: %i\n", writeback.size());

    while (inst_list_it != writeback.end())
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

        inst_list_it--;
        ++num;
    }

    cprintf("Waiting list size: %i\n", waitingList.size());

    inst_list_it = --(waitingList.end());

    while (inst_list_it != waitingList.end())
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

        inst_list_it--;
        ++num;
    }

    cprintf("waitingMemOps list size: %i\n", waitingMemOps.size());

    MemIt waiting_it = waitingMemOps.begin();

    while (waiting_it != waitingMemOps.end())
    {
        cprintf("[sn:%lli] ", (*waiting_it));
        waiting_it++;
        ++num;
    }
    cprintf("\n");
}
