/*
 * Copyright (c) 2013-2014,2018-2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "cpu/minor/writeback.hh"

#include <functional>

#include "cpu/minor/cpu.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/minor/fetch1.hh"
#include "cpu/minor/lsq.hh"
#include "cpu/op_class.hh"
#include "debug/Activity.hh"
#include "debug/Branch.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/MinorWriteback.hh"
#include "debug/MinorInterrupt.hh"
#include "debug/MinorMem.hh"
#include "debug/MinorTrace.hh"
#include "debug/PCEvent.hh"
#include "debug/MinorGUI.hh"

namespace gem5
{

    GEM5_DEPRECATED_NAMESPACE(Minor, minor);
    namespace minor
    {

        Writeback::Writeback(const std::string &name_,
                             MinorCPU &cpu_,
                             const BaseMinorCPUParams &params,
                             Latch<ForwardInstData>::Output inp_,
                             Latch<BranchData>::Input out_,
                             Latch<BranchData>::Output execute_branch_) : Named(name_),
                                                                          inp(inp_),
                                                                          out(out_),
                                                                          execute_branch(execute_branch_),
                                                                          cpu(cpu_),
                                                                          commitLimit(1),
                                                                          memoryCommitLimit(params.executeMemoryCommitLimit),
                                                                          processMoreThanOneInput(params.executeCycleInput),
                                                                          setTraceTimeOnCommit(params.executeSetTraceTimeOnCommit),
                                                                          writebackInfo(params.numThreads,
                                                                                        WritebackThreadInfo(params.executeCommitLimit)),
                                                                          interruptPriority(0),
                                                                          commitPriority(0)
        {
            if (commitLimit < 1)
            {
                fatal("%s: executeCommitLimit must be >= 1 (%d)\n", name_,
                      commitLimit);
            }

            if (memoryCommitLimit > commitLimit)
            {
                fatal("%s: executeMemoryCommitLimit (%d) must be <="
                      " executeCommitLimit (%d)\n",
                      name_, memoryCommitLimit, commitLimit);
            }

            if (params.executeInputBufferSize < 1)
            {
                fatal("%s: executeInputBufferSize must be >= 1 (%d)\n", name_,
                      params.executeInputBufferSize);
            }

            if (params.executeInputBufferSize < 1)
            {
                fatal("%s: executeInputBufferSize must be >= 1 (%d)\n", name_,
                      params.executeInputBufferSize);
            }

            /* Per-thread structures */
            for (ThreadID tid = 0; tid < params.numThreads; tid++)
            {
                std::string tid_str = std::to_string(tid);

                /* Input Buffers */
                inputBuffer.push_back(
                    InputBuffer<ForwardInstData>(
                        name_ + ".inputBuffer" + tid_str, "insts",
                        params.executeInputBufferSize));
            }
        }

        const ForwardInstData *
        Writeback::getInput(ThreadID tid)
        {
            /* Get a line from the inputBuffer to work with */
            if (!inputBuffer[tid].empty())
            {
                const ForwardInstData &head = inputBuffer[tid].front();

                return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
            }
            else
            {
                return NULL;
            }
        }

        void
        Writeback::popInput(ThreadID tid)
        {
            DPRINTF(MinorWriteback, "Popping inputBuffer[%d]\n", tid);
            if (!inputBuffer[tid].empty())
                inputBuffer[tid].pop();

            writebackInfo[tid].inputIndex = 0;
        }

        bool
        Writeback::isInterrupted(ThreadID thread_id) const
        {
            return cpu.checkInterrupts(thread_id);
        }

        /** Increment a cyclic buffer index for indices [0, cycle_size-1] */
        inline unsigned int
        cyclicIndexInc(unsigned int index, unsigned int cycle_size)
        {
            unsigned int ret = index + 1;

            if (ret == cycle_size)
                ret = 0;

            return ret;
        }

        /** Decrement a cyclic buffer index for indices [0, cycle_size-1] */
        inline unsigned int
        cyclicIndexDec(unsigned int index, unsigned int cycle_size)
        {
            int ret = index - 1;

            if (ret < 0)
                ret = cycle_size - 1;

            return ret;
        }

        void
        Writeback::doInstCommitAccounting(MinorDynInstPtr inst)
        {
            assert(!inst->isFault());

            MinorThread *thread = cpu.threads[inst->id.threadId];

            /* Increment the many and various inst and op counts in the
             *  thread and system */
            if (!inst->staticInst->isMicroop() || inst->staticInst->isLastMicroop())
            {
                thread->numInst++;
                thread->threadStats.numInsts++;
                cpu.stats.numInsts++;

                /* Act on events related to instruction counts */
                thread->comInstEventQueue.serviceEvents(thread->numInst);
            }
            thread->numOp++;
            thread->threadStats.numOps++;
            cpu.stats.numOps++;
            cpu.stats.committedInstType[inst->id.threadId]
                                       [inst->staticInst->opClass()]++;

            /** Add a count for every control instruction */
            if (inst->staticInst->isControl())
            {
                if (inst->staticInst->isReturn())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsReturn]++;
                }
                if (inst->staticInst->isCall())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsCall]++;
                }
                if (inst->staticInst->isDirectCtrl())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsDirectControl]++;
                }
                if (inst->staticInst->isIndirectCtrl())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsIndirectControl]++;
                }
                if (inst->staticInst->isCondCtrl())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsCondControl]++;
                }
                if (inst->staticInst->isUncondCtrl())
                {
                    cpu.stats.committedControl[inst->id.threadId]
                                              [gem5::StaticInstFlags::Flags::IsUncondControl]++;
                }
                cpu.stats.committedControl[inst->id.threadId]
                                          [gem5::StaticInstFlags::Flags::IsControl]++;
            }

            /* Set the CP SeqNum to the numOps commit number */
            if (inst->traceData)
                inst->traceData->setCPSeq(thread->numOp);

            cpu.probeInstCommit(inst->staticInst, inst->pc->instAddr());
        }

        void Writeback::actuallyExecuteInst(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, bool &committed)
        {
            if (!inst->executed && !inst->committed)
            {
                ExecContext context(cpu, *cpu.threads[thread_id], inst);
                context.writeback(inst->staticInst);
            }
            else
            {
                DPRINTF(MinorWriteback, "Not executing inst %s, it should have been already executed\n", *inst);
            }

            committed = true;
        }

        bool
        Writeback::commitInst(MinorDynInstPtr inst, Fault &fault, bool &committed)
        {
            ThreadID thread_id = inst->id.threadId;
            ThreadContext *thread = cpu.getContext(thread_id);

            bool completed_inst = true;
            fault = NoFault;

            /* Is the thread for this instruction suspended?  In that case, just
             *  stall as long as there are no pending interrupts */
            if (thread->status() == ThreadContext::Suspended &&
                !isInterrupted(thread_id))
            {
                panic("We should never hit the case where we try to commit from a "
                      "suspended thread as the streamSeqNum should not match");
            }
            else if (inst->isFault())
            {
                ExecContext context(cpu, *cpu.threads[thread_id], inst);

                DPRINTF(MinorWriteback, "Fault inst reached Writeback: %s\n",
                        inst->fault->name());

                fault = inst->fault;
            }
            else
            {
                DPRINTF(MinorWriteback, "Committing inst: %s\n", *inst);

                actuallyExecuteInst(thread_id, inst, fault, thread, committed);

                doInstCommitAccounting(inst);
            }

            return completed_inst;
        }

        void
        Writeback::doCommitAccounting(MinorDynInstPtr const inst, WritebackThreadInfo &wb_info, unsigned int &num_insts_committed)
        {
            bool is_no_cost_inst = inst->isNoCostInst();

            /* Don't show no cost instructions as having taken a commit
             *  slot */
            if (debug::MinorTrace && !is_no_cost_inst)
                wb_info.instsBeingCommitted.insts[num_insts_committed] = inst;

            if (!is_no_cost_inst)
                num_insts_committed++;

            if (num_insts_committed == commitLimit)
                DPRINTF(MinorWriteback, "Reached inst commit limit\n");

            /* Re-set the time of the instruction if that's required for
             * tracing */
            if (inst->traceData)
            {
                if (setTraceTimeOnCommit)
                    inst->traceData->setWhen(curTick());
                inst->traceData->dump();
            }
        }

        void
        Writeback::commit(ThreadID thread_id,
                          bool only_commit_microops /* true if only microops should be committed,
                                                        e.g. when an interrupt has occurred. This
                                                        avoids having partially executed instructions */
        )
        {
            Fault fault = NoFault;
            const ForwardInstData *insts_in = getInput(thread_id);
            WritebackThreadInfo &wb_info = writebackInfo[thread_id];

            if (!insts_in)
                return;

            /**
             * Try and execute as many instructions from the end of FU pipelines as
             *  possible.  This *doesn't* include actually advancing the pipelines.
             *
             * We do this by looping on the front of the inFlightInsts queue for as
             *  long as we can find the desired instruction at the end of the
             *  functional unit it was issued to without seeing a branch or a fault.
             *  In this function, these terms are used:
             *      complete -- The instruction has finished its passage through
             *          its functional unit and its fate has been decided
             *          (committed, discarded, issued to the memory system)
             *      commit -- The instruction is complete(d), not discarded and has
             *          its effects applied to the CPU state
             *      discard(ed) -- The instruction is complete but not committed
             *          as its streamSeqNum disagrees with the current
             *          Writeback::streamSeqNum
             *
             *  Commits are also possible from two other places:
             *
             *  1) Responses returning from the LSQ
             *  2) Mem ops issued to the LSQ ('committed' from the FUs) earlier
             *      than their position in the inFlightInsts queue, but after all
             *      their dependencies are resolved.
             */

            /* Has an instruction been completed?  Once this becomes false, we stop
             *  trying to complete instructions. */
            bool completed_inst = true;

            /* Number of insts committed this cycle to check against commitLimit */
            unsigned int num_insts_committed = 0;

            if (only_commit_microops && insts_in)
            {
                DPRINTF(MinorInterrupt, "Only commit microops %s %d\n",
                        *(insts_in->insts[0]),
                        wb_info.lastCommitWasEndOfMacroop);
            }

            while (insts_in && wb_info.inputIndex < insts_in->width() && /* Some more instructions to process */
                   fault == NoFault &&                                   /* No faults */
                   completed_inst &&                                     /* Still finding instructions to execute */
                   num_insts_committed != commitLimit                    /* Not reached commit limit */
            )
            {
                if (only_commit_microops)
                {
                    DPRINTF(MinorInterrupt, "Committing tail of insts before"
                                            " interrupt: %s\n",
                            *(insts_in->insts[0]));
                }

                MinorDynInstPtr inst = insts_in->insts[wb_info.inputIndex];

                /* The instruction we actually process if completed_inst
                 *  remains true to the end of the loop body.
                 *  Start by considering the the head of the in flight insts queue */

                bool committed_inst = false;
                bool discard_inst = false;

                /* Must set this again to go around the loop */
                completed_inst = false;

                /* If we're just completing a macroop before an interrupt or drain,
                 *  can we stil commit another microop (rather than a memory response)
                 *  without crosing into the next full instruction? */
                bool finished_macroop = only_commit_microops && wb_info.lastCommitWasEndOfMacroop;

                bool can_commit_insts = insts_in && !finished_macroop;

                DPRINTF(MinorWriteback, "Trying to commit canCommitInsts: %d\n",
                        can_commit_insts);

                if (can_commit_insts)
                {
                    /* Try and commit FU-less insts */
                    if (!completed_inst && inst->isNoCostInst())
                    {
                        DPRINTF(MinorWriteback, "Committing no cost inst: %s", *inst);

                        completed_inst = true;
                    }
                    else if (!completed_inst && inst->isMemRef())
                    {
                        DPRINTF(MinorWriteback, "Committing memory reference instruction: %s", *inst);
                    }

                    discard_inst = false; // inst->id.streamSeqNum != wb_info.streamSeqNum;

                    /* Is this instruction discardable as its streamSeqNum
                     *  doesn't match? */
                    if (!discard_inst) // && !inst->isMemRef())
                    {
                        /* @todo Think about making lastMemBarrier be
                         *  MAX_UINT_64 to avoid using 0 as a marker value */
                        completed_inst = commitInst(inst, fault,
                                                    committed_inst);
                    }
                    else
                    {
                        /* Discard instruction */
                        completed_inst = true;
                    }
                }
                else
                {
                    DPRINTF(MinorWriteback, "No instructions to commit\n");
                    completed_inst = false;
                }

                /* All discardable instructions must also be 'completed' by now */
                assert(!(discard_inst && !completed_inst));

                /* Instruction committed but was discarded due to streamSeqNum
                 *  mismatch */
                if (discard_inst)
                {
                    DPRINTF(MinorWriteback, "Discarding inst: %s as its stream"
                                            " state was unexpected, expected: %d\n",
                            *inst, wb_info.streamSeqNum);

                    if (fault == NoFault)
                        cpu.stats.numDiscardedOps++;
                }

                if (completed_inst)
                {
                    DPRINTF(MinorGUI, "Log4GUI: writeback: %d: %d: %x: %s\n",
                            curTick(),
                            false,
                            inst ? inst->pc->instAddr() : 0x0,
                            inst ? inst->staticInst->disassemble(inst->pc->instAddr()) : "");
                    /* Note that this includes discarded insts */
                    DPRINTF(MinorWriteback, "Completed inst: %s\n", *inst);
                    DPRINTF(MinorGUI, "REGISTERS\n");
                    for (auto &rf : inst->getRegsAfterExecution())
                        rf.dump();
                    // Print registers
                    ExecContext context(cpu, *cpu.threads[thread_id], inst);
                    /* Got to the end of a full instruction? */
                    wb_info.lastCommitWasEndOfMacroop = inst->isFault() ||
                                                        inst->isLastOpInInst();
                    wb_info.inputIndex++;
                }

                /* Handle per-cycle instruction counting */
                if (committed_inst)
                {
                    doCommitAccounting(inst, wb_info, num_insts_committed);
                }

                if (wb_info.inputIndex == insts_in->width())
                {
                    popInput(thread_id);
                    insts_in = NULL;
                }
            }
        }

        bool
        Writeback::isInbetweenInsts(ThreadID thread_id) const
        {
            return writebackInfo[thread_id].lastCommitWasEndOfMacroop; // &&
                                                                       //! lsq.accessesInFlight();
        }

        void
        Writeback::changeStream(const BranchData &branch)
        {
            WritebackThreadInfo &thread = writebackInfo[branch.threadId];

            updateExpectedSeqNums(branch);

            /* Start fetching again if we were stopped */
            DPRINTF(MinorWriteback, "Changing stream on branch: %s\n", branch);
        }

        void
        Writeback::updateExpectedSeqNums(const BranchData &branch)
        {
            WritebackThreadInfo &thread = writebackInfo[branch.threadId];

            DPRINTF(MinorWriteback, "Updating streamSeqNum from: %d to %d,",
                    thread.streamSeqNum, branch.newStreamSeqNum);

            /* Change the stream */
            thread.streamSeqNum = branch.newStreamSeqNum;
        }

        void
        Writeback::processBranchesFromLaterStages(const BranchData &branch_in)
        {
            bool execute_thread_valid = branch_in.threadId != InvalidThreadID;

            /** Are both branches from later stages valid and for the same thread? */
            if (execute_thread_valid)
            {
                /* Are we changing stream?  Look to the Execute branches first, then
                 * to predicted changes of stream from Fetch2 */
                if (branch_in.isStreamChange())
                {
                    changeStream(branch_in);
                }
            }
        }

        void
        Writeback::evaluate()
        {
            if (!inp.outputWire->isBubble())
            {
                DPRINTF(MinorWriteback, "Received an instruction\n");
                inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);
            }
            bool is_stalling = false;
            MinorDynInstPtr log_inst = NULL;
            const BranchData &branch_in = *execute_branch.outputWire;

            processBranchesFromLaterStages(branch_in);

            ThreadID commit_tid = getCommittingThread();
            if (commit_tid != InvalidThreadID)
            {
                WritebackThreadInfo &commit_info = writebackInfo[commit_tid];

                DPRINTF(MinorWriteback, "Attempting to commit [tid:%d]\n",
                        commit_tid);
                /* commit can set stalled flags observable to issue and so *must* be
                 *  called first */
                if (commit_info.drainState != NotDraining)
                {
                    if (commit_info.drainState == DrainCurrentInst)
                    {
                        /* Commit only micro-ops, don't kill anything else */
                        commit(commit_tid, true);

                        if (isInbetweenInsts(commit_tid))
                            setDrainState(commit_tid, DrainHaltFetch);
                    }
                    else if (commit_info.drainState == DrainAllInsts)
                    {
                        /* Kill all instructions */
                        while (getInput(commit_tid))
                            popInput(commit_tid);
                    }
                }
                else
                {
                    /* Commit micro-ops only if interrupted.  Otherwise, commit
                     *  anything you like */
                    bool only_commit_microops = hasInterrupt(commit_tid);
                    if (only_commit_microops)
                        DPRINTF(MinorWriteback, "Committing micro-ops for interrupt[tid:%d]\n",
                                commit_tid);
                    commit(commit_tid, only_commit_microops);
                }

                /* Halt fetch, but don't do it until we have the current instruction in
                 *  the bag */
                if (commit_info.drainState == DrainHaltFetch)
                {
                    cpu.wakeupOnEvent(Pipeline::WritebackStageId);
                    setDrainState(commit_tid, DrainAllInsts);
                }
            }

            bool need_to_tick = false;
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                need_to_tick = need_to_tick || getInput(tid);
            }

            if (!need_to_tick)
            {
                DPRINTF(Activity, "The next cycle might be skippable as there are no"
                                  " instructions to commit\n");
            }

            /* Wake up if we need to tick again */
            if (need_to_tick)
                cpu.wakeupOnEvent(Pipeline::WritebackStageId);

            /* Make sure the input (if any left) is pushed */
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->threadId].pushTail();
        }

        bool
        Writeback::hasInterrupt(ThreadID thread_id)
        {
            if (FullSystem && cpu.getInterruptController(thread_id))
            {
                return writebackInfo[thread_id].drainState == NotDraining &&
                       isInterrupted(thread_id);
            }

            return false;
        }

        void
        Writeback::minorTrace() const
        {
            std::ostringstream insts;
            std::ostringstream stalled;

            writebackInfo[0].instsBeingCommitted.reportData(insts);
            inputBuffer[0].minorTrace();

            /* Report functional unit stalling in one string */
            minor::minorTrace("insts=%s inputIndex=%d streamSeqNum=%d"
                              " stalled=%s drainState=%d isInbetweenInsts=%d\n",
                              insts.str(), writebackInfo[0].inputIndex, writebackInfo[0].streamSeqNum,
                              stalled.str(), writebackInfo[0].drainState, isInbetweenInsts(0));
        }

        inline ThreadID
        Writeback::getCommittingThread()
        {
            std::vector<ThreadID> priority_list;

            switch (cpu.threadPolicy)
            {
            case enums::SingleThreaded:
                return 0;
            case enums::RoundRobin:
                priority_list = cpu.roundRobinPriority(commitPriority);
                break;
            case enums::Random:
                priority_list = cpu.randomPriority();
                break;
            default:
                panic("Invalid thread policy");
            }

            for (auto tid : priority_list)
            {
                if (getInput(tid))
                {
                    commitPriority = tid;
                    return tid;
                }
            }

            return InvalidThreadID;
        }

        void
        Writeback::drainResume()
        {
            DPRINTF(Drain, "MinorWriteback drainResume\n");

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                setDrainState(tid, NotDraining);
            }

            cpu.wakeupOnEvent(Pipeline::WritebackStageId);
        }

        std::ostream &operator<<(std::ostream &os, Writeback::DrainState state)
        {
            switch (state)
            {
            case Writeback::NotDraining:
                os << "NotDraining";
                break;
            case Writeback::DrainCurrentInst:
                os << "DrainCurrentInst";
                break;
            case Writeback::DrainHaltFetch:
                os << "DrainHaltFetch";
                break;
            case Writeback::DrainAllInsts:
                os << "DrainAllInsts";
                break;
            default:
                os << "Drain-" << static_cast<int>(state);
                break;
            }

            return os;
        }

        void
        Writeback::setDrainState(ThreadID thread_id, DrainState state)
        {
            DPRINTF(Drain, "setDrainState[%d]: %s\n", thread_id, state);
            writebackInfo[thread_id].drainState = state;
        }

        unsigned int
        Writeback::drain()
        {
            DPRINTF(Drain, "MinorWriteback drain\n");

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                if (writebackInfo[tid].drainState == NotDraining)
                {
                    cpu.wakeupOnEvent(Pipeline::WritebackStageId);

                    /* Go to DrainCurrentInst if we're between microops
                     * or waiting on an unbufferable memory operation.
                     * Otherwise we can go straight to DrainHaltFetch
                     */
                    if (isInbetweenInsts(tid))
                        setDrainState(tid, DrainHaltFetch);
                    else
                        setDrainState(tid, DrainCurrentInst);
                }
            }
            return (isDrained() ? 0 : 1);
        }

        bool
        Writeback::isDrained()
        {
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                if (!inputBuffer[tid].empty())
                {
                    return false;
                }
            }

            return true;
        }

        Writeback::~Writeback() {}

    } // namespace minor
} // namespace gem5
