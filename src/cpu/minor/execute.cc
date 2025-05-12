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

#include "cpu/minor/execute.hh"

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
#include "debug/MinorExecute.hh"
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

        Execute::Execute(const std::string &name_,
                         MinorCPU &cpu_,
                         const BaseMinorCPUParams &params,
                         Latch<ForwardInstData>::Output inp_,
                         Latch<BranchData>::Input out_,
                         Latch<BranchData>::Input branch_out_wb_,
                         std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer,
                         Latch<ForwardInstData>::Input insts_out_,
                         Latch<BranchData>::Input branch_out_mem_) : Named(name_),
                                                                     inp(inp_),
                                                                     out(out_),
                                                                     branch_out_wb(branch_out_wb_),
                                                                     branch_out_mem(branch_out_mem_),
                                                                     out_insts(insts_out_),
                                                                     cpu(cpu_),
                                                                     nextStageReserve(next_stage_input_buffer),
                                                                     outputWidth(1),
                                                                     processMoreThanOneInput(false),
                                                                     issueLimit(params.executeIssueLimit),
                                                                     memoryIssueLimit(params.executeMemoryIssueLimit),
                                                                     commitLimit(1), // params.executeCommitLimit),
                                                                     memoryCommitLimit(params.executeMemoryCommitLimit),
                                                                     fuDescriptions(*params.executeFuncUnits),
                                                                     numFuncUnits(fuDescriptions.funcUnits.size()),
                                                                     setTraceTimeOnCommit(params.executeSetTraceTimeOnCommit),
                                                                     setTraceTimeOnIssue(params.executeSetTraceTimeOnIssue),
                                                                     noCostFUIndex(fuDescriptions.funcUnits.size() + 1),
                                                                     executeInfo(params.numThreads,
                                                                                 ExecuteThreadInfo(params.executeCommitLimit)),
                                                                     interruptPriority(0),
                                                                     issuePriority(0),
                                                                     commitPriority(0)
        {
            if (commitLimit < 1)
            {
                fatal("%s: executeCommitLimit must be >= 1 (%d)\n", name_,
                      commitLimit);
            }

            if (issueLimit < 1)
            {
                fatal("%s: executeCommitLimit must be >= 1 (%d)\n", name_,
                      issueLimit);
            }

            if (memoryIssueLimit < 1)
            {
                fatal("%s: executeMemoryIssueLimit must be >= 1 (%d)\n", name_,
                      memoryIssueLimit);
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

            /* This should be large enough to count all the in-FU instructions
             *  which need to be accounted for in the inFlightInsts
             *  queue */
            unsigned int total_slots = 0;

            /* Make FUPipelines for each MinorFU */
            for (unsigned int i = 0; i < numFuncUnits; i++)
            {
                std::ostringstream fu_name;
                MinorFU *fu_description = fuDescriptions.funcUnits[i];

                /* Note the total number of instruction slots (for sizing
                 *  the inFlightInst queue) and the maximum latency of any FU
                 *  (for sizing the activity recorder) */
                total_slots += fu_description->opLat;

                fu_name << name_ << ".fu." << i;

                FUPipeline *fu = new FUPipeline(fu_name.str(), *fu_description, cpu);

                funcUnits.push_back(fu);
            }

            /** Check that there is a functional unit for all operation classes */
            for (int op_class = No_OpClass + 1; op_class < Num_OpClasses; op_class++)
            {
                bool found_fu = false;
                unsigned int fu_index = 0;

                while (fu_index < numFuncUnits && !found_fu)
                {
                    if (funcUnits[fu_index]->provides(
                            static_cast<OpClass>(op_class)))
                    {
                        found_fu = true;
                    }
                    fu_index++;
                }

                if (!found_fu)
                {
                    warn("No functional unit for OpClass %s\n",
                         enums::OpClassStrings[op_class]);
                }
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

                const auto &regClasses = cpu.threads[tid]->getIsaPtr()->regClasses();

                /* Scoreboards */
                scoreboard.emplace_back(name_ + ".scoreboard" + tid_str, regClasses);

                /* In-flight instruction records */
                executeInfo[tid].inFlightInsts = new Queue<QueuedInst,
                                                           ReportTraitsAdaptor<QueuedInst>>(
                    name_ + ".inFlightInsts" + tid_str, "insts", total_slots);

                executeInfo[tid].inFUMemInsts = new Queue<QueuedInst,
                                                          ReportTraitsAdaptor<QueuedInst>>(
                    name_ + ".inFUMemInsts" + tid_str, "insts", total_slots);
            }
        }

        const ForwardInstData *
        Execute::getInput(ThreadID tid)
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
        Execute::popInput(ThreadID tid)
        {
            if (!inputBuffer[tid].empty())
                inputBuffer[tid].pop();

            executeInfo[tid].inputIndex = 0;
        }

        void
        Execute::tryToBranch(MinorDynInstPtr inst, Fault fault, BranchData &branch)
        {
            ThreadContext *thread = cpu.getContext(inst->id.threadId);
            const std::unique_ptr<PCStateBase> pc_before(inst->pc->clone());
            std::unique_ptr<PCStateBase> target(thread->pcState().clone());

            /* Force a branch for SerializeAfter/SquashAfter instructions
             * at the end of micro-op sequence when we're not suspended */
            bool force_branch = thread->status() != ThreadContext::Suspended &&
                                !inst->isFault() &&
                                inst->isLastOpInInst() &&
                                (inst->staticInst->isSerializeAfter() ||
                                 inst->staticInst->isSquashAfter());

            DPRINTF(Branch, "tryToBranch before: %s after: %s%s\n",
                    *pc_before, *target, (force_branch ? " (forcing)" : ""));

            /* Will we change the PC to something other than the next instruction? */
            bool must_branch = //*pc_before != *target ||
                               fault != NoFault; // ||
                               //force_branch;

            /* The reason for the branch data we're about to generate, set below */
            BranchData::Reason reason = BranchData::NoBranch;

            if (fault == NoFault)
            {
                // Already done in decode
               /*  inst->staticInst->advancePC(*target);
                thread->pcState(*target);

                DPRINTF(Branch, "Advancing current PC from: %s to: %s\n",
                        *pc_before, *target); */ 
            }

            if (inst->predictedTaken && !force_branch)
            {
                /* Predicted to branch */
                if (!must_branch)
                {
                    /* No branch was taken, change stream to get us back to the
                     *  intended PC value */
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x but"
                                    " none happened inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(),
                            *inst);

                    reason = BranchData::BadlyPredictedBranch;
                }
                else if (*inst->predictedTarget == *target)
                {
                    /* Branch prediction got the right target, kill the branch and
                     *  carry on.
                     *  Note that this information to the branch predictor might get
                     *  overwritten by a "real" branch during this cycle */
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x correctly"
                                    " inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(),
                            *inst);

                    reason = BranchData::CorrectlyPredictedBranch;
                }
                else
                {
                    /* Branch prediction got the wrong target */
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x"
                                    " but got the wrong target (actual: 0x%x) inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(),
                            target->instAddr(), *inst);

                    reason = BranchData::BadlyPredictedBranchTarget;
                }
            }
            else if (must_branch)
            {
                /* Unpredicted branch */
                DPRINTF(Branch, "Unpredicted branch from 0x%x to 0x%x inst: %s\n",
                        inst->pc->instAddr(), target->instAddr(), *inst);

                reason = BranchData::UnpredictedBranch;
            }
            else
            {
                /* No branch at all */
                reason = BranchData::NoBranch;
            }

            updateBranchData(inst->id.threadId, reason, inst, *target, branch);
        }

        void
        Execute::updateBranchData(
            ThreadID tid,
            BranchData::Reason reason,
            MinorDynInstPtr inst, const PCStateBase &target,
            BranchData &branch)
        {
            if (reason != BranchData::NoBranch)
            {
                /* Bump up the stream sequence number on a real branch*/
                if (BranchData::isStreamChange(reason)) {
                    executeInfo[tid].streamSeqNum++;
                }

                /* Branches (even mis-predictions) don't change the predictionSeqNum,
                 *  just the streamSeqNum */
                branch = BranchData(reason, tid,
                                    executeInfo[tid].streamSeqNum,
                                    /* Maintaining predictionSeqNum if there's no inst is just a
                                     * courtesy and looks better on minorview */
                                    (inst->isBubble() ? executeInfo[tid].lastPredictionSeqNum
                                                      : inst->id.predictionSeqNum),
                                    target, inst);

                DPRINTF(Branch, "Branch data signalled: %s\n", branch);
            }
        }

        void
        Execute::handleMemResponse(MinorDynInstPtr inst,
                                   LSQ::LSQRequestPtr response, BranchData &branch, Fault &fault)
        {
            LSQ &lsq = cpu.getLSQ();
            ThreadID thread_id = inst->id.threadId;
            ThreadContext *thread = cpu.getContext(thread_id);

            ExecContext context(cpu, *cpu.threads[thread_id], inst);

            PacketPtr packet = response->packet;

            bool is_load = inst->staticInst->isLoad();
            bool is_store = inst->staticInst->isStore();
            bool is_atomic = inst->staticInst->isAtomic();
            bool is_prefetch = inst->staticInst->isDataPrefetch();

            /* If true, the trace's predicate value will be taken from the exec
             *  context predicate, otherwise, it will be set to false */
            bool use_context_predicate = true;

            if (inst->translationFault != NoFault)
            {
                /* Invoke memory faults. */
                DPRINTF(MinorMem, "Completing fault from DTLB access: %s\n",
                        inst->translationFault->name());

                if (inst->staticInst->isPrefetch())
                {
                    DPRINTF(MinorMem, "Not taking fault on prefetch: %s\n",
                            inst->translationFault->name());

                    /* Don't assign to fault */
                }
                else
                {
                    /* Take the fault raised during the TLB/memory access */
                    fault = inst->translationFault;

                    fault->invoke(thread, inst->staticInst);
                }
            }
            else if (!packet)
            {
                DPRINTF(MinorMem, "Completing failed request inst: %s\n",
                        *inst);
                use_context_predicate = false;
                if (!context.readMemAccPredicate())
                    inst->staticInst->completeAcc(nullptr, &context, inst->traceData);
            }
            else if (packet->isError())
            {
                DPRINTF(MinorMem, "Trying to commit error response: %s\n",
                        *inst);

                fatal("Received error response packet for inst: %s\n", *inst);
            }
            else if (is_store || is_load || is_prefetch || is_atomic)
            {
                assert(packet);

                DPRINTF(MinorMem, "Memory response inst: %s addr: 0x%x size: %d\n",
                        *inst, packet->getAddr(), packet->getSize());

                if (is_load && packet->getSize() > 0)
                {
                    DPRINTF(MinorMem, "Memory data[0]: 0x%x\n",
                            static_cast<unsigned int>(packet->getConstPtr<uint8_t>()[0]));
                }

                /* Complete the memory access instruction */
                fault = inst->staticInst->completeAcc(packet, &context,
                                                      inst->traceData);
                inst->executed = true;
                context.writeback(inst->staticInst);

                if (fault != NoFault)
                {
                    /* Invoke fault created by instruction completion */
                    DPRINTF(MinorMem, "Fault in memory completeAcc: %s\n",
                            fault->name());
                    fault->invoke(thread, inst->staticInst);
                }
                else
                {
                    /* Stores need to be pushed into the store buffer to finish
                     *  them off */
                    if (response->needsToBeSentToStoreBuffer())
                        lsq.sendStoreToStoreBuffer(response);
                }
            }
            else
            {
                fatal("There should only ever be reads, "
                      "writes or faults at this point\n");
            }

            lsq.popResponse(response);

            if (inst->traceData)
            {
                inst->traceData->setPredicate((use_context_predicate ? context.readPredicate() : false));
            }

            doInstCommitAccounting(inst);

            /* Generate output to account for branches */
            tryToBranch(inst, fault, branch);
        }

        bool
        Execute::isInterrupted(ThreadID thread_id) const
        {
            return cpu.checkInterrupts(thread_id);
        }

        bool
        Execute::takeInterrupt(ThreadID thread_id, BranchData &branch)
        {
            DPRINTF(MinorInterrupt, "Considering interrupt status from PC: %s\n",
                    cpu.getContext(thread_id)->pcState());

            Fault interrupt = cpu.getInterruptController(thread_id)->getInterrupt();

            if (interrupt != NoFault)
            {
                /* The interrupt *must* set pcState */
                cpu.getInterruptController(thread_id)->updateIntrInfo();
                interrupt->invoke(cpu.getContext(thread_id));

                assert(!cpu.getLSQ().accessesInFlight());

                DPRINTF(MinorInterrupt, "Invoking interrupt: %s to PC: %s\n",
                        interrupt->name(), cpu.getContext(thread_id)->pcState());

                /* Assume that an interrupt *must* cause a branch.  Assert this? */

                updateBranchData(thread_id, BranchData::Interrupt,
                                 MinorDynInst::bubble(), cpu.getContext(thread_id)->pcState(),
                                 branch);
            }

            return interrupt != NoFault;
        }

        bool
        Execute::executeMemRefInst(MinorDynInstPtr inst, BranchData &branch,
                                   bool &passed_predicate, Fault &fault)
        {
            bool issued = false;

            /* Set to true if the mem op. is issued and sent to the mem system */
            passed_predicate = false;

            if (!cpu.getLSQ().canRequest())
            {
                /* Not acting on instruction yet as the memory
                 * queues are full */
                issued = false;
            }
            else
            {
                ThreadContext *thread = cpu.getContext(inst->id.threadId);
                std::unique_ptr<PCStateBase> old_pc(thread->pcState().clone());

                ExecContext context(cpu, *cpu.threads[inst->id.threadId], inst);

                DPRINTF(MinorExecute, "Initiating memRef inst: %s\n", *inst);

                Fault init_fault = inst->staticInst->initiateAcc(&context,
                                                                 inst->traceData);

                if (inst->inLSQ)
                {
                    if (init_fault != NoFault)
                    {
                        assert(inst->translationFault != NoFault);
                        // Translation faults are dealt with in handleMemResponse()
                        init_fault = NoFault;
                    }
                    else
                    {
                        // If we have a translation fault then it got suppressed  by
                        // initateAcc()
                        inst->translationFault = NoFault;
                    }
                }

                if (init_fault != NoFault)
                {
                    DPRINTF(MinorExecute, "Fault on memory inst: %s"
                                          " initiateAcc: %s\n",
                            *inst, init_fault->name());
                    fault = init_fault;
                }
                else
                {
                    /* Only set this if the instruction passed its
                     * predicate */
                    if (!context.readMemAccPredicate())
                    {
                        DPRINTF(MinorMem, "No memory access for inst: %s\n", *inst);
                        assert(context.readPredicate());
                    }
                    passed_predicate = context.readPredicate();

                    /* Set predicate in tracing */
                    if (inst->traceData)
                        inst->traceData->setPredicate(passed_predicate);

                    /* If the instruction didn't pass its predicate
                     * or it is a predicated vector instruction and the
                     * associated predicate register is all-false (and so will not
                     * progress from here)  Try to branch to correct and branch
                     * mis-prediction. */
                    if (!inst->inLSQ)
                    {
                        /* Leave it up to commit to handle the fault */
                        cpu.getLSQ().pushFailedRequest(inst);
                        inst->inLSQ = true;
                    }
                }

                /* Restore thread PC */
                thread->pcState(*old_pc);
                issued = true;
            }

            return issued;
        }

        void
        Execute::issueNoCostInst(ThreadID thread_id, MinorDynInstPtr inst)
        {
            /* And start the countdown on activity to allow
             *  this instruction to get to the end of its FU */
            cpu.activityRecorder->activity();

            /* Mark the destinations for this instruction as
             *  busy .
             * DISABLE AS NOW IT IS DONE IN DECODE STAGE

             */
            // scoreboard[thread_id].markupInstDests(inst, cpu.curCycle() +
            //     Cycles(0), cpu.getContext(thread_id), false);

            DPRINTF(MinorExecute, "Issuing %s to %d\n", inst->id, noCostFUIndex);
            inst->fuIndex = noCostFUIndex;
            inst->extraCommitDelay = Cycles(0);
            inst->extraCommitDelayExpr = NULL;
        }

        void
        Execute::insertIntoFU(ThreadID thread_id, MinorDynInstPtr inst,
                              MinorFUTiming *timing, int fu_index)
        {
            LSQ &lsq = cpu.getLSQ();
            ExecuteThreadInfo &thread = executeInfo[thread_id];
            FUPipeline *fu = funcUnits[fu_index];
            Cycles extra_dest_retire_lat = Cycles(0);
            TimingExpr *extra_dest_retire_lat_expr = NULL;
            Cycles extra_assumed_lat = Cycles(0);

            /* Add the extraCommitDelay and extraAssumeLat to
             *  the FU pipeline timings */
            if (timing)
            {
                extra_dest_retire_lat =
                    timing->extraCommitLat;
                extra_dest_retire_lat_expr =
                    timing->extraCommitLatExpr;
                extra_assumed_lat =
                    timing->extraAssumedLat;
            }

            bool issued_mem_ref = inst->isMemRef();

            QueuedInst fu_inst(inst);

            /* Decorate the inst with FU details */
            inst->fuIndex = fu_index;
            inst->extraCommitDelay = extra_dest_retire_lat;
            inst->extraCommitDelayExpr =
                extra_dest_retire_lat_expr;
            fu_inst.just_issued = true;

            if (issued_mem_ref)
            {
                /* Also queue this instruction in the memory ref
                 *  queue to ensure in-order issue to the LSQ */
                DPRINTF(MinorExecute, "Pushing mem inst: %s\n",
                        *inst);
                thread.inFUMemInsts->push(fu_inst);
            }
            /* Issue to FU */
            fu->push(fu_inst);

            /* Mark the destinations for this instruction as
             *  busy.
             * DISABLE AS NOW IT IS DONE IN DECODE STAGE
             */
            // scoreboard[thread_id].markupInstDests(inst, cpu.curCycle() +
            //     fu->description.opLat +
            //     extra_dest_retire_lat +
            //     extra_assumed_lat,
            //     cpu.getContext(thread_id),
            //     issued_mem_ref && extra_assumed_lat == Cycles(0));

            /* Push the instruction onto the inFlight queue so
             *  it can be committed in order */
            thread.inFlightInsts->push(fu_inst);
        }

        bool
        Execute::canFUIssueInst(MinorDynInstPtr inst, FUPipeline *fu,
                                int fu_index)
        {
            bool ret = false;

            bool fu_is_capable = (!inst->isFault() ? fu->provides(inst->staticInst->opClass()) : true);

            if (!fu_is_capable)
            {
                DPRINTF(MinorExecute, "Can't issue as FU: %d isn't capable\n", fu_index);
            }
            else if (fu->alreadyPushed())
            {
                DPRINTF(MinorExecute, "Can't issue as FU: %d is already busy\n", fu_index);
            }
            // else if (fu->stalled)
            // {
            //     DPRINTF(MinorExecute, "Can't issue inst: %s into FU: %d, it's stalled\n", *inst, fu_index);
            // }
            else if (!fu->canInsert())
            {
                DPRINTF(MinorExecute, "Can't issue inst: %s to busy FU for another: %d cycles\n", *inst, fu->cyclesBeforeInsert());
            }
            else
            {
                ret = true;
            }
            return ret;
        }

        bool
        Execute::tryIssueInstruction(ThreadID thread_id,
                                     const MinorDynInstPtr &inst, unsigned int &fu_index,
                                     bool &discarded, bool &issued_mem_ref)
        {

            bool issued = false;
            ExecuteThreadInfo &thread = executeInfo[thread_id];

            if (inst->isBubble())
            {
                /* Skip */
                issued = true;
            }
            else if (cpu.getContext(thread_id)->status() ==
                     ThreadContext::Suspended)
            {

                DPRINTF(MinorExecute, "Discarding inst: %s from suspended thread\n", *inst);
                issued = true;
                discarded = true;
            }
            else if (inst->id.streamSeqNum != thread.streamSeqNum)
            {
                if (branchDelay != -1) {
                    branchDelay--;
                }
                if (branchDelay == 0) {
                    branchDelay = -1;
                }
                if (branchDelay == -1 || (branchDelay >= 0 && inst->id.streamSeqNum != delayStreamSeqNum))
                {
                    DPRINTF(MinorExecute, "Discarding inst: %s as its stream"
                                          " state was unexpected, expected: %d\n",
                            *inst, thread.streamSeqNum);
                    issued = true;
                    discarded = true;
                }
            }
            else
            {
                /* Try and issue an instruction into an FU, assume we didn't and
                 * fix that in the loop.
                 * Try FU from 0 each instruction.
                 * Try and issue a single instruction stepping through the
                 *  available FUs */
                for (fu_index = 0, issued = false;
                     fu_index != numFuncUnits && !issued;
                     fu_index++)
                {

                    FUPipeline *fu = funcUnits[fu_index];

                    DPRINTF(MinorExecute, "Trying to issue inst: %s to FU: %d\n",
                            *inst, fu_index);

                    if (inst->isNoCostInst())
                    {
                        /* Issue free insts. to a fake numbered FU */
                        fu_index = noCostFUIndex;
                        issueNoCostInst(thread_id, inst);

                        /* Push the instruction onto the inFlight queue so
                         *  it can be committed in order */
                        QueuedInst fu_inst(inst);
                        thread.inFlightInsts->push(fu_inst);

                        /* FORMAT >>> Log4GUI: stage: tick: stall_bit: inst_address:
                            ...assembly: fu_index*/
                        DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                curTick(),
                                0, /* not stalling */
                                inst->pc->instAddr(),
                                inst->staticInst->disassemble(inst->pc->instAddr()),
                                -1);

                        issued = true;
                    }
                    else if (canFUIssueInst(inst, fu, fu_index))
                    {
                        MinorFUTiming *timing = (!inst->isFault() ? fu->findTiming(inst->staticInst) : NULL);
                        /*
                         *
                         * const std::vector<Cycles> *src_latencies =
                         *  (timing ? &(timing->srcRegsRelativeLats)
                         *      : NULL);
                         *
                         * const std::vector<bool> *cant_forward_from_fu_indices =
                         *  &(fu->cantForwardFromFUIndices);                 *
                         */

                        if (timing && timing->suppress)
                        {
                            DPRINTF(MinorExecute, "Can't issue inst: %s as extra"
                                                  " decoding is suppressing it\n",
                                    *inst);
                        }
                        else
                        {
                            if (inst->isMemRef())
                            {
                                DPRINTF(MinorExecute, "Trying to issue mem inst: %s\n",
                                        *inst);
                            }
                            /* Can insert the instruction into this FU */
                            DPRINTF(MinorExecute, "Issuing inst: %s"
                                                  " into FU %d\n",
                                    *inst,
                                    fu_index);

                            insertIntoFU(thread_id, inst, timing, fu_index);
                            issued_mem_ref = inst->isMemRef();
                            /* And start the countdown on activity to allow
                             *  this instruction to get to the end of its FU */
                            cpu.activityRecorder->activity();

                            /* FORMAT >>> Log4GUI: stage: tick: stall_bit: inst_address:
                                ...assembly: fu_index*/
                            FUPipeline *fu = funcUnits[fu_index];
                            if (fu->opLatency(inst->staticInst->opClass()) == 1 || fu->occupancy == 1)
                            {
                                fu->advance();
                            }
                            if (!fu->m_was_stalled && fu->stalled) {
                                DPRINTF(MinorExecute, "Stalling fu %d in issue\n", fu_index);
                            }
                            DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                    curTick(),
                                    fu->m_was_stalled, /* not stalling */
                                    inst->pc->instAddr(),
                                    inst->staticInst->disassemble(inst->pc->instAddr()),
                                    fu_index);
                            DPRINTF(MinorExecute, "Advancing FU[%d]\n", fu_index);
                            issued = true;
                        }
                    }
                }
            }

            if (!issued)
            {
                /* FORMAT >>> Log4GUI: stage: tick: stall_bit: inst_address:
                    ...assembly: fu_index*/
                DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                        curTick(),
                        1, /* is stalling */
                        inst->pc->instAddr(),
                        inst->staticInst->disassemble(inst->pc->instAddr()),
                        -1);
            }
            // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
            //         curTick(),
            //         0, /* is stalling */
            //         inst->pc->instAddr(),
            //         inst->staticInst->disassemble(inst->pc->instAddr()),
            //         inst->fuIndex);

            return issued;
        }

        unsigned int
        Execute::issue(ThreadID thread_id, unsigned int &fu_idx)
        {
            const ForwardInstData *insts_in = getInput(thread_id);
            ExecuteThreadInfo &thread = executeInfo[thread_id];

            /* Early termination if we have no instructions */
            if (!insts_in)
                return 0;

            /* Start from the first FU */
            unsigned int fu_index = 0;

            /* Remains true while instructions are still being issued.  If any
             *  instruction fails to issue, this is set to false and we exit issue.
             *  This strictly enforces in-order issue.  For other issue behaviours,
             *  a more complicated test in the outer while loop below is needed. */
            bool issued = true;

            /* Number of insts issues this cycle to check for issueLimit */
            unsigned num_insts_issued = 0;

            /* Number of memory ops issues this cycle to check for memoryIssueLimit */
            unsigned num_mem_insts_issued = 0;

            /* Number of instructions discarded this cycle in order to enforce a
             *  discardLimit. @todo, add that parameter? */
            unsigned num_insts_discarded = 0;

            do
            {
                MinorDynInstPtr inst = insts_in->insts[thread.inputIndex];
                Fault fault = inst->fault;
                bool discarded = false;
                bool issued_mem_ref = false;

                /* Try to issue current instruction */
                issued = tryIssueInstruction(thread_id, inst,
                                             fu_index, discarded, issued_mem_ref);

                if (issued)
                {
                    fu_idx = fu_index;
                    /* Handle correctly issued, update simulation statistics */

                    /* Generate MinorTrace's MinorInst lines.  Do this at commit
                     *  to allow better instruction annotation? */
                    if (debug::MinorTrace && !inst->isBubble())
                    {
                        inst->minorTraceInst(*this);
                    }

                    /* Mark up barriers in the LSQ */
                    if (!discarded && inst->isInst() &&
                        inst->staticInst->isFullMemBarrier())
                    {
                        DPRINTF(MinorMem, "Issuing memory barrier inst: %s\n", *inst);
                        cpu.getLSQ().issuedMemBarrierInst(inst);
                    }

                    if (inst->traceData && setTraceTimeOnIssue)
                    {
                        inst->traceData->setWhen(curTick());
                    }

                    if (issued_mem_ref)
                        num_mem_insts_issued++;

                    if (discarded)
                    {
                        num_insts_discarded++;
                    }
                    else if (!inst->isBubble())
                    {
                        num_insts_issued++;

                        if (num_insts_issued == issueLimit)
                            DPRINTF(MinorExecute, "Reached inst issue limit\n");
                    }

                    thread.inputIndex++;
                    DPRINTF(MinorExecute, "Stepping to next inst inputIndex: %d\n",
                            thread.inputIndex);
                }
                else
                {
                    fu_idx = -1;
                    DPRINTF(MinorExecute, "Didn't issue inst: %s\n", *inst);
                }

                /* Got to the end of a line */
                if (thread.inputIndex == insts_in->width())
                {
                    popInput(thread_id);
                    /* Set insts_in to null to force us to leave the surrounding
                     *  loop */
                    insts_in = NULL;

                    if (processMoreThanOneInput)
                    {
                        DPRINTF(MinorExecute, "Wrapping\n");
                        insts_in = getInput(thread_id);
                    }
                }
            } while (insts_in && thread.inputIndex < insts_in->width() &&
                     /* We still have instructions */
                     fu_index != numFuncUnits &&       /* Not visited all FUs */
                     issued &&                         /* We've not yet failed to issue an instruction */
                     num_insts_issued != issueLimit && /* Still allowed to issue */
                     num_mem_insts_issued != memoryIssueLimit);

            return num_insts_issued;
        }

        bool
        Execute::tryPCEvents(ThreadID thread_id)
        {
            ThreadContext *thread = cpu.getContext(thread_id);
            unsigned int num_pc_event_checks = 0;

            /* Handle PC events on instructions */
            Addr oldPC;
            do
            {
                oldPC = thread->pcState().instAddr();
                cpu.threads[thread_id]->pcEventQueue.service(oldPC, thread);
                num_pc_event_checks++;
            } while (oldPC != thread->pcState().instAddr());

            if (num_pc_event_checks > 1)
            {
                DPRINTF(PCEvent, "Acting on PC Event to PC: %s\n",
                        thread->pcState());
            }

            return num_pc_event_checks > 1;
        }

        void
        Execute::doInstCommitAccounting(MinorDynInstPtr inst)
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

        void Execute::startMemRefExecution(MinorDynInstPtr inst, BranchData &branch, Fault &fault, gem5::ThreadContext *thread, bool early_memory_issue, bool &completed_inst, bool &completed_mem_issue)
        {
            /* Memory accesses are executed in two parts:
             *  executeMemRefInst -- calculates the EA and issues the access
             *      to memory.  This is done here.
             *  handleMemResponse -- handles the response packet, done by
             *      Execute::commit
             *
             *  While the memory access is in its FU, the EA is being
             *  calculated.  At the end of the FU, when it is ready to
             *  'commit' (in this function), the access is presented to the
             *  memory queues.  When a response comes back from memory,
             *  Execute::commit will commit it.
             */
            bool predicate_passed = false;
            bool completed_mem_inst = executeMemRefInst(inst, branch,
                                                        predicate_passed, fault);

            if (completed_mem_inst && fault != NoFault)
            {
                if (early_memory_issue)
                {
                    DPRINTF(MinorExecute, "Fault in early executing inst: %s\n",
                            fault->name());
                    /* Don't execute the fault, just stall the instruction
                     *  until it gets to the head of inFlightInsts */
                    inst->canEarlyIssue = false;
                    /* Not completed as we'll come here again to pick up
                     * the fault when we get to the end of the FU */
                    completed_inst = false;
                }
                else
                {
                    DPRINTF(MinorExecute, "Fault in execute: %s\n",
                            fault->name());
                    fault->invoke(thread, NULL);
                    tryToBranch(inst, fault, branch);
                    completed_inst = true;
                }
            }
            else
            {
                completed_inst = completed_mem_inst;
            }
            completed_mem_issue = completed_inst;
        }

        void Execute::actuallyExecuteInst(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, bool &committed)
        {
            ExecContext context(cpu, *cpu.threads[thread_id], inst);

            fault = inst->staticInst->execute(&context,
                                              inst->traceData);

            /* Set the predicate for tracing and dump */
            if (inst->traceData)
                inst->traceData->setPredicate(context.readPredicate());

            committed = true;

            if (fault != NoFault)
            {
                if (inst->traceData)
                {
                    if (debug::ExecFaulting)
                    {
                        inst->traceData->setFaulting(true);
                    }
                    else
                    {
                        delete inst->traceData;
                        inst->traceData = NULL;
                    }
                }

                DPRINTF(MinorExecute, "Fault in execute of inst: %s fault: %s\n",
                        *inst, fault->name());
                fault->invoke(thread, inst->staticInst);
            }
        }

        void Execute::checkSuspension(ThreadID thread_id, MinorDynInstPtr inst, gem5::ThreadContext *thread, BranchData &branch)
        {
            if (!inst->isFault() &&
                thread->status() == ThreadContext::Suspended &&
                branch.isBubble() &&       /* It didn't branch too */
                !isInterrupted(thread_id)) /* Don't suspend if we have
                    interrupts */
            {
                auto &resume_pc = cpu.getContext(thread_id)->pcState();

                assert(resume_pc.microPC() == 0);

                DPRINTF(MinorInterrupt, "Suspending thread: %d from Execute"
                                        " inst: %s\n",
                        thread_id, *inst);

                cpu.stats.numFetchSuspends++;

                updateBranchData(thread_id, BranchData::SuspendThread, inst,
                                 resume_pc, branch);
            }
        }

        bool
        Execute::commitInst(MinorDynInstPtr inst, ForwardInstData &insts_out, unsigned int *output_index, bool early_memory_issue,
                            BranchData &branch, Fault &fault, bool &issued_mem_ref)
        {
            ThreadID thread_id = inst->id.threadId;
            ThreadContext *thread = cpu.getContext(thread_id);

            bool completed_inst = true;
            fault = NoFault;
            DPRINTF(MinorExecute, "Committing inst: %s\n", *inst);
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

                DPRINTF(MinorExecute, "Fault inst reached Execute: %s\n",
                        inst->fault->name());

                fault = inst->fault;
                inst->fault->invoke(thread, NULL);
                inst->executed = true;
                tryToBranch(inst, fault, branch);
            }
            else if (inst->isInst() && inst->staticInst->isQuiesce() && !branch.isBubble())
            {
                /* This instruction can suspend, need to be able to communicate
                 * backwards, so no other branches may evaluate this cycle*/
                completed_inst = false;
            }
            else if (inst->staticInst->isMemRef())
            {
                FUPipeline *fu = funcUnits[inst->fuIndex];
                MinorFUTiming *timing = (!inst->isFault() ? fu->findTiming(inst->staticInst) : NULL);
                const std::vector<Cycles> *src_latencies =
                    (timing ? &(timing->srcRegsRelativeLats)
                            : NULL);
                const std::vector<bool> *cant_forward_from_fu_indices =
                    &(fu->cantForwardFromFUIndices);

                completed_inst = false;
                if (cpu.getScoreboard()[thread_id].canMemInstIssueAAA(inst, NULL, NULL, cpu.curCycle() + Cycles(1), thread, false))
                {
                    startMemRefExecution(inst, branch, fault, thread, false, completed_inst, issued_mem_ref);
                    DPRINTF(MinorExecute, "Sending mem inst to MEM: %s\n", *inst);

                    packIntoOutput(inst, insts_out, output_index);
                }
            }
            else if (inst->isInst() && inst->staticInst->isFullMemBarrier() &&
                     !cpu.getLSQ().canPushIntoStoreBuffer())
            {
                DPRINTF(MinorExecute, "Can't commit data barrier inst: %s yet as"
                                      " there isn't space in the store buffer\n",
                        *inst);
                completed_inst = false;
            }
            else
            {
                DPRINTF(MinorExecute, "Executing inst: %s\n", *inst);
                ExecContext context(cpu, *cpu.threads[thread_id], inst);
                if (!inst->staticInst->isMemRef())
                {
                    if (!inst->staticInst->isControl()) {
                        DPRINTF(MinorExecute, "Executing nomemref inst: %s\n", *inst);
                        fault = inst->staticInst->execute(&context, inst->traceData);

                        if (inst->traceData)
                            inst->traceData->setPredicate(context.readPredicate());

                        if (fault != NoFault)
                        {
                            if (inst->traceData)
                            {
                                if (debug::ExecFaulting)
                                {
                                    inst->traceData->setFaulting(true);
                                }
                                else
                                {
                                    delete inst->traceData;
                                    inst->traceData = NULL;
                                }
                            }

                            DPRINTF(MinorExecute, "Fault in execute of inst: %s fault: %s\n",
                                    *inst, fault->name());
                            fault->invoke(thread, inst->staticInst);
                        }
                    } else if (inst->staticInst->isControl()) {
                        DPRINTF(MinorExecute, "Not executing control inst because already executed: %s\n", *inst);
                        //context.writeback(inst->staticInst);
                        //inst->executed = true;
                        tryToBranch(inst, fault, branch);
                    }
                    inst->setRegsAfterExecution(context.getFwdRegFiles());
                }
                DPRINTF(MinorExecute, "Sending inst to MEM: %s\n", *inst);
                packIntoOutput(inst, insts_out, output_index);
            }

            if (completed_inst)
            {

                /* Keep a copy of this instruction's predictionSeqNum just in case
                 * we need to issue a branch without an instruction (such as an
                 * interrupt) */
                executeInfo[thread_id].lastPredictionSeqNum = inst->id.predictionSeqNum;

                /* Check to see if this instruction suspended the current thread. */
                checkSuspension(thread_id, inst, thread, branch);
            }

            return completed_inst;
        }

        void
        Execute::tryToHandleMemResponses(
            ExecuteThreadInfo &ex_info,
            bool discard_inst,
            bool &committed_inst,
            bool &completed_mem_ref,
            bool &completed_inst,
            MinorDynInstPtr inst,
            LSQ::LSQRequestPtr mem_response,
            BranchData &branch,
            Fault &fault)
        {
            DPRINTF(MinorExecute, "Trying to commit mem response: %s\n", *inst);

            /* Complete or discard the response */
            if (discard_inst)
            {
                DPRINTF(MinorExecute, "Discarding mem inst: %s as its"
                                      " stream state was unexpected, expected: %d\n",
                        *inst, ex_info.streamSeqNum);

                cpu.getLSQ().popResponse(mem_response);
            }
            else
            {
                handleMemResponse(inst, mem_response, branch, fault);
                committed_inst = true;
            }

            completed_mem_ref = true;
            completed_inst = true;
        }

        void
        Execute::checkIfCommitFromFUsPossible(const MinorDynInstPtr inst, bool &completed_inst, bool &try_to_commit, InstSeqNum head_exec_seq_num)
        {
            if (!completed_inst && !inst->inLSQ)
            {
                DPRINTF(MinorExecute, "Trying to commit from FUs\n");

                /* Try to commit from a functional unit */
                /* Is the head inst of the expected inst's FU actually the
                 *  expected inst? */
                QueuedInst &fu_inst =
                    funcUnits[inst->fuIndex]->front();
                InstSeqNum fu_inst_seq_num = fu_inst.inst->id.execSeqNum;

                if (fu_inst.inst->isBubble())
                {
                    /* No instruction ready */
                    completed_inst = false;
                }
                else if (fu_inst_seq_num != head_exec_seq_num)
                {
                    /* Past instruction: we must have already executed it
                     * in the same cycle and so the head inst isn't
                     * actually at the end of its pipeline
                     * Future instruction: handled above and only for
                     * mem refs on their way to the LSQ */
                }
                else if (fu_inst.inst->id == inst->id)
                {
                    /* All instructions can be committed if they have the
                     *  right execSeqNum and there are no in-flight
                     *  mem insts before us */
                    try_to_commit = true;
                    completed_inst = true;
                }
            }
        }

        void
        Execute::doCommitAccounting(MinorDynInstPtr const inst, ExecuteThreadInfo &ex_info, unsigned int &num_insts_committed, unsigned int &num_mem_refs_committed, bool completed_mem_ref)
        {
            bool is_no_cost_inst = inst->isNoCostInst();

            /* Don't show no cost instructions as having taken a commit
             *  slot */
            if (debug::MinorTrace && !is_no_cost_inst)
                ex_info.instsBeingCommitted.insts[num_insts_committed] = inst;

            if (!is_no_cost_inst)
                num_insts_committed++;

            if (num_insts_committed == commitLimit)
                DPRINTF(MinorExecute, "Reached inst commit limit\n");

            /* Re-set the time of the instruction if that's required for
             * tracing */
            if (inst->traceData)
            {
                if (setTraceTimeOnCommit)
                    inst->traceData->setWhen(curTick());
                inst->traceData->dump();
            }

            if (completed_mem_ref)
                num_mem_refs_committed++;

            if (num_mem_refs_committed == memoryCommitLimit)
                DPRINTF(MinorExecute, "Reached mem ref commit limit\n");
        }

        void
        Execute::finalizeCompletedInstruction(ThreadID thread_id, const MinorDynInstPtr inst, ExecuteThreadInfo &ex_info, const Fault &fault, bool issued_mem_ref, bool committed_inst)
        {
            /* Pop issued (to LSQ) and discarded mem refs from the inFUMemInsts
             *  as they've *definitely* exited the FUs */
            auto inmemqueue = ex_info.inFUMemInsts->getQueue();
            if (inst->isMemRef())
            {
                /* The MemRef could have been discarded from the FU or the memory
                 *  queue, so just check an FU instruction */
                auto el = std::find_if(inmemqueue.begin(), inmemqueue.end(), [inst](const QueuedInst &qi)
                                       { return qi.inst == inst; });
                if (!ex_info.inFUMemInsts->empty() && el != inmemqueue.end())
                {
                    ex_info.inFUMemInsts->getQueue().erase(el);
                }
            }

            if (!issued_mem_ref || fault != NoFault)
            {
                /* Note that this includes discarded insts */
                DPRINTF(MinorExecute, "Completed inst: %s\n", *inst);

                /* Got to the end of a full instruction? */
                ex_info.lastCommitWasEndOfMacroop = inst->isFault() ||
                                                    inst->isLastOpInInst();

                /* lastPredictionSeqNum is kept as a convenience to prevent its
                 *  value from changing too much on the minorview display */
                ex_info.lastPredictionSeqNum = inst->id.predictionSeqNum;

                /* Finished with the inst, remove it from the inst queue and
                 *  clear its dependencies */
                auto el = std::find_if(ex_info.inFlightInsts->getQueue().begin(),
                                       ex_info.inFlightInsts->getQueue().end(),
                                       [inst](const QueuedInst &qi)
                                       { return qi.inst == inst; });
                ex_info.inFlightInsts->getQueue().erase(el);

                // MOVETO: Memory
                /* Complete barriers in the LSQ/move to store buffer */
                if (inst->isInst() && inst->staticInst->isFullMemBarrier())
                {
                    DPRINTF(MinorMem, "Completing memory barrier"
                                      " inst: %s committed: %d\n",
                            *inst, committed_inst);
                    cpu.getLSQ().completeMemBarrierInst(inst, committed_inst);
                }
                if (!inst->isMemRef())
                    scoreboard[thread_id].clearInstDests(inst, inst->isMemRef());
            }
        }

        void
        Execute::sendCommittableOutput(ThreadID thread_id, ForwardInstData &insts_out, unsigned int *output_index,
                                       bool only_commit_microops, /* true if only microops should be committed,
                                                                     e.g. when an interrupt has occurred. This
                                                                     avoids having partially executed instructions */
                                       bool discard,              // discard all instructions
                                       BranchData &branch         // Eventual branches get written here
        )
        {
            Fault fault = NoFault;
            Cycles now = cpu.curCycle();
            ExecuteThreadInfo &ex_info = executeInfo[thread_id];
            // LSQ &lsq = cpu.getLSQ();
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
             *          Execute::streamSeqNum
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

            /* Number of memory access instructions committed to check against
             *  memCommitLimit */
            // unsigned int num_mem_refs_committed = 0;

            if (only_commit_microops && !ex_info.inFlightInsts->empty())
            {
                DPRINTF(MinorInterrupt, "Only commit microops %s %d\n",
                        *(ex_info.inFlightInsts->front().inst),
                        ex_info.lastCommitWasEndOfMacroop);
            }

            while (!ex_info.inFlightInsts->empty() && /* Some more instructions to process */
                   !branch.isStreamChange() &&        /* No real branch */
                   fault == NoFault &&                /* No faults */
                   completed_inst &&                  /* Still finding instructions to execute */
                   *output_index < outputWidth &&
                   num_insts_committed != commitLimit /* Not reached commit limit */
            )
            {
                if (only_commit_microops)
                {
                    DPRINTF(MinorInterrupt, "Committing tail of insts before"
                                            " interrupt: %s\n",
                            *(ex_info.inFlightInsts->front().inst));
                }

                QueuedInst *head_inflight_inst = &(ex_info.inFlightInsts->front());

                InstSeqNum head_exec_seq_num =
                    head_inflight_inst->inst->id.execSeqNum;

                /* The instruction we actually process if completed_inst
                 *  remains true to the end of the loop body.
                 *  Start by considering the the head of the in flight insts queue */
                MinorDynInstPtr inst = head_inflight_inst->inst;

                bool committed_inst = false;
                bool discard_inst = false;
                // bool completed_mem_ref = false;
                bool issued_mem_ref = false;
                // bool early_memory_issue = false;

                /* Must set this again to go around the loop */
                completed_inst = false;

                /* If we're just completing a macroop before an interrupt or drain,
                 *  can we stil commit another microop (rather than a memory response)
                 *  without crosing into the next full instruction? */
                bool in_flight_insts = !ex_info.inFlightInsts->empty();
                bool finished_macroop = only_commit_microops && ex_info.lastCommitWasEndOfMacroop;

                bool can_commit_insts = (in_flight_insts || inst->isMemRef()) && !finished_macroop;

                /* Can we find a mem response for this inst */
                // LSQ::LSQRequestPtr mem_response =
                // (inst->inLSQ ? lsq.findResponse(inst) : NULL);

                DPRINTF(MinorExecute, "Trying to commit canCommitInsts: %d\n",
                        can_commit_insts);

                /* Test for PC events after every instruction */
                if (isInbetweenInsts(thread_id) && tryPCEvents(thread_id))
                {
                    ThreadContext *thread = cpu.getContext(thread_id);

                    /* Branch as there was a change in PC */
                    updateBranchData(thread_id, BranchData::UnpredictedBranch,
                                     MinorDynInst::bubble(), thread->pcState(), branch);
                }
                // else if (mem_response &&
                //          num_mem_refs_committed < memoryCommitLimit)
                // {
                //     // MOVETO: MEMORY
                //     discard_inst = inst->id.streamSeqNum !=
                //                        ex_info.streamSeqNum ||
                //                    discard;
                //     tryToHandleMemResponses(ex_info, discard_inst, committed_inst, completed_mem_ref, completed_inst, inst, mem_response, branch, fault);
                // }
                else if (can_commit_insts)
                {
                    /* If true, this instruction will, subject to timing tweaks,
                     *  be considered for completion.  try_to_commit flattens
                     *  the `if' tree a bit and allows other tests for inst
                     *  commit to be inserted here. */
                    bool try_to_commit = false;
                    QueuedInst *committable_inst = NULL;
                    InstSeqNum committable_inst_seq_num = UINT64_MAX;
                    auto inst_queue = ex_info.inFlightInsts->getQueue();
                    for (auto it = inst_queue.begin(); it != inst_queue.end(); it++)
                    {
                        QueuedInst *fu_inst = &(*it);
                        if (fu_inst->inst->isNoCostInst())
                        {
                            if (fu_inst->inst->id.execSeqNum < committable_inst_seq_num)
                            {
                                committable_inst = fu_inst;
                                committable_inst_seq_num = fu_inst->inst->id.execSeqNum;
                                try_to_commit = true;
                            }
                            else
                            {
                                // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                //         curTick(),
                                //         1, /* not stalling */
                                //         inst->pc->instAddr(),
                                //         inst->staticInst->disassemble(inst->pc->instAddr()),
                                //         -1);
                            }
                        }
                        else
                        {
                            MinorDynInstPtr inst_ptr = funcUnits[fu_inst->inst->fuIndex]->front().inst;
                            if (inst_ptr->isBubble() || inst_ptr != fu_inst->inst)
                            {
                                // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                //         curTick(),
                                //         0, /* not stalling */
                                //         inst->pc->instAddr(),
                                //         inst->staticInst->disassemble(inst->pc->instAddr()),
                                //         fu_inst->inst->fuIndex);
                            }
                        }
                    }
                    for (unsigned int fu_index = 0; fu_index < funcUnits.size(); fu_index++)
                    {
                        QueuedInst *fu_inst = &(funcUnits[fu_index]->front());
                        if (!fu_inst->inst->isBubble() && fu_inst->inst->id.execSeqNum < committable_inst_seq_num)
                        {
                            committable_inst = fu_inst;
                            committable_inst_seq_num = fu_inst->inst->id.execSeqNum;
                            try_to_commit = true;
                        }
                    }
                    if (committable_inst)
                    {
                        inst = committable_inst->inst;
                    }
                    /* Try and commit FU-less insts */
                    if (!completed_inst && inst->isNoCostInst())
                    {
                        DPRINTF(MinorExecute, "Committing no cost inst: %s", *inst);

                        completed_inst = true;
                        try_to_commit = true;
                    }

                    if (try_to_commit)
                    {
                        discard_inst = inst->id.streamSeqNum !=
                                           ex_info.streamSeqNum ||
                                       discard;

                        /* Is this instruction discardable as its streamSeqNum
                         *  doesn't match? */
                        if (!discard_inst)
                        {
                            /* Try to commit or discard a non-memory instruction.
                             *  Memory ops are actually 'committed' from this FUs
                             *  and 'issued' into the memory system so we need to
                             *  account for them later (commit_was_mem_issue gets
                             *  set) */
                            if (inst->extraCommitDelayExpr)
                            {
                                DPRINTF(MinorExecute, "Evaluating expression for"
                                                      " extra commit delay inst: %s\n",
                                        *inst);

                                ThreadContext *thread = cpu.getContext(thread_id);

                                TimingExprEvalContext context(inst->staticInst,
                                                              thread, NULL);

                                uint64_t extra_delay = inst->extraCommitDelayExpr->evalFwd(context);

                                DPRINTF(MinorExecute, "Extra commit delay expr"
                                                      " result: %d\n",
                                        extra_delay);

                                if (extra_delay < 128)
                                {
                                    inst->extraCommitDelay += Cycles(extra_delay);
                                }
                                else
                                {
                                    DPRINTF(MinorExecute, "Extra commit delay was"
                                                          " very long: %d\n",
                                            extra_delay);
                                }
                                inst->extraCommitDelayExpr = NULL;
                            }

                            /* Move the extraCommitDelay from the instruction
                             *  into the minimumCommitCycle */
                            if (inst->extraCommitDelay != Cycles(0))
                            {
                                inst->minimumCommitCycle = cpu.curCycle() +
                                                           inst->extraCommitDelay;
                                inst->extraCommitDelay = Cycles(0);
                            }

                            else if (inst->minimumCommitCycle > now)
                            {
                                DPRINTF(MinorExecute, "Not committing inst: %s yet"
                                                      " as it wants to be stalled for %d more cycles\n",
                                        *inst, inst->minimumCommitCycle - now);
                                completed_inst = false;
                            }
                            else
                            {
                                completed_inst = commitInst(inst, insts_out, output_index,
                                                            false, branch, fault, issued_mem_ref);
                            }
                        }
                        else
                        {
                            /* Discard instruction */
                            completed_inst = true;
                            inst->committed = true;
                        }

                        if (completed_inst)
                        {
                            /* Allow the pipeline to advance.  If the FU head
                             *  instruction wasn't the inFlightInsts head
                             *  but had already been committed, it would have
                             *  unstalled the pipeline before here */
                            // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                            //         curTick(),
                            //         committable_inst ? committable_inst->fu_was_stalling : 0, /* not stalling */
                            //         inst->pc->instAddr(),
                            //         inst->staticInst->disassemble(inst->pc->instAddr()),
                            //         inst->fuIndex);
                            if (inst->fuIndex != noCostFUIndex)
                            {
                                DPRINTF(MinorExecute, "Unstalling %d for inst %s\n", inst->fuIndex, inst->id);
                                bool aaa = funcUnits[inst->fuIndex]->m_was_stalled;
                                funcUnits[inst->fuIndex]->stalled = false;
                                if (funcUnits[inst->fuIndex]->opLatency(inst->staticInst->opClass()) > 1)
                                {
                                    DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                            curTick(),
                                            funcUnits[inst->fuIndex]->m_was_stalled,
                                            inst->pc->instAddr(),
                                            inst->staticInst->disassemble(inst->pc->instAddr()),
                                            inst->fuIndex);
                                    // if (funcUnits[inst->fuIndex]->occupancy > 1)
                                    funcUnits[inst->fuIndex]->advance();
                                    funcUnits[inst->fuIndex]->m_was_stalled = aaa;
                                }
                            }
                        }
                        else
                        {
                            // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                            //         curTick(),
                            //         1, /* not stalling */
                            //         inst->pc->instAddr(),
                            //         inst->staticInst->disassemble(inst->pc->instAddr()),
                            //         inst->fuIndex);
                        }
                    }
                }
                else
                {
                    DPRINTF(MinorExecute, "No instructions to commit\n");
                    completed_inst = false;
                }

                /* All discardable instructions must also be 'completed' by now */
                assert(!(discard_inst && !completed_inst));

                /* Instruction committed but was discarded due to streamSeqNum
                 *  mismatch */
                if (discard_inst)
                {
                    DPRINTF(MinorExecute, "Discarding inst: %s as its stream"
                                          " state was unexpected, expected: %d\n",
                            *inst, ex_info.streamSeqNum);

                    if (fault == NoFault)
                        cpu.stats.numDiscardedOps++;
                }

                if (completed_inst && inst->isMemRef())
                {
                    /* The MemRef could have been discarded from the FU or the memory
                     *  queue, so just check an FU instruction */
                    if (!ex_info.inFUMemInsts->empty() &&
                        ex_info.inFUMemInsts->front().inst == inst)
                    {
                        ex_info.inFUMemInsts->pop();
                    }
                }

                /* Mark the mem inst as being in the LSQ */
                if (issued_mem_ref)
                {
                    inst->fuIndex = 0;
                    inst->inLSQ = true;
                }
                if (completed_inst /* && !(issued_mem_ref && fault == NoFault) */)
                {
                    finalizeCompletedInstruction(thread_id, inst, ex_info, fault, false, committed_inst);
                }

                /* Handle per-cycle instruction counting */
                if (committed_inst)
                {
                    unsigned int aaa = 0;
                    doCommitAccounting(inst, ex_info, num_insts_committed, aaa, false);
                }
            }
        }

        bool
        Execute::isInbetweenInsts(ThreadID thread_id) const
        {
            return executeInfo[thread_id].lastCommitWasEndOfMacroop &&
                   !cpu.getLSQ().accessesInFlight();
        }

        void
        Execute::evaluate()
        {
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

            BranchData &branch = *out.inputWire;
            BranchData &writeback_branch = *branch_out_wb.inputWire;
            BranchData &memory_branch = *branch_out_mem.inputWire;

            ForwardInstData &insts_out = *out_insts.inputWire;
            unsigned int output_index = 0;

            // LSQ &lsq = cpu.getLSQ();
            unsigned int num_issued = 0;

            /* Do all the cycle-wise activities for dcachePort here to potentially
             *  free up input spaces in the LSQ's requests queue */
            // lsq.step(); // MOVETO: MEMORY
            ThreadID issue_tid = getIssuingThread();
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
                executeInfo[tid].blocked = !nextStageReserve[tid].canReserve() || cpu.isMemoryExecuting(tid);

            /* Check interrupts first.  Will halt commit if interrupt found */
            bool interrupted = false;
            ThreadID interrupt_tid = checkInterrupts(branch, interrupted);
            ThreadID commit_tid = getCommittingThread();
            bool becoming_stalled = true;

            if (interrupt_tid != InvalidThreadID)
            {
                /* Signalling an interrupt this cycle, not issuing/committing from
                 * any other threads */
            }
            else if (!branch.isBubble())
            {
                /* It's important that this is here to carry Fetch1 wakeups to Fetch1
                 *  without overwriting them */
                DPRINTF(MinorInterrupt, "Execute skipping a cycle to allow old"
                                        " branch to complete\n");
            }
            else
            {
                /* This will issue merrily even when interrupted in the sure and
                 *  certain knowledge that the interrupt with change the stream */
                if (issue_tid != InvalidThreadID)
                {
                    DPRINTF(MinorExecute, "Attempting to issue [tid:%d]\n",
                            issue_tid);
                    unsigned int fu_idx = -1;
                    num_issued = issue(issue_tid, fu_idx);
                }
                else
                {
                    DPRINTF(MinorExecute, "No thread to issue\n");
                }
                
                commit_tid = getCommittingThread();
                attemptCommit(commit_tid, insts_out, &output_index, branch, interrupted);
            }

            /* Run logic to step functional units + decide if we are active on the next
             * clock cycle */
            std::vector<MinorDynInstPtr> next_issuable_insts;
            bool can_issue_next = checkForIssuableInsts(next_issuable_insts);

            /* Advance the pipelines and note whether they still need to be
             * advanced */
            advanceFunctionalUnits(next_issuable_insts, becoming_stalled, can_issue_next);

            bool head_inst_might_commit = headInstMightCommit();

            DPRINTF(Activity, "Need to tick num issued insts: %s%s%s%s%s\n",
                    (num_issued != 0 ? " (issued some insts)" : ""),
                    (becoming_stalled ? "(becoming stalled)" : "(not becoming stalled)"),
                    (can_issue_next ? " (can issued next inst)" : ""),
                    (head_inst_might_commit ? "(head inst might commit)" : ""),
                    (interrupted ? " (interrupted)" : ""));

            bool need_to_tick =
                num_issued != 0 ||        /* Issued some insts this cycle */
                !becoming_stalled ||      /* Some FU pipelines can still move */
                can_issue_next ||         /* Can still issue a new inst */
                head_inst_might_commit || /* Could possible commit the next inst */
                interrupted;              /* There are pending interrupts */

            if (!need_to_tick)
            {
                DPRINTF(Activity, "The next cycle might be skippable as there are no"
                                  " advanceable FUs\n");
            }

            /* Wake up if we need to tick again */
            cpu.wakeupOnEvent(Pipeline::ExecuteStageId);
            if (cpu.getLSQ().needsToTick())
                cpu.wakeupOnEvent(Pipeline::MemoryStageId);
            /* Note activity of following buffer */
            writeback_branch = branch;
            memory_branch = branch;
            if (!branch.isBubble() || !insts_out.isBubble())
            {
                cpu.activityRecorder->activity();
            }

            if (!insts_out.isBubble())
            {
                /* Note activity of following buffer */
                insts_out.threadId = commit_tid;
                nextStageReserve[commit_tid].reserve();
            }

            /* Make sure the input (if any left) is pushed */
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->threadId].pushTail();
        }

        ThreadID
        Execute::checkInterrupts(BranchData &branch, bool &interrupted)
        {
            ThreadID tid = interruptPriority;
            /* Evaluate interrupts in round-robin based upon service */
            do
            {
                /* Has an interrupt been signalled?  This may not be acted on
                 *  straighaway so this is different from took_interrupt */
                bool thread_interrupted = false;

                if (FullSystem && cpu.getInterruptController(tid))
                {
                    /* This is here because it seems that after drainResume the
                     * interrupt controller isn't always set */
                    thread_interrupted = executeInfo[tid].drainState == NotDraining &&
                                         isInterrupted(tid);
                    interrupted = interrupted || thread_interrupted;
                }
                else
                {
                    DPRINTF(MinorInterrupt, "No interrupt controller\n");
                }
                DPRINTF(MinorInterrupt, "[tid:%d] thread_interrupted?=%d isInbetweenInsts?=%d\n",
                        tid, thread_interrupted, isInbetweenInsts(tid));
                /* Act on interrupts */
                if (thread_interrupted && isInbetweenInsts(tid))
                {
                    if (takeInterrupt(tid, branch))
                    {
                        interruptPriority = tid;
                        return tid;
                    }
                }
                else
                {
                    tid = (tid + 1) % cpu.numThreads;
                }
            } while (tid != interruptPriority);

            return InvalidThreadID;
        }

        bool
        Execute::hasInterrupt(ThreadID thread_id)
        {
            if (FullSystem && cpu.getInterruptController(thread_id))
            {
                return executeInfo[thread_id].drainState == NotDraining &&
                       isInterrupted(thread_id);
            }

            return false;
        }

        void
        Execute::minorTrace() const
        {
            std::ostringstream insts;
            std::ostringstream stalled;

            executeInfo[0].instsBeingCommitted.reportData(insts);
            cpu.getLSQ().minorTrace();
            inputBuffer[0].minorTrace();
            scoreboard[0].minorTrace();

            /* Report functional unit stalling in one string */
            unsigned int i = 0;
            while (i < numFuncUnits)
            {
                stalled << (funcUnits[i]->stalled ? '1' : 'E');
                i++;
                if (i != numFuncUnits)
                    stalled << ',';
            }

            minor::minorTrace("insts=%s inputIndex=%d streamSeqNum=%d"
                              " stalled=%s drainState=%d isInbetweenInsts=%d\n",
                              insts.str(), executeInfo[0].inputIndex, executeInfo[0].streamSeqNum,
                              stalled.str(), executeInfo[0].drainState, isInbetweenInsts(0));

            std::for_each(funcUnits.begin(), funcUnits.end(),
                          std::mem_fn(&FUPipeline::minorTrace));

            executeInfo[0].inFlightInsts->minorTrace();
            executeInfo[0].inFUMemInsts->minorTrace();
        }

        inline ThreadID
        Execute::getCommittingThread()
        {
            std::vector<ThreadID> priority_list;
            // LSQ &lsq = cpu.getLSQ();
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
                ExecuteThreadInfo &ex_info = executeInfo[tid];
                bool can_commit_insts = !ex_info.inFlightInsts->empty() && !ex_info.blocked;
                if (can_commit_insts)
                {
                    QueuedInst *head_inflight_inst = &(ex_info.inFlightInsts->front());
                    MinorDynInstPtr inst = head_inflight_inst->inst;
                    auto inst_queue = ex_info.inFlightInsts->getQueue();
                    QueuedInst *committable_inst = NULL;
                    InstSeqNum committable_inst_seq_num = UINT64_MAX;
                    for (auto it = inst_queue.begin(); it != inst_queue.end(); it++)
                    {
                        QueuedInst *fu_inst = &(*it);
                        if (fu_inst->inst->isNoCostInst())
                        {
                            if (fu_inst->inst->id.execSeqNum < committable_inst_seq_num)
                            {
                                // committable_inst = fu_inst;
                                // committable_inst_seq_num = fu_inst->inst->id.execSeqNum;
                                // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                //         curTick(),
                                //         0, /* not stalling */
                                //         fu_inst->inst->pc->instAddr(),
                                //         fu_inst->inst->staticInst->disassemble(fu_inst->inst->pc->instAddr()),
                                //         -1);
                            }
                            else
                            {
                                // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                //         curTick(),
                                //         1, /* not stalling */
                                //         fu_inst->inst->pc->instAddr(),
                                //         fu_inst->inst->staticInst->disassemble(fu_inst->inst->pc->instAddr()),
                                //         -1);
                            }
                        }
                        else
                        {
                            MinorDynInstPtr inst_ptr = funcUnits[fu_inst->inst->fuIndex]->front().inst;
                            if (inst_ptr->isBubble() || inst_ptr != fu_inst->inst)
                            {
                                // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                //         curTick(),
                                //         0, /* not stalling */
                                //         fu_inst->inst->pc->instAddr(),
                                //         fu_inst->inst->staticInst->disassemble(fu_inst->inst->pc->instAddr()),
                                //         fu_inst->inst->fuIndex);
                            }
                        }
                    }
                    for (unsigned int fu_index = 0; fu_index < funcUnits.size(); fu_index++)
                    {
                        QueuedInst *fu_inst = &(funcUnits[fu_index]->front());
                        if (!fu_inst->inst->isBubble())
                        {
                            committable_inst = fu_inst;
                            committable_inst_seq_num = fu_inst->inst->id.execSeqNum;
                        }
                    }
                    if (committable_inst)
                    {
                        inst = committable_inst->inst;
                    }
                    bool can_transfer_mem_inst = false;

                    bool can_execute_fu_inst = inst->fuIndex == noCostFUIndex;
                    if (can_commit_insts && !can_transfer_mem_inst &&
                        inst->fuIndex != noCostFUIndex)
                    {
                        QueuedInst &fu_inst = funcUnits[inst->fuIndex]->front();
                        can_execute_fu_inst = !fu_inst.inst->isBubble() &&
                                              fu_inst.inst->id == inst->id;
                    }

                    can_commit_insts = can_commit_insts &&
                                       (can_transfer_mem_inst || can_execute_fu_inst);
                    if (can_commit_insts && executeInfo[tid].blocked)
                    {
                        // DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                        //         curTick(),
                        //         1, /* not stalling */
                        //         inst->pc->instAddr(),
                        //         inst->staticInst->disassemble(inst->pc->instAddr()),
                        //         inst->fuIndex);
                    }
                    // }
                }

                if (can_commit_insts && !executeInfo[tid].blocked)
                {
                    commitPriority = tid;
                    return tid;
                }
            }

            return InvalidThreadID;
        }

        inline ThreadID
        Execute::getIssuingThread()
        {
            std::vector<ThreadID> priority_list;

            switch (cpu.threadPolicy)
            {
            case enums::SingleThreaded:
                return 0;
            case enums::RoundRobin:
                priority_list = cpu.roundRobinPriority(issuePriority);
                break;
            case enums::Random:
                priority_list = cpu.randomPriority();
                break;
            default:
                panic("Invalid thread scheduling policy.");
            }

            for (auto tid : priority_list)
            {
                if (getInput(tid))
                {
                    issuePriority = tid;
                    return tid;
                }
            }

            return InvalidThreadID;
        }

        void
        Execute::drainResume()
        {
            DPRINTF(Drain, "MinorExecute drainResume\n");

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                setDrainState(tid, NotDraining);
            }

            cpu.wakeupOnEvent(Pipeline::ExecuteStageId);
        }

        std::ostream &operator<<(std::ostream &os, Execute::DrainState state)
        {
            switch (state)
            {
            case Execute::NotDraining:
                os << "NotDraining";
                break;
            case Execute::DrainCurrentInst:
                os << "DrainCurrentInst";
                break;
            case Execute::DrainHaltFetch:
                os << "DrainHaltFetch";
                break;
            case Execute::DrainAllInsts:
                os << "DrainAllInsts";
                break;
            default:
                os << "Drain-" << static_cast<int>(state);
                break;
            }

            return os;
        }

        void
        Execute::setDrainState(ThreadID thread_id, DrainState state)
        {
            DPRINTF(Drain, "setDrainState[%d]: %s\n", thread_id, state);
            executeInfo[thread_id].drainState = state;
        }

        unsigned int
        Execute::drain()
        {
            DPRINTF(Drain, "MinorExecute drain\n");

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                if (executeInfo[tid].drainState == NotDraining)
                {
                    cpu.wakeupOnEvent(Pipeline::ExecuteStageId);

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
        Execute::isDrained()
        {
            if (!cpu.getLSQ().isDrained())
                return false;

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                if (!inputBuffer[tid].empty() ||
                    !executeInfo[tid].inFlightInsts->empty())
                {

                    return false;
                }
            }

            return true;
        }

        Execute::~Execute()
        {
            for (unsigned int i = 0; i < numFuncUnits; i++)
                delete funcUnits[i];

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
                delete executeInfo[tid].inFlightInsts;
        }

        bool
        Execute::instIsRightStream(MinorDynInstPtr inst)
        {
            return inst->id.streamSeqNum == executeInfo[inst->id.threadId].streamSeqNum;
        }

        bool
        Execute::instIsHeadInst(MinorDynInstPtr inst)
        {
            bool ret = false;

            if (!executeInfo[inst->id.threadId].inFlightInsts->empty())
                ret = executeInfo[inst->id.threadId].inFlightInsts->front().inst->id == inst->id;

            return ret;
        }

        MinorCPU::MinorCPUPort &
        Execute::getDcachePort()
        {
            return cpu.getLSQ().getDcachePort();
        }

        bool
        Execute::checkForIssuableInsts(std::vector<MinorDynInstPtr> &next_issuable_insts)
        {
            bool can_issue_next = false;
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                if (getInput(tid))
                {
                    unsigned int input_index = executeInfo[tid].inputIndex;
                    MinorDynInstPtr inst = getInput(tid)->insts[input_index];
                    if (inst->isFault())
                    {
                        can_issue_next = true;
                    }
                    else if (!inst->isBubble())
                    {
                        next_issuable_insts.push_back(inst);
                    }
                }
            }
            return can_issue_next;
        }

        void
        Execute::advanceFunctionalUnits(std::vector<MinorDynInstPtr> &next_issuable_insts, bool &becoming_stalled, bool &can_issue_next)
        {
            for (unsigned int i = 0; i < numFuncUnits; i++)
            {
                FUPipeline *fu = funcUnits[i];
                bool was_stalled = fu->m_was_stalled;
                auto inst_at_fu_end = fu->front().inst->staticInst;
                bool has_issued_this_cycle = false;
                for (auto &inst : executeInfo[0].inFlightInsts->getQueue())
                {
                    if (inst.inst->fuIndex == i)
                    {
                        has_issued_this_cycle = has_issued_this_cycle || inst.just_issued;
                    }
                }
                bool has_fu_advanced = !fu->stalled;
                /* If we need to tick again, the pipeline will have been left or set
                 * to be unstalled */
                fu->advance();
                if (!has_issued_this_cycle || fu->occupancy <= 1)
                {
                    if (!fu->m_was_stalled && fu->stalled)
                    {
                        DPRINTF(MinorExecute, "Stalling fu %d in advanceFuncUnits\n", i);
                        DPRINTF(MinorExecute, "Occupancy: %d\n", fu->occupancy);
                        DPRINTF(MinorExecute, "Front inst: %s\n", fu->front().inst->id);
                    }
                    DPRINTF(MinorExecute, "Advancing FU[%d]\n", i);
                }
                if (fu->occupancy != 0 && !fu->stalled)
                {
                    becoming_stalled = false;
                }
                else if (!fu->front().inst->isBubble())
                {
                    // if (!fu->front().just_issued)
                    //     DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                    //             curTick(),
                    //             was_stalled, /* not stalling */
                    //             fu->front().inst->pc->instAddr(),
                    //             fu->front().inst->staticInst->disassemble(fu->front().inst->pc->instAddr()),
                    //             fu->front().inst->fuIndex);
                    // else
                    //     fu->front().just_issued = false;
                }
                for (auto &inst : executeInfo[0].inFlightInsts->getQueue())
                {
                    inst.fu_was_stalling = was_stalled;
                    if (inst.inst->fuIndex == i)
                    {
                        if (!inst.just_issued)
                            DPRINTF(MinorGUI, "Log4GUI: execute: %d: %d: %x: %s: %d\n",
                                    curTick(),
                                    was_stalled, /* not stalling */
                                    inst.inst->pc->instAddr(),
                                    inst.inst->staticInst->disassemble(inst.inst->pc->instAddr()),
                                    inst.inst->fuIndex);
                        else
                            inst.just_issued = false;
                    }
                }

                /* Could we possibly issue the next instruction from any thread?
                 * This is quite an expensive test and is only used to determine
                 * if the CPU should remain active, only run it if we aren't sure
                 * we are active next cycle yet */
                for (auto inst : next_issuable_insts)
                {
                    if (!fu->stalled && fu->provides(inst->staticInst->opClass()) &&
                        scoreboard[inst->id.threadId].canInstIssue(inst,
                                                                   NULL, NULL, cpu.curCycle() + Cycles(1),
                                                                   cpu.getContext(inst->id.threadId)))
                    {
                        can_issue_next = true;
                        break;
                    }
                }
            }
        }

        bool Execute::headInstMightCommit()
        {
            bool head_inst_might_commit = false;

            /* Could the head in flight insts be committed */
            for (auto const &info : executeInfo)
            {
                if (!info.inFlightInsts->empty())
                {
                    const QueuedInst &head_inst = info.inFlightInsts->front();

                    if (head_inst.inst->isNoCostInst())
                    {
                        head_inst_might_commit = true;
                    }
                    else
                    {
                        FUPipeline *fu = funcUnits[head_inst.inst->fuIndex];
                        if ((fu->stalled &&
                             fu->front().inst->id == head_inst.inst->id))
                        {
                            head_inst_might_commit = true;
                            break;
                        }
                    }
                }
            }

            return head_inst_might_commit;
        }

        void Execute::attemptCommit(ThreadID commit_tid, ForwardInstData &insts_out, unsigned int *output_index, BranchData &branch, bool interrupted)
        {
            if (commit_tid != InvalidThreadID)
            {
                ExecuteThreadInfo &commit_info = executeInfo[commit_tid];

                DPRINTF(MinorExecute, "Attempting to commit [tid:%d]\n",
                        commit_tid);
                /* commit can set stalled flags observable to issue and so *must* be
                 *  called first */
                if (commit_info.drainState != NotDraining)
                {
                    if (commit_info.drainState == DrainCurrentInst)
                    {
                        /* Commit only micro-ops, don't kill anything else */
                        sendCommittableOutput(commit_tid, insts_out, output_index, true, false, branch);

                        if (isInbetweenInsts(commit_tid))
                            setDrainState(commit_tid, DrainHaltFetch);

                        /* Discard any generated branch */
                        branch = BranchData::bubble();
                    }
                    else if (commit_info.drainState == DrainAllInsts)
                    {
                        /* Kill all instructions */
                        while (getInput(commit_tid))
                            popInput(commit_tid);
                        sendCommittableOutput(commit_tid, insts_out, output_index, false, true, branch);
                    }
                }
                else
                {
                    /* Commit micro-ops only if interrupted.  Otherwise, commit
                     *  anything you like */
                    DPRINTF(MinorExecute, "Committing micro-ops for interrupt[tid:%d]\n",
                            commit_tid);
                    bool only_commit_microops = interrupted &&
                                                hasInterrupt(commit_tid);
                    sendCommittableOutput(commit_tid, insts_out, output_index, only_commit_microops, false, branch);
                }

                /* Halt fetch, but don't do it until we have the current instruction in
                 *  the bag */
                if (commit_info.drainState == DrainHaltFetch)
                {
                    updateBranchData(commit_tid, BranchData::HaltFetch,
                                     MinorDynInst::bubble(),
                                     cpu.getContext(commit_tid)->pcState(), branch);

                    cpu.wakeupOnEvent(Pipeline::ExecuteStageId);
                    setDrainState(commit_tid, DrainAllInsts);
                }
            }
        }

    } // namespace minor
} // namespace gem5
