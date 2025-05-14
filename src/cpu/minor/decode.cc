/*
 * Copyright (c) 2013-2014 ARM Limited
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

#include "cpu/minor/decode.hh"

#include "arch/generic/decoder.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/minor/pipeline.hh"
#include "cpu/null_static_inst.hh"
#include "cpu/pred/bpred_unit.hh"
#include "debug/Branch.hh"
#include "debug/Decode.hh"
#include "debug/MinorTrace.hh"
#include "debug/MinorGUI.hh"
#include "cpu/minor/exec_context.hh"

namespace gem5
{

    GEM5_DEPRECATED_NAMESPACE(Minor, minor);
    namespace minor
    {

        Decode::Decode(const std::string &name,
                       MinorCPU &cpu_,
                       const BaseMinorCPUParams &params,
                       Latch<ForwardLineData>::Output inp_,
                       Latch<BranchData>::Output branchInp_,
                       Latch<BranchData>::Input predictionOut_,
                       Latch<ForwardInstData>::Input out_,
                       std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer,
                       std::vector<Scoreboard> &scoreboard_,
                       std::vector<FUPipeline *> &funcUnits_) : Named(name),
                                                                cpu(cpu_),
                                                                inp(inp_),
                                                                branchInp(branchInp_),
                                                                predictionOut(predictionOut_),
                                                                out(out_),
                                                                nextStageReserve(next_stage_input_buffer),
                                                                outputWidth(params.executeInputWidth),
                                                                processMoreThanOneInput(false), // params.decodeCycleInput),
                                                                branchPredictor(*params.branchPred),
                                                                macroInstPending(false), macroInstPendingPtr(NULL),
                                                                scoreboard(scoreboard_),
                                                                funcUnits(funcUnits_),
                                                                decodeInfo(params.numThreads),
                                                                threadPriority(0),
                                                                stats(&cpu_)
        {
            if (outputWidth < 1)
                fatal("%s: executeInputWidth must be >= 1 (%d)\n", name, outputWidth);

            if (params.decodeInputBufferSize < 1)
            {
                fatal("%s: decodeInputBufferSize must be >= 1 (%d)\n", name,
                      params.decodeInputBufferSize);
            }

            /* Per-thread input buffers */
            for (ThreadID tid = 0; tid < params.numThreads; tid++)
            {
                inputBuffer.push_back(
                    InputBuffer<ForwardLineData>(
                        name + ".inputBuffer" + std::to_string(tid), "insts",
                        params.decodeInputBufferSize));

                instWaitingDependencies.push_back(false);
                instWaitingDependenciesPtr.push_back(nullptr);
            }
        }

        const ForwardLineData *
        Decode::getInput(ThreadID tid)
        {
            /* Get insts from the inputBuffer to work with */
            if (!inputBuffer[tid].empty())
            {
                const ForwardLineData &head = inputBuffer[tid].front();

                return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
            }
            else
            {
                return NULL;
            }
        }

        void
        Decode::popInput(ThreadID tid)
        {
            if (!inputBuffer[tid].empty())
                inputBuffer[tid].pop();

            decodeInfo[tid].inputIndex = 0;
            decodeInfo[tid].inMacroop = false;
        }

        void
        Decode::dumpAllInput(ThreadID tid)
        {
            DPRINTF(Decode, "Dumping whole input buffer\n");
            while (!inputBuffer[tid].empty())
                popInput(tid);

            decodeInfo[tid].inputIndex = 0;
        }

        void
        Decode::updateBranchPrediction(const BranchData &branch)
        {
            MinorDynInstPtr inst = branch.inst;

            /* Don't even consider instructions we didn't try to predict or faults */
            if (inst->isFault() || !inst->triedToPredict)
                return;

            switch (branch.reason)
            {
            case BranchData::NoBranch:
                /* No data to update */
                break;
            case BranchData::Interrupt:
                /* Never try to predict interrupts */
                break;
            case BranchData::SuspendThread:
                /* Don't need to act on suspends */
                break;
            case BranchData::HaltFetch:
                /* Don't need to act on fetch wakeup */
                break;
            case BranchData::BranchPrediction:
                /* Shouldn't happen.  Fetch2 is the only source of
                 *  BranchPredictions */
                break;
            case BranchData::UnpredictedBranch:
                /* Unpredicted branch or barrier */
                DPRINTF(Branch, "Unpredicted branch seen inst: %s\n", *inst);
                branchPredictor.squash(inst->id.fetchSeqNum,
                                       *branch.target, true, inst->id.threadId);
                // Update after squashing to accomodate O3CPU
                // using the branch prediction code.
                branchPredictor.update(inst->id.fetchSeqNum,
                                       inst->id.threadId);
                break;
            case BranchData::CorrectlyPredictedBranch:
                /* Predicted taken, was taken */
                DPRINTF(Branch, "Branch predicted correctly inst: %s\n", *inst);
                branchPredictor.update(inst->id.fetchSeqNum,
                                       inst->id.threadId);
                break;
            case BranchData::BadlyPredictedBranch:
                /* Predicted taken, not taken */
                DPRINTF(Branch, "Branch mis-predicted inst: %s\n", *inst);
                branchPredictor.squash(inst->id.fetchSeqNum,
                                       *branch.target /* Not used */, false, inst->id.threadId);
                // Update after squashing to accomodate O3CPU
                // using the branch prediction code.
                branchPredictor.update(inst->id.fetchSeqNum,
                                       inst->id.threadId);
                break;
            case BranchData::BadlyPredictedBranchTarget:
                /* Predicted taken, was taken but to a different target */
                DPRINTF(Branch, "Branch mis-predicted target inst: %s target: %s\n",
                        *inst, *branch.target);
                branchPredictor.squash(inst->id.fetchSeqNum,
                                       *branch.target, true, inst->id.threadId);
                break;
            }
        }

        void
        Decode::predictBranch(MinorDynInstPtr inst, BranchData &branch, bool &early_branch)
        {
            DecodeThreadInfo &thread = decodeInfo[inst->id.threadId];

            assert(!inst->predictedTaken);

            /* Skip non-control/sys call instructions */
            if (inst->staticInst->isControl() || inst->staticInst->isSyscall())
            {
                std::unique_ptr<PCStateBase> inst_pc(inst->pc->clone());

                /* Tried to predict */
                inst->triedToPredict = true;

                DPRINTF(Branch, "Trying to predict for inst: %s\n", *inst);

                if (branchPredictor.predict(inst->staticInst,
                                            inst->id.fetchSeqNum, *inst_pc, inst->id.threadId))
                {
                    set(branch.target, *inst_pc);
                    inst->predictedTaken = true;
                    set(inst->predictedTarget, inst_pc);
                }

                if (inst->staticInst->isControl())
                {
                    
                }
            }

            else
            {
                DPRINTF(Branch, "Not attempting prediction for inst: %s\n", *inst);
            }

            if (branch.reason != BranchData::Reason::NoBranch)

            {
                thread.expectedStreamSeqNum = inst->id.streamSeqNum + 1;
                thread.predictionSeqNum++;
            }
            /* If we predict taken, set branch and update sequence numbers */
            else if (inst->predictedTaken)
            {
                /* Update the predictionSeqNum and remember the streamSeqNum that it
                 *  was associated with */
                thread.expectedStreamSeqNum = inst->id.streamSeqNum;

                BranchData new_branch = BranchData(BranchData::BranchPrediction,
                                                   inst->id.threadId,
                                                   inst->id.streamSeqNum, thread.predictionSeqNum + 1,
                                                   *inst->predictedTarget, inst);

                /* Mark with a new prediction number by the stream number of the
                 *  instruction causing the prediction */
                thread.predictionSeqNum++;
                branch = new_branch;

                DPRINTF(Branch, "Branch predicted taken inst: %s target: %s"
                                " new predictionSeqNum: %d\n",
                        *inst, *inst->predictedTarget, thread.predictionSeqNum);
            }
        }

#if TRACING_ON
        /** Add the tracing data to an instruction.  This originates in
         *  decode because this is the first place that execSeqNums are known
         *  (these are used as the 'FetchSeq' in tracing data) */
        static void
        dynInstAddTracing(MinorDynInstPtr inst, StaticInstPtr static_inst,
                          MinorCPU &cpu)
        {
            inst->traceData = cpu.getTracer()->getInstRecord(curTick(),
                                                             cpu.getContext(inst->id.threadId),
                                                             inst->staticInst, *inst->pc, static_inst);

            /* Use the execSeqNum as the fetch sequence number as this most closely
             *  matches the other processor models' idea of fetch sequence */
            if (inst->traceData)
                inst->traceData->setFetchSeq(inst->id.execSeqNum);
        }
#endif
        void Decode::earlyBranch(MinorDynInstPtr inst, Fault &f, BranchData &branch)
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

            DPRINTF(Branch, "earlyBranch before: %s after: %s%s\n",
                    *pc_before, *target, (force_branch ? " (forcing)" : ""));
            /* Will we change the PC to something other than the next instruction? */
            bool must_branch = *pc_before != *target ||
                               f != NoFault ||
                               force_branch;

            /* The reason for the branch data we're about to generate, set below */
            BranchData::Reason reason = BranchData::NoBranch;

            if (f == NoFault)
            {
                inst->staticInst->advancePC(*target);
                thread->pcState(*target);
                last_branch_pc = target->clone()->instAddr();

                DPRINTF(Decode, "Advancing current PC from: %s to: %s\n",
                        *pc_before, *target);
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
            DPRINTF(Decode, "Early branch: %s\n",
                    reason == BranchData::NoBranch ? "NoBranch" : "Branch");

            branch.reason = reason;
        }
        void
        Decode::pushIntoInpBuffer()
        {
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->id.threadId].setTail(*inp.outputWire);
        }

        void
        Decode::dumpIfBranchesExecuted(const BranchData &branch)
        {
            if (branch.isStreamChange())
            {
                DPRINTF(Decode, "Dumping all input as a stream changing branch"
                                " has arrived\n");
                dumpAllInput(delayBranchTid);
                decodeInfo[delayBranchTid].havePC = false;
            }
        }

        void
        Decode::popLinesIfPredictionMismatch(ThreadID tid)
        {
            const ForwardLineData *line_in = getInput(tid);

            while (line_in &&
                   decodeInfo[tid].expectedStreamSeqNum == line_in->id.streamSeqNum &&
                   decodeInfo[tid].predictionSeqNum != line_in->id.predictionSeqNum)
            {
                DPRINTF(Decode, "Discarding line %s"
                                " due to predictionSeqNum mismatch (expected: %d)\n",
                        line_in->id, decodeInfo[tid].predictionSeqNum);

                popInput(tid);
                decodeInfo[tid].havePC = false;

                if (true)
                {
                    DPRINTF(Decode, "Wrapping\n");
                    line_in = getInput(tid);
                }
                else
                {
                    line_in = NULL;
                }
            }
        }

        void
        Decode::updateAllThreadsStatus()
        {
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                decodeInfo[tid].blocked = !nextStageReserve[tid].canReserve();
                popLinesIfPredictionMismatch(tid);
            }
        }

        void
        Decode::givePcIfValidInstruction(
            ThreadID tid,
            bool discard_line,
            const ForwardLineData *line_in,
            InstDecoder *decoder)
        {
            /* Set the PC if the stream changes.  Setting havePC to false in
             *  a previous cycle handles all other change of flow of control
             *  issues */
            bool set_pc = decodeInfo[tid].lastStreamSeqNum != line_in->id.streamSeqNum;

            if (discard_line || (decodeInfo[tid].havePC && !set_pc))
            {
                return;
            }

            /* Set the inputIndex to be the MachInst-aligned offset
             *  from lineBaseAddr of the new PC value */
            decodeInfo[tid].inputIndex =
                (line_in->pc->instAddr() & decoder->pcMask()) -
                line_in->lineBaseAddr;
            DPRINTF(Decode, "Setting new PC value: %s inputIndex: 0x%x"
                            " lineBaseAddr: 0x%x lineWidth: 0x%x\n",
                    *line_in->pc, decodeInfo[tid].inputIndex, line_in->lineBaseAddr,
                    line_in->lineWidth);
            set(decodeInfo[tid].pc, *line_in->pc);
            decodeInfo[tid].havePC = true;
            decoder->reset();
        }

        MinorDynInstPtr
        Decode::packFault(const ForwardLineData *line_in, ThreadID tid)
        {
            MinorDynInstPtr dyn_inst = new MinorDynInst(nullStaticInstPtr,
                                                        line_in->id);

            /* Fetch and prediction sequence numbers originate here */
            dyn_inst->id.fetchSeqNum = decodeInfo[tid].fetchSeqNum;
            dyn_inst->id.predictionSeqNum = decodeInfo[tid].predictionSeqNum;

            /* To complete the set, test that exec sequence number has
             *  not been set */
            assert(dyn_inst->id.execSeqNum == 0);

            set(dyn_inst->pc, decodeInfo[tid].pc);

            /* Pack a faulting instruction but allow other
             *  instructions to be generated. (Fetch2 makes no
             *  immediate judgement about streamSeqNum) */
            dyn_inst->fault = line_in->fault;

            return dyn_inst;
        }

        MinorDynInstPtr
        Decode::packInst(StaticInstPtr decoded_inst, InstId id, ThreadID tid)
        {
            MinorDynInstPtr dyn_inst = new MinorDynInst(decoded_inst, id);

            /* Fetch and prediction sequence numbers originate here */
            dyn_inst->id.fetchSeqNum = decodeInfo[tid].fetchSeqNum;
            dyn_inst->id.predictionSeqNum = decodeInfo[tid].predictionSeqNum;
            /* To complete the set, test that exec sequence number
             *  has not been set */
            assert(dyn_inst->id.execSeqNum == 0);

            set(dyn_inst->pc, decodeInfo[tid].pc);

            return dyn_inst;
        }

        void
        Decode::collectStats(StaticInstPtr decoded_inst)
        {
            if (decoded_inst->isLoad())
                stats.loadInstructions++;
            else if (decoded_inst->isStore())
                stats.storeInstructions++;
            else if (decoded_inst->isAtomic())
                stats.amoInstructions++;
            else if (decoded_inst->isVector())
                stats.vecInstructions++;
            else if (decoded_inst->isFloating())
                stats.fpInstructions++;
            else if (decoded_inst->isInteger())
                stats.intInstructions++;
        }

        void
        Decode::advanceInput(ThreadID tid)
        {
            decodeInfo[tid].inputIndex++;
            decodeInfo[tid].inMacroop = false;
        }

        void
        Decode::setUpPcForMicroop(ThreadID tid, MinorDynInstPtr inst)
        {
            if (!decodeInfo[tid].inMacroop)
            {
                set(decodeInfo[tid].microopPC, *inst->pc);
                decodeInfo[tid].inMacroop = true;
            }
        }

        StaticInstPtr
        Decode::extractMicroInst(ThreadID tid, StaticInstPtr static_inst)
        {
            return static_inst->fetchMicroop(
                decodeInfo[tid].microopPC->microPC());
        }

        /*
        MinorDynInstPtr
        Decode::packInst(StaticInstPtr static_micro_inst, InstId id, ThreadID tid)
        {
            MinorDynInstPtr output_inst = NULL;

            output_inst = new MinorDynInst(static_micro_inst, id);
            set(output_inst->pc, decodeInfo[tid].microopPC);
            output_inst->fault = NoFault;

            return output_inst;
        }*/

        void
        Decode::allowPredictionOnLastMicroop(
            StaticInstPtr static_micro_inst,
            MinorDynInstPtr output_inst,
            MinorDynInstPtr macro_inst)
        {
            if (!static_micro_inst->isLastMicroop())
                return;

            output_inst->predictedTaken = macro_inst->predictedTaken;
            set(output_inst->predictedTarget,
                macro_inst->predictedTarget);
        }

        MinorDynInstPtr
        Decode::decomposition(
            ThreadID tid,
            MinorDynInstPtr inst,
            unsigned int output_index)
        {
            /* Generate a new micro-op */
            StaticInstPtr static_micro_inst;
            StaticInstPtr static_inst = inst->staticInst;

            /* Set up PC for the next micro-op emitted */
            setUpPcForMicroop(tid, inst);

            /* Get the micro-op static instruction from the
             * static_inst. */
            static_micro_inst = extractMicroInst(tid, static_inst);

            MinorDynInstPtr output_inst = packInst(static_micro_inst, inst->id, tid);

            /* Allow a predicted next address only on the last
             *  microop */
            allowPredictionOnLastMicroop(static_micro_inst, output_inst, inst);

            DPRINTF(Decode, "Microop decomposition inputIndex:"
                            " %d output_index: %d lastMicroop: %s microopPC:"
                            " %s inst: %d\n",
                    decodeInfo[tid].inputIndex, output_index,
                    (static_micro_inst->isLastMicroop() ? "true" : "false"),
                    *decodeInfo[tid].microopPC,
                    *output_inst);

            static_micro_inst->advancePC(*decodeInfo[tid].microopPC);

            /* Always update macroInstPending flag */
            macroInstPending = !static_micro_inst->isLastMicroop();

            /* Step input if this is the last micro-op */
            /*if (static_micro_inst->isLastMicroop()) {
                advanceInput(tid);
            }*/
            return output_inst;
        }

        void
        Decode::assignExecSeqNum(ThreadID tid, MinorDynInstPtr output_inst)
        {
            output_inst->id.execSeqNum = decodeInfo[tid].execSeqNum;
            /* Step to next sequence number */
            decodeInfo[tid].execSeqNum++;
        }

        void
        Decode::packIntoOutput(
            MinorDynInstPtr output_inst,
            ForwardInstData &insts_out,
            unsigned int *output_index)
        {
            /* Correctly size the output before writing */
            if (*output_index == 0)
            {
                insts_out.resize(outputWidth);
            }

            /* Push into output */
            insts_out.insts[*output_index] = output_inst;

            (*output_index) += 1;
        }

        void
        Decode::macroopTraceInst(MinorDynInstPtr dyn_inst)
        {
            if (debug::MinorTrace && !dyn_inst->isFault() &&
                dyn_inst->staticInst->isMacroop())
            {
                dyn_inst->minorTraceInst(*this);
            }
        }

        void
        Decode::finishLineProcessing(
            ThreadID tid,
            const ForwardLineData **line_in,
            bool prediction,
            bool discard_line)
        {
            /* Asked to discard line or there was a branch or fault */
            if (prediction || /* The remains of a
                line with a prediction in it */
                (*line_in)->isFault() /* A line which is just a fault */)
            {
                DPRINTF(Decode, "Discarding all input on branch/fault\n");
                dumpAllInput(tid);
                decodeInfo[tid].havePC = false;
                (*line_in) = NULL;
            }
            else if (discard_line)
            {
                /* Just discard one line, one's behind it may have new
                 *  stream sequence numbers.  There's a DPRINTF above
                 *  for this event */
                popInput(tid);
                decodeInfo[tid].havePC = false;
                (*line_in) = NULL;
            }
            else if (decodeInfo[tid].inputIndex == (*line_in)->lineWidth)
            {
                /* Got to end of a line, pop the line but keep PC
                 *  in case this is a line-wrapping inst. */
                popInput(tid);
                (*line_in) = NULL;
            }
        }

        const ForwardLineData *
        Decode::maybeMoreInput(ThreadID tid, const ForwardLineData *line_in)
        {
            if (!line_in && processMoreThanOneInput)
            {
                DPRINTF(Decode, "Wrapping\n");
                return getInput(tid);
            }

            return NULL;
        }

        void
        Decode::reserveSpaceInNextStage(ForwardInstData &insts_out, ThreadID tid)
        {
            if (!insts_out.isBubble())
            {
                /* Note activity of following buffer */
                cpu.activityRecorder->activity();
                insts_out.threadId = tid;
                nextStageReserve[tid].reserve();
            }
        }

        void
        Decode::markStageActivity()
        {
            for (ThreadID i = 0; i < cpu.numThreads; i++)
            {
                if ((getInput(i) && nextStageReserve[i].canReserve()) || instWaitingDependencies[i])
                {
                    cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
                    break;
                }
            }
            cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
        }

        void
        Decode::pushTailInpBuffer()
        {
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->id.threadId].pushTail();
        }

        int
        Decode::findFunctionUnit(MinorDynInstPtr output_inst, std::vector<FUPipeline *> &funcUnits)
        {
            const unsigned int numFuncUnits = funcUnits.size();
            for (unsigned int fu_index = 0; fu_index < numFuncUnits; fu_index++)
            {

                bool fu_is_capable = (!output_inst->isFault() ? funcUnits[fu_index]->provides(output_inst->staticInst->opClass()) : true);
                if (fu_is_capable &&
                    !funcUnits[fu_index]->m_was_stalled &&
                    funcUnits[fu_index]->canInsertNextCycle() &&
                    !funcUnits[fu_index]->alreadyPushed())
                {
                    return fu_index;
                }
            }
            return -1;
        }

        bool
        Decode::checkScoreboardAndUpdate(MinorDynInstPtr output_inst, ThreadID tid)
        {
            if (output_inst->isNoCostInst())
            {
                cpu.getScoreboard()[tid].markupInstDests(output_inst, cpu.curCycle() + Cycles(0) + Cycles(1), cpu.getContext(tid), false);

                return true;
            }

            /* Find FU that can execute this instruction */
            int fu_index = findFunctionUnit(output_inst, funcUnits);
            if (fu_index == -1)
                return false;

            FUPipeline *fu = funcUnits[fu_index];
            MinorFUTiming *timing = (!output_inst->isFault() ? fu->findTiming(output_inst->staticInst) : NULL);
            const std::vector<Cycles> *src_latencies =
                (timing ? &(timing->srcRegsRelativeLats)
                        : NULL);
            const std::vector<bool> *cant_forward_from_fu_indices =
                &(fu->cantForwardFromFUIndices);

            if (output_inst->isMemRef())
            {
                /* If it's a memory instruction, we need to check the
                 *  memory ordering model */
                if (!cpu.getScoreboard()[tid].canMemInstIssue(output_inst,
                                                              src_latencies, cant_forward_from_fu_indices,
                                                              cpu.curCycle() + Cycles(1), cpu.getContext(tid), true))
                {
                    return false;
                }
            }
            Cycles delay = Cycles(1);
            if (output_inst->isInst() && output_inst->staticInst->isControl())
            {
                delay = Cycles(0);
            }

            bool can_issue = output_inst->isMemRef() ? cpu.getScoreboard()[tid].canMemInstIssue(output_inst,
                                                                                                src_latencies, cant_forward_from_fu_indices,
                                                                                                cpu.curCycle() + delay, cpu.getContext(tid), false)
                                                     : cpu.getScoreboard()[tid].canInstIssue(output_inst,
                                                                                             src_latencies, cant_forward_from_fu_indices,
                                                                                             cpu.curCycle() + delay, cpu.getContext(tid));
            if (!can_issue)
            {
                return false;
            }

            bool issued_mem_ref = output_inst->isMemRef();
            Cycles extra_dest_retire_lat = Cycles(0);
            Cycles extra_assumed_lat = Cycles(0);

            if (timing)
            {
                extra_dest_retire_lat =
                    timing->extraCommitLat;
                extra_assumed_lat =
                    timing->extraAssumedLat;
            }

            cpu.getScoreboard()[tid].markupInstDests(output_inst, cpu.curCycle() + Cycles(1) + fu->description.opLat + extra_dest_retire_lat + extra_assumed_lat,
                                                     cpu.getContext(tid),
                                                     issued_mem_ref && extra_assumed_lat == Cycles(0));
            return true;
        }

        void
        Decode::evaluate()
        {
            /* Push input onto appropriate input buffer */
            if (!macroInstPending)
            {
                pushIntoInpBuffer();
            }

            ForwardInstData &insts_out = *out.inputWire;
            BranchData prediction;
            BranchData &branch_inp = *branchInp.outputWire;

            MinorDynInstPtr inst_ptr_4_GUI = NULL;

            assert(insts_out.isBubble());
            /* React to branches from Execute to update local branch prediction
             *  structures */
            updateBranchPrediction(branch_inp);

            /* If a branch arrives, don't try and do anything about it.  Only
             *  react to your own predictions */
            dumpIfBranchesExecuted(branch_inp);
            
            assert(insts_out.isBubble());

            /* Even when blocked, clear out input lines with the wrong
             *  prediction sequence number */
            updateAllThreadsStatus();

            ThreadID tid = getScheduledThread();
            DPRINTF(Decode, "Scheduled Thread: %d\n", tid);

            assert(insts_out.isBubble());

            if (tid == InvalidThreadID)
            {
                assert(insts_out.isBubble());
            }
            else
            {
                DecodeThreadInfo &decode_info = decodeInfo[tid];
                const ForwardLineData *line_in = getInput(tid);
                if (!line_in && has_branched) {
                    last_inst_pc = last_branch_pc;
                    waiting_fetch = true;
                } else {
                    waiting_fetch = false;
                }

                unsigned int output_index = 0;

                /* Pack instructions into the output while we can.  This may involve
                 * using more than one input line.  Note that lineWidth will be 0
                 * for faulting lines */
                bool early_branch = false;
                while ((((line_in &&
                          (line_in->isFault() ||
                           decode_info.inputIndex < line_in->lineWidth)) || /* More input */
                         macroInstPending) &&                               /* Some macroinst not completely processed */
                        output_index < outputWidth &&                       /* More output to fill */
                        prediction.isBubble() &&
                        !early_branch) ||
                       instWaitingDependencies[tid]) /* No predicted branch */
                {
                    has_branched = false;
                    /* The generated instruction.  Leave as NULL if no instruction
                     *  is to be packed into the output */
                    MinorDynInstPtr dyn_inst = NULL;

                    if (!macroInstPending && !instWaitingDependencies[tid])
                    {
                        ThreadContext *thread = cpu.getContext(line_in->id.threadId);
                        InstDecoder *decoder = thread->getDecoderPtr();

                        /* Discard line due to prediction sequence number being wrong but
                         * without the streamSeqNum number having changed */
                        bool discard_line =
                            decode_info.expectedStreamSeqNum == line_in->id.streamSeqNum &&
                            decode_info.predictionSeqNum != line_in->id.predictionSeqNum;

                        givePcIfValidInstruction(tid, discard_line, line_in, decoder);

                        if (discard_line)
                        {
                            /* Rest of line was from an older prediction in the same
                             *  stream */
                            DPRINTF(Decode, "Discarding line %s (from inputIndex: %d)"
                                            " due to predictionSeqNum mismatch (expected: %d)\n",
                                    line_in->id, decode_info.inputIndex,
                                    decode_info.predictionSeqNum);
                        }
                        else if (line_in->isFault())
                        {
                            /* Pack a fault as a MinorDynInst with ->fault set
                             * Make a new instruction and pick up the line, stream,
                             * prediction, thread ids from the incoming line */
                            dyn_inst = packFault(line_in, tid);
                            DPRINTF(Decode, "Fault being passed output_index: "
                                            "%d: %s\n",
                                    output_index, dyn_inst->fault->name());
                        }
                        else
                        {
                            uint8_t *line = line_in->line;

                            /* The instruction is wholly in the line,
                             *  can just copy. */
                            memcpy(decoder->moreBytesPtr(),
                                   line + decode_info.inputIndex,
                                   decoder->moreBytesSize());

                            if (!decoder->instReady())
                            {
                                decoder->moreBytes(*decode_info.pc,
                                                   line_in->lineBaseAddr + decode_info.inputIndex);
                                DPRINTF(Decode,
                                        "Offering MachInst to decoder addr: 0x%x\n",
                                        line_in->lineBaseAddr + decode_info.inputIndex);
                            }

                            /* Maybe make the above a loop to accomodate ISAs with
                             *  instructions longer than sizeof(MachInst) */

                            if (decoder->instReady())
                            {
                                /* Note that the decoder can update the given PC.
                                 *  Remember not to assign it until *after* calling
                                 *  decode */
                                StaticInstPtr decoded_inst =
                                    decoder->decode(*decode_info.pc);

                                /* Make a new instruction and pick up the line, stream,
                                 *  prediction, thread ids from the incoming line */
                                dyn_inst = new MinorDynInst(decoded_inst, line_in->id);

                                /* Fetch and prediction sequence numbers originate
                                 *  here */
                                dyn_inst->id.fetchSeqNum = decode_info.fetchSeqNum;
                                dyn_inst->id.predictionSeqNum =
                                    decode_info.predictionSeqNum;
                                /* To complete the set, test that exec sequence number
                                 *  has not been set */
                                assert(dyn_inst->id.execSeqNum == 0);

                                set(dyn_inst->pc, decode_info.pc);
                                DPRINTF(Decode, "decoder inst %s\n", *dyn_inst);

                                // Collect some basic inst class stats
                                if (decoded_inst->isLoad())
                                    stats.loadInstructions++;
                                else if (decoded_inst->isStore())
                                    stats.storeInstructions++;
                                else if (decoded_inst->isAtomic())
                                    stats.amoInstructions++;
                                else if (decoded_inst->isVector())
                                    stats.vecInstructions++;
                                else if (decoded_inst->isFloating())
                                    stats.fpInstructions++;
                                else if (decoded_inst->isInteger())
                                    stats.intInstructions++;

                                DPRINTF(Decode, "Instruction extracted from line %s"
                                                " lineWidth: %d output_index: %d inputIndex: %d"
                                                " pc: %s inst: %s\n",
                                        line_in->id,
                                        line_in->lineWidth, output_index,
                                        decode_info.inputIndex,
                                        *decode_info.pc, *dyn_inst);
                                auto next_pc = std::unique_ptr<PCStateBase>(dyn_inst->pc->clone());
                                next_pc->advance();
                                last_inst_pc = next_pc->instAddr();
                                /*
                                 * In SE mode, it's possible to branch to a microop when
                                 * replaying faults such as page faults (or simply
                                 * intra-microcode branches in X86).  Unfortunately,
                                 * as Minor has micro-op decomposition in a separate
                                 * pipeline stage from instruction decomposition, the
                                 * following advancePC (which may follow a branch with
                                 * microPC() != 0) *must* see a fresh macroop.
                                 *
                                 * X86 can branch within microops so we need to deal with
                                 * the case that, after a branch, the first un-advanced PC
                                 * may be pointing to a microop other than 0.  Once
                                 * advanced, however, the microop number *must* be 0
                                 */
                                decode_info.pc->uReset();

                                /* Advance PC for the next instruction */
                                decoded_inst->advancePC(*decode_info.pc);

                                /* Predict any branches and issue a branch if
                                 *  necessary */
                                predictBranch(dyn_inst, prediction, early_branch);
                            }
                            else
                            {
                                DPRINTF(Decode, "Inst not ready yet\n");
                            }

                            /* Step on the pointer into the line if there's no
                             *  complete instruction waiting */
                            if (decoder->needMoreBytes())
                            {
                                decode_info.inputIndex += decoder->moreBytesSize();

                                DPRINTF(Decode, "Updated inputIndex value PC: %s"
                                                " inputIndex: 0x%x lineBaseAddr: 0x%x lineWidth:"
                                                " 0x%x\n",
                                        *line_in->pc, decode_info.inputIndex,
                                        line_in->lineBaseAddr,
                                        line_in->lineWidth);
                            }
                        }

                        /* Remember the streamSeqNum of this line so we can tell when
                         *  we change stream */
                        decode_info.lastStreamSeqNum = line_in->id.streamSeqNum;

                        finishLineProcessing(tid, &line_in,
                                             !prediction.isBubble(), discard_line);

                        line_in = maybeMoreInput(tid, line_in);

                        /* Info for GUI */
                        inst_ptr_4_GUI = dyn_inst;
                    }

                    if (instWaitingDependencies[tid])
                    {

                        if (checkScoreboardAndUpdate(instWaitingDependenciesPtr[tid], tid))
                        {
                            DPRINTF(Decode, "Can pass to Execute: %s\n",
                                    *instWaitingDependenciesPtr[tid]);
                            inst_ptr_4_GUI = instWaitingDependenciesPtr[tid];

                            DPRINTF(MinorGUI, "Log4GUI: decode: %d: %d: %x: %s\n",
                                    curTick(),
                                    decode_is_stalling,
                                    inst_ptr_4_GUI->pc->instAddr(),
                                    inst_ptr_4_GUI->staticInst->disassemble(inst_ptr_4_GUI->pc->instAddr()));
                            if (inst_ptr_4_GUI->isInst() && inst_ptr_4_GUI->staticInst->isControl())
                            {
                                DPRINTF(Decode, "Early executing control inst: %s\n", *inst_ptr_4_GUI);
                                ExecContext context(cpu, *cpu.threads[inst_ptr_4_GUI->id.threadId], inst_ptr_4_GUI);

                                Fault fault = inst_ptr_4_GUI->staticInst->execute(&context, inst_ptr_4_GUI->traceData);

                                if (inst_ptr_4_GUI->traceData)
                                    inst_ptr_4_GUI->traceData->setPredicate(context.readPredicate());

                                if (fault != NoFault)
                                {
                                    DPRINTF(Decode, "Fault in execute of inst: %s fault: %s\n",
                                            *inst_ptr_4_GUI, fault->name());
                                    panic("Fault in early branch execution of inst: %s fault: %s\n",
                                          *inst_ptr_4_GUI, fault->name());
                                }
                                if (prediction.reason != BranchData::Reason::NoBranch)

                                {
                                    decode_info.expectedStreamSeqNum = inst_ptr_4_GUI->id.streamSeqNum + 1;
                                    decode_info.predictionSeqNum++;
                                }
                                DPRINTF(Decode, "Executing control inst: %s\n", *inst_ptr_4_GUI);
                                context.writeback(inst_ptr_4_GUI->staticInst);
                                inst_ptr_4_GUI->executed = true;
                                earlyBranch(inst_ptr_4_GUI, fault, prediction);
                                dumpIfBranchesExecuted(prediction);
                                decode_info.expectedStreamSeqNum = inst_ptr_4_GUI->id.streamSeqNum;
                                ThreadContext *thread = cpu.getContext(inst_ptr_4_GUI->id.threadId);
                                prediction = BranchData(prediction.reason,
                                                                   inst_ptr_4_GUI->id.threadId,
                                                                   inst_ptr_4_GUI->id.streamSeqNum, prediction.isStreamChange() ? decode_info.predictionSeqNum + 1 : decode_info.predictionSeqNum,
                                                                   *thread->pcState().clone(), inst_ptr_4_GUI);

                                /* Mark with a new prediction number by the stream number of the
                                 *  instruction causing the prediction */
                                if (prediction.isStreamChange())
                                {
                                    decode_info.predictionSeqNum++;
                                }
                            }
                            packIntoOutput(instWaitingDependenciesPtr[tid],
                                           insts_out, &output_index);
                            auto next_pc = std::unique_ptr<PCStateBase>(instWaitingDependenciesPtr[tid]->pc->clone());
                            next_pc->advance();
                            last_inst_pc = next_pc->instAddr();
                            instWaitingDependencies[tid] = false;
                            instWaitingDependenciesPtr[tid] = nullptr;
                            /* Continue the execution */
                            was_stalling = decode_is_stalling;
                            decode_is_stalling = false;
                            if (prediction.isStreamChange())
                            {
                                //last_inst_pc = prediction.target->instAddr();
                                has_branched = true;
                            }
                        }
                        else
                        {
                            DPRINTF(Decode, "Cannot pass to Execute: %s\n",
                                    *instWaitingDependenciesPtr[tid]);
                            /* Stall here */
                            inst_ptr_4_GUI = instWaitingDependenciesPtr[tid];
                            DPRINTF(MinorGUI, "Log4GUI: decode: %d: %d: %x: %s\n",
                                    curTick(),
                                    decode_is_stalling,
                                    inst_ptr_4_GUI->pc->instAddr(),
                                    inst_ptr_4_GUI->staticInst->disassemble(inst_ptr_4_GUI->pc->instAddr()));
                            auto next_pc = std::unique_ptr<PCStateBase>(inst_ptr_4_GUI->pc->clone());
                            next_pc->advance();
                            last_inst_pc = next_pc->instAddr();
                            was_stalling = decode_is_stalling;
                            decode_is_stalling = true;
                            has_branched = false;
                        }
                        break;
                    }
                    else if (dyn_inst || macroInstPending)
                    {
                        MinorDynInstPtr curr_inst = NULL;
                        MinorDynInstPtr output_inst = NULL;

                        if (macroInstPending)
                        {
                            curr_inst = macroInstPendingPtr;
                        }
                        else
                        {
                            curr_inst = dyn_inst;

                            /* Step to next sequence number */
                            decode_info.fetchSeqNum++;
                        }

                        /* Output MinorTrace instruction info for
                         *  pre-microop decomposition macroops */
                        macroopTraceInst(curr_inst);

                        StaticInstPtr static_inst = curr_inst->staticInst;

#if TRACING_ON
                        if (curr_inst->isFault())
                        {
                            dynInstAddTracing(curr_inst, NULL, cpu);
                        }
                        else
                        {
                            dynInstAddTracing(curr_inst, static_inst, cpu);
                        }
#endif

                        if (static_inst->isMacroop())
                        {
                            output_inst = decomposition(tid, curr_inst, output_index);
                            macroInstPendingPtr = curr_inst;
                        }
                        else
                        {
                            output_inst = dyn_inst;
                        }

                        assignExecSeqNum(tid, output_inst);

                        if (checkScoreboardAndUpdate(output_inst, tid))
                        {
                            DPRINTF(Decode, "Can pass to Execute: %s\n", *output_inst);
                            // if (output_inst->isInst() && output_inst->staticInst->isControl())
                            // {
                            //     earlyExecuteBranch(output_inst, prediction);
                            // }
                            if (output_inst->isInst() && output_inst->staticInst->isControl()) {
                                DPRINTF(Decode, "Early executing control inst: %s\n", *output_inst);
                                ExecContext context(cpu, *cpu.threads[output_inst->id.threadId], output_inst);

                                Fault fault = output_inst->staticInst->execute(&context, output_inst->traceData);

                                if (output_inst->traceData)
                                    output_inst->traceData->setPredicate(context.readPredicate());

                                if (fault != NoFault)
                                {
                                    DPRINTF(Decode, "Fault in execute of inst: %s fault: %s\n",
                                            *output_inst, fault->name());
                                    panic("Fault in early branch execution of inst: %s fault: %s\n",
                                          *output_inst, fault->name());
                                }
                                DPRINTF(Decode, "Executing control inst: %s\n", *output_inst);
                                context.writeback(output_inst->staticInst);
                                output_inst->executed = true;
                                earlyBranch(output_inst, fault, prediction);
                                dumpIfBranchesExecuted(prediction);
                                decode_info.expectedStreamSeqNum = inst_ptr_4_GUI->id.streamSeqNum;
                                ThreadContext *thread = cpu.getContext(inst_ptr_4_GUI->id.threadId);
                                prediction = BranchData(prediction.reason,
                                                        inst_ptr_4_GUI->id.threadId,
                                                        inst_ptr_4_GUI->id.streamSeqNum, prediction.isStreamChange() ? decode_info.predictionSeqNum + 1 : decode_info.predictionSeqNum,
                                                        *thread->pcState().clone(), inst_ptr_4_GUI);

                                /* Mark with a new prediction number by the stream number of the
                                 *  instruction causing the prediction */
                                if (prediction.isStreamChange())
                                {
                                    decode_info.predictionSeqNum++;
                                }
                            }
                            packIntoOutput(output_inst, insts_out, &output_index);
                            last_inst_pc = output_inst->pc->instAddr();
                            /* Continue the execution normally */
                            DPRINTF(MinorGUI, "Log4GUI: decode: %d: %d: %x: %s\n",
                                    curTick(),
                                    decode_is_stalling,
                                    output_inst->pc->instAddr(),
                                    output_inst->staticInst->disassemble(output_inst->pc->instAddr()));
                            auto next_pc = std::unique_ptr<PCStateBase>(output_inst->pc->clone());
                            next_pc->advance();
                            last_inst_pc = next_pc->instAddr();
                            was_stalling = decode_is_stalling;
                            decode_is_stalling = false;
                            if (prediction.isStreamChange())
                            {
                                //last_inst_pc = prediction.target->instAddr();
                                has_branched = true;
                            }
                        }
                        else
                        {
                            DPRINTF(Decode, "Cannot pass to Execute: %s\n", *output_inst);
                            instWaitingDependencies[tid] = true;
                            instWaitingDependenciesPtr[tid] = output_inst;
                            /* Stall here */
                            DPRINTF(MinorGUI, "Log4GUI: decode: %d: %d: %x: %s\n",
                                    curTick(),
                                    decode_is_stalling,
                                    output_inst->pc->instAddr(),
                                    output_inst->staticInst->disassemble(output_inst->pc->instAddr()));
                            was_stalling = decode_is_stalling;
                            decode_is_stalling = true;
                            auto next_pc = std::unique_ptr<PCStateBase>(output_inst->pc->clone());
                            next_pc->advance();
                            last_inst_pc = next_pc->instAddr();
                            has_branched = false;
                            break;
                        }
                    }
                }

                /* The rest of the output (if any) should already have been packed
                 *  with bubble instructions by insts_out's initialisation */
            }

            /** Reserve a slot in the next stage and output data */
            *predictionOut.inputWire = prediction;

            /* If we generated output, reserve space for the result in the next stage
             *  and mark the stage as being active this cycle */
            reserveSpaceInNextStage(insts_out, tid);

            /* If we still have input to process and somewhere to put it,
             *  mark stage as active */
            markStageActivity();

            /* Make sure the input (if any left) is pushed. In case of
             *  macroinstructions, push only when last one was packed into output */
            if (!macroInstPending)
            {
                pushTailInpBuffer();
            }

            /* Format >>> Log4GUI: decode: tick: stall_bit: inst_address: assembly */
            /*if (inst_ptr_4_GUI)
            {
                DPRINTF(MinorGUI, "Log4GUI: decode: %d: %d: %x: %s\n",
                        curTick(),
                        is_stalling,
                        inst_ptr_4_GUI->pc->instAddr(),
                        inst_ptr_4_GUI->staticInst->disassemble(inst_ptr_4_GUI->pc->instAddr()));
            }*/
        }

        inline ThreadID
        Decode::getScheduledThread()
        {
            /* Select thread via policy. */
            std::vector<ThreadID> priority_list;

            switch (cpu.threadPolicy)
            {
            case enums::SingleThreaded:
                priority_list.push_back(0);
                break;
            case enums::RoundRobin:
                priority_list = cpu.roundRobinPriority(threadPriority);
                break;
            case enums::Random:
                priority_list = cpu.randomPriority();
                break;
            default:
                panic("Unknown fetch policy");
            }

            for (auto tid : priority_list)
            {
                if (/* getInput(tid) &&  */ !decodeInfo[tid].blocked)
                {
                    threadPriority = tid;
                    return tid;
                }
            }

            DPRINTF(Decode, "No thread selected\n");

            return InvalidThreadID;
        }

        bool
        Decode::isDrained()
        {
            for (const auto &buffer : inputBuffer)
            {
                if (!buffer.empty())
                    return false;
            }

            return (*inp.outputWire).isBubble();
        }

        Decode::DecodeStats::DecodeStats(MinorCPU *cpu)
            : statistics::Group(cpu, "decode"),
              ADD_STAT(intInstructions, statistics::units::Count::get(),
                       "Number of integer instructions successfully decoded"),
              ADD_STAT(fpInstructions, statistics::units::Count::get(),
                       "Number of floating point instructions successfully decoded"),
              ADD_STAT(vecInstructions, statistics::units::Count::get(),
                       "Number of SIMD instructions successfully decoded"),
              ADD_STAT(loadInstructions, statistics::units::Count::get(),
                       "Number of memory load instructions successfully decoded"),
              ADD_STAT(storeInstructions, statistics::units::Count::get(),
                       "Number of memory store instructions successfully decoded"),
              ADD_STAT(amoInstructions, statistics::units::Count::get(),
                       "Number of memory atomic instructions successfully decoded")
        {
            intInstructions
                .flags(statistics::total);
            fpInstructions
                .flags(statistics::total);
            vecInstructions
                .flags(statistics::total);
            loadInstructions
                .flags(statistics::total);
            storeInstructions
                .flags(statistics::total);
            amoInstructions
                .flags(statistics::total);
        }

        void
        Decode::minorTrace() const
        {
            std::ostringstream data;

            if (decodeInfo[0].blocked)
                data << 'B';
            else
                (*out.inputWire).reportData(data);

            minor::minorTrace("insts=%s\n", data.str());
            inputBuffer[0].minorTrace();
        }

    } // namespace minor
} // namespace gem5
