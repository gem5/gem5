/*
 * Copyright (c) 2013-2014,2016 ARM Limited
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
 *
 * Authors: Andrew Bardsley
 */

#include "cpu/minor/fetch2.hh"

#include <string>

#include "arch/decoder.hh"
#include "arch/utility.hh"
#include "cpu/minor/pipeline.hh"
#include "cpu/pred/bpred_unit.hh"
#include "debug/Branch.hh"
#include "debug/Fetch.hh"
#include "debug/MinorTrace.hh"

namespace Minor
{

Fetch2::Fetch2(const std::string &name,
    MinorCPU &cpu_,
    MinorCPUParams &params,
    Latch<ForwardLineData>::Output inp_,
    Latch<BranchData>::Output branchInp_,
    Latch<BranchData>::Input predictionOut_,
    Latch<ForwardInstData>::Input out_,
    std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer) :
    Named(name),
    cpu(cpu_),
    inp(inp_),
    branchInp(branchInp_),
    predictionOut(predictionOut_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    outputWidth(params.decodeInputWidth),
    processMoreThanOneInput(params.fetch2CycleInput),
    branchPredictor(*params.branchPred),
    fetchInfo(params.numThreads),
    threadPriority(0)
{
    if (outputWidth < 1)
        fatal("%s: decodeInputWidth must be >= 1 (%d)\n", name, outputWidth);

    if (params.fetch2InputBufferSize < 1) {
        fatal("%s: fetch2InputBufferSize must be >= 1 (%d)\n", name,
        params.fetch2InputBufferSize);
    }

    /* Per-thread input buffers */
    for (ThreadID tid = 0; tid < params.numThreads; tid++) {
        inputBuffer.push_back(
            InputBuffer<ForwardLineData>(
                name + ".inputBuffer" + std::to_string(tid), "lines",
                params.fetch2InputBufferSize));
    }
}

const ForwardLineData *
Fetch2::getInput(ThreadID tid)
{
    /* Get a line from the inputBuffer to work with */
    if (!inputBuffer[tid].empty()) {
        return &(inputBuffer[tid].front());
    } else {
        return NULL;
    }
}

void
Fetch2::popInput(ThreadID tid)
{
    if (!inputBuffer[tid].empty()) {
        inputBuffer[tid].front().freeLine();
        inputBuffer[tid].pop();
    }

    fetchInfo[tid].inputIndex = 0;
}

void
Fetch2::dumpAllInput(ThreadID tid)
{
    DPRINTF(Fetch, "Dumping whole input buffer\n");
    while (!inputBuffer[tid].empty())
        popInput(tid);

    fetchInfo[tid].inputIndex = 0;
    fetchInfo[tid].havePC = false;
}

void
Fetch2::updateBranchPrediction(const BranchData &branch)
{
    MinorDynInstPtr inst = branch.inst;

    /* Don't even consider instructions we didn't try to predict or faults */
    if (inst->isFault() || !inst->triedToPredict)
        return;

    switch (branch.reason) {
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
            branch.target, true, inst->id.threadId);
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
            branch.target /* Not used */, false, inst->id.threadId);
        // Update after squashing to accomodate O3CPU
        // using the branch prediction code.
        branchPredictor.update(inst->id.fetchSeqNum,
            inst->id.threadId);
        break;
      case BranchData::BadlyPredictedBranchTarget:
        /* Predicted taken, was taken but to a different target */
        DPRINTF(Branch, "Branch mis-predicted target inst: %s target: %s\n",
            *inst, branch.target);
        branchPredictor.squash(inst->id.fetchSeqNum,
            branch.target, true, inst->id.threadId);
        break;
    }
}

void
Fetch2::predictBranch(MinorDynInstPtr inst, BranchData &branch)
{
    Fetch2ThreadInfo &thread = fetchInfo[inst->id.threadId];
    TheISA::PCState inst_pc = inst->pc;

    assert(!inst->predictedTaken);

    /* Skip non-control/sys call instructions */
    if (inst->staticInst->isControl() ||
        inst->staticInst->isSyscall())
    {
        /* Tried to predict */
        inst->triedToPredict = true;

        DPRINTF(Branch, "Trying to predict for inst: %s\n", *inst);

        if (branchPredictor.predict(inst->staticInst,
            inst->id.fetchSeqNum, inst_pc,
            inst->id.threadId))
        {
            inst->predictedTaken = true;
            inst->predictedTarget = inst_pc;
            branch.target = inst_pc;
        }
    } else {
        DPRINTF(Branch, "Not attempting prediction for inst: %s\n", *inst);
    }

    /* If we predict taken, set branch and update sequence numbers */
    if (inst->predictedTaken) {
        /* Update the predictionSeqNum and remember the streamSeqNum that it
         *  was associated with */
        thread.expectedStreamSeqNum = inst->id.streamSeqNum;

        BranchData new_branch = BranchData(BranchData::BranchPrediction,
            inst->id.threadId,
            inst->id.streamSeqNum, thread.predictionSeqNum + 1,
            inst->predictedTarget, inst);

        /* Mark with a new prediction number by the stream number of the
         *  instruction causing the prediction */
        thread.predictionSeqNum++;
        branch = new_branch;

        DPRINTF(Branch, "Branch predicted taken inst: %s target: %s"
            " new predictionSeqNum: %d\n",
            *inst, inst->predictedTarget, thread.predictionSeqNum);
    }
}

void
Fetch2::evaluate()
{
    /* Push input onto appropriate input buffer */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->id.threadId].setTail(*inp.outputWire);

    ForwardInstData &insts_out = *out.inputWire;
    BranchData prediction;
    BranchData &branch_inp = *branchInp.outputWire;

    assert(insts_out.isBubble());

    /* React to branches from Execute to update local branch prediction
     *  structures */
    updateBranchPrediction(branch_inp);

    /* If a branch arrives, don't try and do anything about it.  Only
     *  react to your own predictions */
    if (branch_inp.isStreamChange()) {
        DPRINTF(Fetch, "Dumping all input as a stream changing branch"
            " has arrived\n");
        dumpAllInput(branch_inp.threadId);
        fetchInfo[branch_inp.threadId].havePC = false;
    }

    assert(insts_out.isBubble());
    /* Even when blocked, clear out input lines with the wrong
     *  prediction sequence number */
    for (ThreadID tid = 0; tid < cpu.numThreads; tid++) {
        Fetch2ThreadInfo &thread = fetchInfo[tid];

        thread.blocked = !nextStageReserve[tid].canReserve();

        const ForwardLineData *line_in = getInput(tid);

        while (line_in &&
            thread.expectedStreamSeqNum == line_in->id.streamSeqNum &&
            thread.predictionSeqNum != line_in->id.predictionSeqNum)
        {
            DPRINTF(Fetch, "Discarding line %s"
                " due to predictionSeqNum mismatch (expected: %d)\n",
                line_in->id, thread.predictionSeqNum);

            popInput(tid);
            fetchInfo[tid].havePC = false;

            if (processMoreThanOneInput) {
                DPRINTF(Fetch, "Wrapping\n");
                line_in = getInput(tid);
            } else {
                line_in = NULL;
            }
        }
    }

    ThreadID tid = getScheduledThread();
    DPRINTF(Fetch, "Scheduled Thread: %d\n", tid);

    assert(insts_out.isBubble());
    if (tid != InvalidThreadID) {
        Fetch2ThreadInfo &fetch_info = fetchInfo[tid];

        const ForwardLineData *line_in = getInput(tid);

        unsigned int output_index = 0;

        /* Pack instructions into the output while we can.  This may involve
         * using more than one input line.  Note that lineWidth will be 0
         * for faulting lines */
        while (line_in &&
            (line_in->isFault() ||
                fetch_info.inputIndex < line_in->lineWidth) && /* More input */
            output_index < outputWidth && /* More output to fill */
            prediction.isBubble() /* No predicted branch */)
        {
            ThreadContext *thread = cpu.getContext(line_in->id.threadId);
            TheISA::Decoder *decoder = thread->getDecoderPtr();

            /* Discard line due to prediction sequence number being wrong but
             * without the streamSeqNum number having changed */
            bool discard_line =
                fetch_info.expectedStreamSeqNum == line_in->id.streamSeqNum &&
                fetch_info.predictionSeqNum != line_in->id.predictionSeqNum;

            /* Set the PC if the stream changes.  Setting havePC to false in
             *  a previous cycle handles all other change of flow of control
             *  issues */
            bool set_pc = fetch_info.lastStreamSeqNum != line_in->id.streamSeqNum;

            if (!discard_line && (!fetch_info.havePC || set_pc)) {
                /* Set the inputIndex to be the MachInst-aligned offset
                 *  from lineBaseAddr of the new PC value */
                fetch_info.inputIndex =
                    (line_in->pc.instAddr() & BaseCPU::PCMask) -
                    line_in->lineBaseAddr;
                DPRINTF(Fetch, "Setting new PC value: %s inputIndex: 0x%x"
                    " lineBaseAddr: 0x%x lineWidth: 0x%x\n",
                    line_in->pc, fetch_info.inputIndex, line_in->lineBaseAddr,
                    line_in->lineWidth);
                fetch_info.pc = line_in->pc;
                fetch_info.havePC = true;
                decoder->reset();
            }

            /* The generated instruction.  Leave as NULL if no instruction
             *  is to be packed into the output */
            MinorDynInstPtr dyn_inst = NULL;

            if (discard_line) {
                /* Rest of line was from an older prediction in the same
                 *  stream */
                DPRINTF(Fetch, "Discarding line %s (from inputIndex: %d)"
                    " due to predictionSeqNum mismatch (expected: %d)\n",
                    line_in->id, fetch_info.inputIndex,
                    fetch_info.predictionSeqNum);
            } else if (line_in->isFault()) {
                /* Pack a fault as a MinorDynInst with ->fault set */

                /* Make a new instruction and pick up the line, stream,
                 *  prediction, thread ids from the incoming line */
                dyn_inst = new MinorDynInst(line_in->id);

                /* Fetch and prediction sequence numbers originate here */
                dyn_inst->id.fetchSeqNum = fetch_info.fetchSeqNum;
                dyn_inst->id.predictionSeqNum = fetch_info.predictionSeqNum;
                /* To complete the set, test that exec sequence number has
                 *  not been set */
                assert(dyn_inst->id.execSeqNum == 0);

                dyn_inst->pc = fetch_info.pc;

                /* Pack a faulting instruction but allow other
                 *  instructions to be generated. (Fetch2 makes no
                 *  immediate judgement about streamSeqNum) */
                dyn_inst->fault = line_in->fault;
                DPRINTF(Fetch, "Fault being passed output_index: "
                    "%d: %s\n", output_index, dyn_inst->fault->name());
            } else {
                uint8_t *line = line_in->line;

                TheISA::MachInst inst_word;
                /* The instruction is wholly in the line, can just
                 *  assign */
                inst_word = TheISA::gtoh(
                    *(reinterpret_cast<TheISA::MachInst *>
                    (line + fetch_info.inputIndex)));

                if (!decoder->instReady()) {
                    decoder->moreBytes(fetch_info.pc,
                        line_in->lineBaseAddr + fetch_info.inputIndex,
                        inst_word);
                    DPRINTF(Fetch, "Offering MachInst to decoder addr: 0x%x\n",
                            line_in->lineBaseAddr + fetch_info.inputIndex);
                }

                /* Maybe make the above a loop to accomodate ISAs with
                 *  instructions longer than sizeof(MachInst) */

                if (decoder->instReady()) {
                    /* Make a new instruction and pick up the line, stream,
                     *  prediction, thread ids from the incoming line */
                    dyn_inst = new MinorDynInst(line_in->id);

                    /* Fetch and prediction sequence numbers originate here */
                    dyn_inst->id.fetchSeqNum = fetch_info.fetchSeqNum;
                    dyn_inst->id.predictionSeqNum = fetch_info.predictionSeqNum;
                    /* To complete the set, test that exec sequence number
                     *  has not been set */
                    assert(dyn_inst->id.execSeqNum == 0);

                    /* Note that the decoder can update the given PC.
                     *  Remember not to assign it until *after* calling
                     *  decode */
                    StaticInstPtr decoded_inst = decoder->decode(fetch_info.pc);
                    dyn_inst->staticInst = decoded_inst;

                    dyn_inst->pc = fetch_info.pc;
                    DPRINTF(Fetch, "decoder inst %s\n", *dyn_inst);

                    // Collect some basic inst class stats
                    if (decoded_inst->isLoad())
                        loadInstructions++;
                    else if (decoded_inst->isStore())
                        storeInstructions++;
                    else if (decoded_inst->isAtomic())
                        amoInstructions++;
                    else if (decoded_inst->isVector())
                        vecInstructions++;
                    else if (decoded_inst->isFloating())
                        fpInstructions++;
                    else if (decoded_inst->isInteger())
                        intInstructions++;

                    DPRINTF(Fetch, "Instruction extracted from line %s"
                        " lineWidth: %d output_index: %d inputIndex: %d"
                        " pc: %s inst: %s\n",
                        line_in->id,
                        line_in->lineWidth, output_index, fetch_info.inputIndex,
                        fetch_info.pc, *dyn_inst);

#if THE_ISA == X86_ISA || THE_ISA == ARM_ISA
                    /* In SE mode, it's possible to branch to a microop when
                     *  replaying faults such as page faults (or simply
                     *  intra-microcode branches in X86).  Unfortunately,
                     *  as Minor has micro-op decomposition in a separate
                     *  pipeline stage from instruction decomposition, the
                     *  following advancePC (which may follow a branch with
                     *  microPC() != 0) *must* see a fresh macroop.  This
                     *  kludge should be improved with an addition to PCState
                     *  but I offer it in this form for the moment
                     *
                     * X86 can branch within microops so we need to deal with
                     * the case that, after a branch, the first un-advanced PC
                     * may be pointing to a microop other than 0.  Once
                     * advanced, however, the microop number *must* be 0 */
                    fetch_info.pc.upc(0);
                    fetch_info.pc.nupc(1);
#endif

                    /* Advance PC for the next instruction */
                    TheISA::advancePC(fetch_info.pc, decoded_inst);

                    /* Predict any branches and issue a branch if
                     *  necessary */
                    predictBranch(dyn_inst, prediction);
                } else {
                    DPRINTF(Fetch, "Inst not ready yet\n");
                }

                /* Step on the pointer into the line if there's no
                 *  complete instruction waiting */
                if (decoder->needMoreBytes()) {
                    fetch_info.inputIndex += sizeof(TheISA::MachInst);

                DPRINTF(Fetch, "Updated inputIndex value PC: %s"
                    " inputIndex: 0x%x lineBaseAddr: 0x%x lineWidth: 0x%x\n",
                    line_in->pc, fetch_info.inputIndex, line_in->lineBaseAddr,
                    line_in->lineWidth);
                }
            }

            if (dyn_inst) {
                /* Step to next sequence number */
                fetch_info.fetchSeqNum++;

                /* Correctly size the output before writing */
                if (output_index == 0) {
                    insts_out.resize(outputWidth);
                }
                /* Pack the generated dynamic instruction into the output */
                insts_out.insts[output_index] = dyn_inst;
                output_index++;

                /* Output MinorTrace instruction info for
                 *  pre-microop decomposition macroops */
                if (DTRACE(MinorTrace) && !dyn_inst->isFault() &&
                    dyn_inst->staticInst->isMacroop())
                {
                    dyn_inst->minorTraceInst(*this);
                }
            }

            /* Remember the streamSeqNum of this line so we can tell when
             *  we change stream */
            fetch_info.lastStreamSeqNum = line_in->id.streamSeqNum;

            /* Asked to discard line or there was a branch or fault */
            if (!prediction.isBubble() || /* The remains of a
                    line with a prediction in it */
                line_in->isFault() /* A line which is just a fault */)
            {
                DPRINTF(Fetch, "Discarding all input on branch/fault\n");
                dumpAllInput(tid);
                fetch_info.havePC = false;
                line_in = NULL;
            } else if (discard_line) {
                /* Just discard one line, one's behind it may have new
                 *  stream sequence numbers.  There's a DPRINTF above
                 *  for this event */
                popInput(tid);
                fetch_info.havePC = false;
                line_in = NULL;
            } else if (fetch_info.inputIndex == line_in->lineWidth) {
                /* Got to end of a line, pop the line but keep PC
                 *  in case this is a line-wrapping inst. */
                popInput(tid);
                line_in = NULL;
            }

            if (!line_in && processMoreThanOneInput) {
                DPRINTF(Fetch, "Wrapping\n");
                line_in = getInput(tid);
            }
        }

        /* The rest of the output (if any) should already have been packed
         *  with bubble instructions by insts_out's initialisation */
    }
    if (tid == InvalidThreadID) {
        assert(insts_out.isBubble());
    }
    /** Reserve a slot in the next stage and output data */
    *predictionOut.inputWire = prediction;

    /* If we generated output, reserve space for the result in the next stage
     *  and mark the stage as being active this cycle */
    if (!insts_out.isBubble()) {
        /* Note activity of following buffer */
        cpu.activityRecorder->activity();
        insts_out.threadId = tid;
        nextStageReserve[tid].reserve();
    }

    /* If we still have input to process and somewhere to put it,
     *  mark stage as active */
    for (ThreadID i = 0; i < cpu.numThreads; i++)
    {
        if (getInput(i) && nextStageReserve[i].canReserve()) {
            cpu.activityRecorder->activateStage(Pipeline::Fetch2StageId);
            break;
        }
    }

    /* Make sure the input (if any left) is pushed */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->id.threadId].pushTail();
}

inline ThreadID
Fetch2::getScheduledThread()
{
    /* Select thread via policy. */
    std::vector<ThreadID> priority_list;

    switch (cpu.threadPolicy) {
      case Enums::SingleThreaded:
        priority_list.push_back(0);
        break;
      case Enums::RoundRobin:
        priority_list = cpu.roundRobinPriority(threadPriority);
        break;
      case Enums::Random:
        priority_list = cpu.randomPriority();
        break;
      default:
        panic("Unknown fetch policy");
    }

    for (auto tid : priority_list) {
        if (cpu.getContext(tid)->status() == ThreadContext::Active &&
            getInput(tid) &&
            !fetchInfo[tid].blocked) {
            threadPriority = tid;
            return tid;
        }
    }

   return InvalidThreadID;
}

bool
Fetch2::isDrained()
{
    for (const auto &buffer : inputBuffer) {
        if (!buffer.empty())
            return false;
    }

    return (*inp.outputWire).isBubble() &&
           (*predictionOut.inputWire).isBubble();
}

void
Fetch2::regStats()
{
    using namespace Stats;

    intInstructions
        .name(name() + ".int_instructions")
        .desc("Number of integer instructions successfully decoded")
        .flags(total);

    fpInstructions
        .name(name() + ".fp_instructions")
        .desc("Number of floating point instructions successfully decoded")
        .flags(total);

    vecInstructions
        .name(name() + ".vec_instructions")
        .desc("Number of SIMD instructions successfully decoded")
        .flags(total);

    loadInstructions
        .name(name() + ".load_instructions")
        .desc("Number of memory load instructions successfully decoded")
        .flags(total);

    storeInstructions
        .name(name() + ".store_instructions")
        .desc("Number of memory store instructions successfully decoded")
        .flags(total);

    amoInstructions
        .name(name() + ".amo_instructions")
        .desc("Number of memory atomic instructions successfully decoded")
        .flags(total);
}

void
Fetch2::minorTrace() const
{
    std::ostringstream data;

    if (fetchInfo[0].blocked)
        data << 'B';
    else
        (*out.inputWire).reportData(data);

    MINORTRACE("inputIndex=%d havePC=%d predictionSeqNum=%d insts=%s\n",
        fetchInfo[0].inputIndex, fetchInfo[0].havePC, fetchInfo[0].predictionSeqNum, data.str());
    inputBuffer[0].minorTrace();
}

}
