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
 *
 * Authors: Andrew Bardsley
 */

#include "cpu/minor/decode.hh"
#include "cpu/minor/pipeline.hh"
#include "debug/Decode.hh"

namespace Minor
{

Decode::Decode(const std::string &name,
    MinorCPU &cpu_,
    MinorCPUParams &params,
    Latch<ForwardInstData>::Output inp_,
    Latch<ForwardInstData>::Input out_,
    std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer) :
    Named(name),
    cpu(cpu_),
    inp(inp_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    outputWidth(params.executeInputWidth),
    processMoreThanOneInput(params.decodeCycleInput),
    decodeInfo(params.numThreads),
    threadPriority(0)
{
    if (outputWidth < 1)
        fatal("%s: executeInputWidth must be >= 1 (%d)\n", name, outputWidth);

    if (params.decodeInputBufferSize < 1) {
        fatal("%s: decodeInputBufferSize must be >= 1 (%d)\n", name,
        params.decodeInputBufferSize);
    }

    /* Per-thread input buffers */
    for (ThreadID tid = 0; tid < params.numThreads; tid++) {
        inputBuffer.push_back(
            InputBuffer<ForwardInstData>(
                name + ".inputBuffer" + std::to_string(tid), "insts",
                params.decodeInputBufferSize));
    }
}

const ForwardInstData *
Decode::getInput(ThreadID tid)
{
    /* Get insts from the inputBuffer to work with */
    if (!inputBuffer[tid].empty()) {
        const ForwardInstData &head = inputBuffer[tid].front();

        return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
    } else {
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
        inst->staticInst, inst->pc, static_inst);

    /* Use the execSeqNum as the fetch sequence number as this most closely
     *  matches the other processor models' idea of fetch sequence */
    if (inst->traceData)
        inst->traceData->setFetchSeq(inst->id.execSeqNum);
}
#endif

void
Decode::evaluate()
{
    /* Push input onto appropriate input buffer */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

    ForwardInstData &insts_out = *out.inputWire;

    assert(insts_out.isBubble());

    for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
        decodeInfo[tid].blocked = !nextStageReserve[tid].canReserve();

    ThreadID tid = getScheduledThread();

    if (tid != InvalidThreadID) {
        DecodeThreadInfo &decode_info = decodeInfo[tid];
        const ForwardInstData *insts_in = getInput(tid);

        unsigned int output_index = 0;

        /* Pack instructions into the output while we can.  This may involve
         * using more than one input line */
        while (insts_in &&
           decode_info.inputIndex < insts_in->width() && /* Still more input */
           output_index < outputWidth /* Still more output to fill */)
        {
            MinorDynInstPtr inst = insts_in->insts[decode_info.inputIndex];

            if (inst->isBubble()) {
                /* Skip */
                decode_info.inputIndex++;
                decode_info.inMacroop = false;
            } else {
                StaticInstPtr static_inst = inst->staticInst;
                /* Static inst of a macro-op above the output_inst */
                StaticInstPtr parent_static_inst = NULL;
                MinorDynInstPtr output_inst = inst;

                if (inst->isFault()) {
                    DPRINTF(Decode, "Fault being passed: %d\n",
                        inst->fault->name());

                    decode_info.inputIndex++;
                    decode_info.inMacroop = false;
                } else if (static_inst->isMacroop()) {
                    /* Generate a new micro-op */
                    StaticInstPtr static_micro_inst;

                    /* Set up PC for the next micro-op emitted */
                    if (!decode_info.inMacroop) {
                        decode_info.microopPC = inst->pc;
                        decode_info.inMacroop = true;
                    }

                    /* Get the micro-op static instruction from the
                     * static_inst. */
                    static_micro_inst =
                        static_inst->fetchMicroop(
                                decode_info.microopPC.microPC());

                    output_inst = new MinorDynInst(inst->id);
                    output_inst->pc = decode_info.microopPC;
                    output_inst->staticInst = static_micro_inst;
                    output_inst->fault = NoFault;

                    /* Allow a predicted next address only on the last
                     *  microop */
                    if (static_micro_inst->isLastMicroop()) {
                        output_inst->predictedTaken = inst->predictedTaken;
                        output_inst->predictedTarget = inst->predictedTarget;
                    }

                    DPRINTF(Decode, "Microop decomposition inputIndex:"
                        " %d output_index: %d lastMicroop: %s microopPC:"
                        " %d.%d inst: %d\n",
                        decode_info.inputIndex, output_index,
                        (static_micro_inst->isLastMicroop() ?
                            "true" : "false"),
                        decode_info.microopPC.instAddr(),
                        decode_info.microopPC.microPC(),
                        *output_inst);

                    /* Acknowledge that the static_inst isn't mine, it's my
                     * parent macro-op's */
                    parent_static_inst = static_inst;

                    static_micro_inst->advancePC(decode_info.microopPC);

                    /* Step input if this is the last micro-op */
                    if (static_micro_inst->isLastMicroop()) {
                        decode_info.inputIndex++;
                        decode_info.inMacroop = false;
                    }
                } else {
                    /* Doesn't need decomposing, pass on instruction */
                    DPRINTF(Decode, "Passing on inst: %s inputIndex:"
                        " %d output_index: %d\n",
                        *output_inst, decode_info.inputIndex, output_index);

                    parent_static_inst = static_inst;

                    /* Step input */
                    decode_info.inputIndex++;
                    decode_info.inMacroop = false;
                }

                /* Set execSeqNum of output_inst */
                output_inst->id.execSeqNum = decode_info.execSeqNum;
                /* Add tracing */
#if TRACING_ON
                dynInstAddTracing(output_inst, parent_static_inst, cpu);
#endif

                /* Step to next sequence number */
                decode_info.execSeqNum++;

                /* Correctly size the output before writing */
                if (output_index == 0) insts_out.resize(outputWidth);
                /* Push into output */
                insts_out.insts[output_index] = output_inst;
                output_index++;
            }

            /* Have we finished with the input? */
            if (decode_info.inputIndex == insts_in->width()) {
                /* If we have just been producing micro-ops, we *must* have
                 * got to the end of that for inputIndex to be pushed past
                 * insts_in->width() */
                assert(!decode_info.inMacroop);
                popInput(tid);
                insts_in = NULL;

                if (processMoreThanOneInput) {
                    DPRINTF(Decode, "Wrapping\n");
                    insts_in = getInput(tid);
                }
            }
        }

        /* The rest of the output (if any) should already have been packed
         *  with bubble instructions by insts_out's initialisation
         *
         *  for (; output_index < outputWidth; output_index++)
         *      assert(insts_out.insts[output_index]->isBubble());
         */
    }

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
            cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
            break;
        }
    }

    /* Make sure the input (if any left) is pushed */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].pushTail();
}

inline ThreadID
Decode::getScheduledThread()
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
        if (getInput(tid) && !decodeInfo[tid].blocked) {
            threadPriority = tid;
            return tid;
        }
    }

   return InvalidThreadID;
}

bool
Decode::isDrained()
{
    for (const auto &buffer : inputBuffer) {
        if (!buffer.empty())
            return false;
    }

    return (*inp.outputWire).isBubble();
}

void
Decode::minorTrace() const
{
    std::ostringstream data;

    if (decodeInfo[0].blocked)
        data << 'B';
    else
        (*out.inputWire).reportData(data);

    MINORTRACE("insts=%s\n", data.str());
    inputBuffer[0].minorTrace();
}

}
