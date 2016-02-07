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
    Reservable &next_stage_input_buffer) :
    Named(name),
    cpu(cpu_),
    inp(inp_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    outputWidth(params.executeInputWidth),
    processMoreThanOneInput(params.decodeCycleInput),
    inputBuffer(name + ".inputBuffer", "insts", params.decodeInputBufferSize),
    inputIndex(0),
    inMacroop(false),
    execSeqNum(InstId::firstExecSeqNum),
    blocked(false)
{
    if (outputWidth < 1)
        fatal("%s: executeInputWidth must be >= 1 (%d)\n", name, outputWidth);

    if (params.decodeInputBufferSize < 1) {
        fatal("%s: decodeInputBufferSize must be >= 1 (%d)\n", name,
        params.decodeInputBufferSize);
    }
}

const ForwardInstData *
Decode::getInput()
{
    /* Get insts from the inputBuffer to work with */
    if (!inputBuffer.empty()) {
        const ForwardInstData &head = inputBuffer.front();

        return (head.isBubble() ? NULL : &(inputBuffer.front()));
    } else {
        return NULL;
    }
}

void
Decode::popInput()
{
    if (!inputBuffer.empty())
        inputBuffer.pop();

    inputIndex = 0;
    inMacroop = false;
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
    inputBuffer.setTail(*inp.outputWire);
    ForwardInstData &insts_out = *out.inputWire;

    assert(insts_out.isBubble());

    blocked = false;

    if (!nextStageReserve.canReserve()) {
        blocked = true;
    } else {
        const ForwardInstData *insts_in = getInput();

        unsigned int output_index = 0;

        /* Pack instructions into the output while we can.  This may involve
         * using more than one input line */
        while (insts_in &&
           inputIndex < insts_in->width() && /* Still more input */
           output_index < outputWidth /* Still more output to fill */)
        {
            MinorDynInstPtr inst = insts_in->insts[inputIndex];

            if (inst->isBubble()) {
                /* Skip */
                inputIndex++;
                inMacroop = false;
            } else {
                StaticInstPtr static_inst = inst->staticInst;
                /* Static inst of a macro-op above the output_inst */
                StaticInstPtr parent_static_inst = NULL;
                MinorDynInstPtr output_inst = inst;

                if (inst->isFault()) {
                    DPRINTF(Decode, "Fault being passed: %d\n",
                        inst->fault->name());

                    inputIndex++;
                    inMacroop = false;
                } else if (static_inst->isMacroop()) {
                    /* Generate a new micro-op */
                    StaticInstPtr static_micro_inst;

                    /* Set up PC for the next micro-op emitted */
                    if (!inMacroop) {
                        microopPC = inst->pc;
                        inMacroop = true;
                    }

                    /* Get the micro-op static instruction from the
                     * static_inst. */
                    static_micro_inst =
                        static_inst->fetchMicroop(microopPC.microPC());

                    output_inst = new MinorDynInst(inst->id);
                    output_inst->pc = microopPC;
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
                        inputIndex, output_index,
                        (static_micro_inst->isLastMicroop() ?
                            "true" : "false"),
                        microopPC.instAddr(), microopPC.microPC(),
                        *output_inst);

                    /* Acknowledge that the static_inst isn't mine, it's my
                     * parent macro-op's */
                    parent_static_inst = static_inst;

                    static_micro_inst->advancePC(microopPC);

                    /* Step input if this is the last micro-op */
                    if (static_micro_inst->isLastMicroop()) {
                        inputIndex++;
                        inMacroop = false;
                    }
                } else {
                    /* Doesn't need decomposing, pass on instruction */
                    DPRINTF(Decode, "Passing on inst: %s inputIndex:"
                        " %d output_index: %d\n",
                        *output_inst, inputIndex, output_index);

                    parent_static_inst = static_inst;

                    /* Step input */
                    inputIndex++;
                    inMacroop = false;
                }

                /* Set execSeqNum of output_inst */
                output_inst->id.execSeqNum = execSeqNum;
                /* Add tracing */
#if TRACING_ON
                dynInstAddTracing(output_inst, parent_static_inst, cpu);
#endif

                /* Step to next sequence number */
                execSeqNum++;

                /* Correctly size the output before writing */
                if (output_index == 0) insts_out.resize(outputWidth);
                /* Push into output */
                insts_out.insts[output_index] = output_inst;
                output_index++;
            }

            /* Have we finished with the input? */
            if (inputIndex == insts_in->width()) {
                /* If we have just been producing micro-ops, we *must* have
                 * got to the end of that for inputIndex to be pushed past
                 * insts_in->width() */
                assert(!inMacroop);
                popInput();
                insts_in = NULL;

                if (processMoreThanOneInput) {
                    DPRINTF(Decode, "Wrapping\n");
                    insts_in = getInput();
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
        nextStageReserve.reserve();
    }

    /* If we still have input to process and somewhere to put it,
     *  mark stage as active */
    if (getInput() && nextStageReserve.canReserve())
        cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);

    /* Make sure the input (if any left) is pushed */
    inputBuffer.pushTail();
}

bool
Decode::isDrained()
{
    return inputBuffer.empty() && (*inp.outputWire).isBubble();
}

void
Decode::minorTrace() const
{
    std::ostringstream data;

    if (blocked)
        data << 'B';
    else
        (*out.inputWire).reportData(data);

    MINORTRACE("insts=%s\n", data.str());
    inputBuffer.minorTrace();
}

}
