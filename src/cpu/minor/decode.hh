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

/**
 * @file
 *
 *  Decode collects macro-ops from Fetch2 and splits them into micro-ops
 *  passed to Execute.
 */

#ifndef __CPU_MINOR_DECODE_HH__
#define __CPU_MINOR_DECODE_HH__

#include <vector>

#include "base/named.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/pipe_data.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{

/* Decode takes instructions from Fetch2 and decomposes them into micro-ops
 * to feed to Execute.  It generates a new sequence number for each
 * instruction: execSeqNum.
 */
class Decode : public Named
{
  protected:
    /** Pointer back to the containing CPU */
    MinorCPU &cpu;

    /** Input port carrying macro instructions from Fetch2 */
    Latch<ForwardInstData>::Output inp;
    /** Output port carrying micro-op decomposed instructions to Execute */
    Latch<ForwardInstData>::Input out;

    /** Interface to reserve space in the next stage */
    std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;

    /** Width of output of this stage/input of next in instructions */
    unsigned int outputWidth;

    /** If true, more than one input word can be processed each cycle if
     *  there is room in the output to contain its processed data */
    bool processMoreThanOneInput;

  public:
    /* Public for Pipeline to be able to pass it to Fetch2 */
    std::vector<InputBuffer<ForwardInstData>> inputBuffer;

  protected:
    /** Data members after this line are cycle-to-cycle state */

    struct DecodeThreadInfo
    {
        DecodeThreadInfo() {}

        DecodeThreadInfo(const DecodeThreadInfo& other) :
            inputIndex(other.inputIndex),
            inMacroop(other.inMacroop),
            execSeqNum(other.execSeqNum),
            blocked(other.blocked)
        {
            set(microopPC, other.microopPC);
        }


        /** Index into the inputBuffer's head marking the start of unhandled
         *  instructions */
        unsigned int inputIndex = 0;

        /** True when we're in the process of decomposing a micro-op and
         *  microopPC will be valid.  This is only the case when there isn't
         *  sufficient space in Executes input buffer to take the whole of a
         *  decomposed instruction and some of that instructions micro-ops must
         *  be generated in a later cycle */
        bool inMacroop = false;
        std::unique_ptr<PCStateBase> microopPC;

        /** Source of execSeqNums to number instructions. */
        InstSeqNum execSeqNum = InstId::firstExecSeqNum;

        /** Blocked indication for report */
        bool blocked = false;
    };

    std::vector<DecodeThreadInfo> decodeInfo;
    ThreadID threadPriority;

  protected:
    /** Get a piece of data to work on, or 0 if there is no data. */
    const ForwardInstData *getInput(ThreadID tid);

    /** Pop an element off the input buffer, if there are any */
    void popInput(ThreadID tid);

    /** Use the current threading policy to determine the next thread to
     *  decode from. */
    ThreadID getScheduledThread();
  public:
    Decode(const std::string &name,
        MinorCPU &cpu_,
        const BaseMinorCPUParams &params,
        Latch<ForwardInstData>::Output inp_,
        Latch<ForwardInstData>::Input out_,
        std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer);

  public:
    /** Pass on input/buffer data to the output if you can */
    void evaluate();

    void minorTrace() const;

    /** Is this stage drained?  For Decoed, draining is initiated by
     *  Execute halting Fetch1 causing Fetch2 to naturally drain
     *  into Decode and on to Execute which is responsible for
     *  actually killing instructions */
    bool isDrained();
};

} // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_DECODE_HH__ */
