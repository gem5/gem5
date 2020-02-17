/*
 * Copyright (c) 2013-2014, 2016-2017 ARM Limited
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
 *  A simple instruction scoreboard for tracking dependencies in Execute.
 */

#ifndef __CPU_MINOR_SCOREBOARD_HH__
#define __CPU_MINOR_SCOREBOARD_HH__

#include "cpu/minor/cpu.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/trace.hh"

namespace Minor
{

/** A scoreboard of register dependencies including, for each register:
 *  The number of in-flight instructions which will generate a result for
 *  this register */
class Scoreboard : public Named
{
  public:
    /** The number of registers in the Scoreboard.  These
     *  are just the integer, CC and float registers packed
     *  together with integer regs in the range [0,NumIntRegs-1],
     *  CC regs in the range [NumIntRegs, NumIntRegs+NumCCRegs-1]
     *  and float regs in the range
     *  [NumIntRegs+NumCCRegs, NumFloatRegs+NumIntRegs+NumCCRegs-1] */
    const unsigned numRegs;

    /** Type to use when indexing numResults */
    typedef unsigned short int Index;

    /** Count of the number of in-flight instructions that
     *  have results for each register */
    std::vector<Index> numResults;

    /** Count of the number of results which can't be predicted */
    std::vector<Index> numUnpredictableResults;

    /** Index of the FU generating this result */
    std::vector<int> fuIndices;

    /** The estimated cycle number that the result will be presented.
     *  This can be offset from to allow forwarding to be simulated as
     *  long as instruction completion is *strictly* in order with
     *  respect to instructions with unpredictable result timing */
    std::vector<Cycles> returnCycle;

    /** The execute sequence number of the most recent inst to generate this
     *  register value */
    std::vector<InstSeqNum> writingInst;

  public:
    Scoreboard(const std::string &name) :
        Named(name),
        numRegs(TheISA::NumIntRegs + TheISA::NumCCRegs +
            TheISA::NumFloatRegs +
            (TheISA::NumVecRegs * TheISA::NumVecElemPerVecReg) +
            TheISA::NumVecPredRegs),
        numResults(numRegs, 0),
        numUnpredictableResults(numRegs, 0),
        fuIndices(numRegs, 0),
        returnCycle(numRegs, Cycles(0)),
        writingInst(numRegs, 0)
    { }

  public:
    /** Sets scoreboard_index to the index into numResults of the
     *  given register index.  Returns true if the given register
     *  is in the scoreboard and false if it isn't */
    bool findIndex(const RegId& reg, Index &scoreboard_index);

    /** Mark up an instruction's effects by incrementing
     *  numResults counts.  If mark_unpredictable is true, the inst's
     *  destination registers are marked as being unpredictable without
     *  an estimated retire time */
    void markupInstDests(MinorDynInstPtr inst, Cycles retire_time,
        ThreadContext *thread_context, bool mark_unpredictable);

    /** Clear down the dependencies for this instruction.  clear_unpredictable
     *  must match mark_unpredictable for the same inst. */
    void clearInstDests(MinorDynInstPtr inst, bool clear_unpredictable);

    /** Returns the exec sequence number of the most recent inst on
     *  which the given inst depends.  Useful for determining which
     *  inst must actually be committed before a dependent inst
     *  can call initiateAcc */
    InstSeqNum execSeqNumToWaitFor(MinorDynInstPtr inst,
        ThreadContext *thread_context);

    /** Can this instruction be issued.  Are any of its source registers
     *  due to be written by other marked-up instructions in flight */
    bool canInstIssue(MinorDynInstPtr inst,
        const std::vector<Cycles> *src_reg_relative_latencies,
        const std::vector<bool> *cant_forward_from_fu_indices,
        Cycles now, ThreadContext *thread_context);

    /** MinorTraceIF interface */
    void minorTrace() const;
};

}

#endif /* __CPU_MINOR_SCOREBOARD_HH__ */
