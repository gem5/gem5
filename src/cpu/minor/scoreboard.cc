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

#include "arch/registers.hh"
#include "cpu/minor/scoreboard.hh"
#include "cpu/reg_class.hh"
#include "debug/MinorScoreboard.hh"
#include "debug/MinorTiming.hh"

namespace Minor
{

bool
Scoreboard::findIndex(RegIndex reg, Index &scoreboard_index)
{
    RegClass reg_class = regIdxToClass(reg);
    bool ret = false;

    if (reg == TheISA::ZeroReg) {
        /* Don't bother with the zero register */
        ret = false;
    } else {
        switch (reg_class)
        {
          case IntRegClass:
            scoreboard_index = reg;
            ret = true;
            break;
          case FloatRegClass:
            scoreboard_index = TheISA::NumIntRegs + TheISA::NumCCRegs +
                reg - TheISA::FP_Reg_Base;
            ret = true;
            break;
          case CCRegClass:
            scoreboard_index = TheISA::NumIntRegs + reg - TheISA::FP_Reg_Base;
            ret = true;
            break;
          case MiscRegClass:
              /* Don't bother with Misc registers */
            ret = false;
            break;
        }
    }

    return ret;
}

/** Flatten a RegIndex, irrespective of what reg type it's pointing to */
static TheISA::RegIndex
flattenRegIndex(TheISA::RegIndex reg, ThreadContext *thread_context)
{
    RegClass reg_class = regIdxToClass(reg);
    TheISA::RegIndex ret = reg;

    switch (reg_class)
    {
      case IntRegClass:
        ret = thread_context->flattenIntIndex(reg);
        break;
      case FloatRegClass:
        ret = thread_context->flattenFloatIndex(reg);
        break;
      case CCRegClass:
        ret = thread_context->flattenCCIndex(reg);
        break;
      case MiscRegClass:
        /* Don't bother to flatten misc regs as we don't need them here */
        /* return thread_context->flattenMiscIndex(reg); */
        ret = reg;
        break;
    }

    return ret;
}

void
Scoreboard::markupInstDests(MinorDynInstPtr inst, Cycles retire_time,
    ThreadContext *thread_context, bool mark_unpredictable)
{
    if (inst->isFault())
        return;

    StaticInstPtr staticInst = inst->staticInst;
    unsigned int num_dests = staticInst->numDestRegs();

    /** Mark each destination register */
    for (unsigned int dest_index = 0; dest_index < num_dests;
        dest_index++)
    {
        RegIndex reg = flattenRegIndex(
            staticInst->destRegIdx(dest_index), thread_context);
        Index index;

        if (findIndex(reg, index)) {
            if (mark_unpredictable)
                numUnpredictableResults[index]++;

            inst->flatDestRegIdx[dest_index] = reg;

            numResults[index]++;
            returnCycle[index] = retire_time;
            /* We should be able to rely on only being given accending
             *  execSeqNums, but sanity check */
            if (inst->id.execSeqNum > writingInst[index]) {
                writingInst[index] = inst->id.execSeqNum;
                fuIndices[index] = inst->fuIndex;
            }

            DPRINTF(MinorScoreboard, "Marking up inst: %s"
                " regIndex: %d final numResults: %d returnCycle: %d\n",
                *inst, index, numResults[index], returnCycle[index]);
        } else {
            /* Use ZeroReg to mark invalid/untracked dests */
            inst->flatDestRegIdx[dest_index] = TheISA::ZeroReg;
        }
    }
}

InstSeqNum
Scoreboard::execSeqNumToWaitFor(MinorDynInstPtr inst,
    ThreadContext *thread_context)
{
    InstSeqNum ret = 0;

    if (inst->isFault())
        return ret;

    StaticInstPtr staticInst = inst->staticInst;
    unsigned int num_srcs = staticInst->numSrcRegs();

    for (unsigned int src_index = 0; src_index < num_srcs; src_index++) {
        RegIndex reg = flattenRegIndex(staticInst->srcRegIdx(src_index),
            thread_context);
        unsigned short int index;

        if (findIndex(reg, index)) {
            if (writingInst[index] > ret)
                ret = writingInst[index];
        }
    }

    DPRINTF(MinorScoreboard, "Inst: %s depends on execSeqNum: %d\n",
        *inst, ret);

    return ret;
}

void
Scoreboard::clearInstDests(MinorDynInstPtr inst, bool clear_unpredictable)
{
    if (inst->isFault())
        return;

    StaticInstPtr staticInst = inst->staticInst;
    unsigned int num_dests = staticInst->numDestRegs();

    /** Mark each destination register */
    for (unsigned int dest_index = 0; dest_index < num_dests;
        dest_index++)
    {
        RegIndex reg = inst->flatDestRegIdx[dest_index];
        Index index;

        if (findIndex(reg, index)) {
            if (clear_unpredictable && numUnpredictableResults[index] != 0)
                numUnpredictableResults[index] --;

            numResults[index] --;

            if (numResults[index] == 0) {
                returnCycle[index] = Cycles(0);
                writingInst[index] = 0;
                fuIndices[index] = -1;
            }

            DPRINTF(MinorScoreboard, "Clearing inst: %s"
                " regIndex: %d final numResults: %d\n",
                *inst, index, numResults[index]);
        }
    }
}

bool
Scoreboard::canInstIssue(MinorDynInstPtr inst,
    const std::vector<Cycles> *src_reg_relative_latencies,
    const std::vector<bool> *cant_forward_from_fu_indices,
    Cycles now, ThreadContext *thread_context)
{
    /* Always allow fault to be issued */
    if (inst->isFault())
        return true;

    StaticInstPtr staticInst = inst->staticInst;
    unsigned int num_srcs = staticInst->numSrcRegs();

    /* Default to saying you can issue */
    bool ret = true;

    unsigned int num_relative_latencies = 0;
    Cycles default_relative_latency = Cycles(0);

    /* Where relative latencies are given, the default is the last
     *  one as that allows the rel. lat. list to be shorted than the
     *  number of src. regs */
    if (src_reg_relative_latencies &&
        src_reg_relative_latencies->size() != 0)
    {
        num_relative_latencies = src_reg_relative_latencies->size();
        default_relative_latency = (*src_reg_relative_latencies)
            [num_relative_latencies-1];
    }

    /* For each source register, find the latest result */
    unsigned int src_index = 0;
    while (src_index < num_srcs && /* More registers */
        ret /* Still possible */)
    {
        RegIndex reg = flattenRegIndex(staticInst->srcRegIdx(src_index),
            thread_context);
        unsigned short int index;

        if (findIndex(reg, index)) {
            bool cant_forward = fuIndices[index] != 1 &&
                cant_forward_from_fu_indices &&
                index < cant_forward_from_fu_indices->size() &&
                (*cant_forward_from_fu_indices)[index];

            Cycles relative_latency = (cant_forward ? Cycles(0) :
                (src_index >= num_relative_latencies ?
                    default_relative_latency :
                    (*src_reg_relative_latencies)[src_index]));

            if (returnCycle[index] > (now + relative_latency) ||
                numUnpredictableResults[index] != 0)
            {
                ret = false;
            }
        }
        src_index++;
    }

    if (DTRACE(MinorTiming)) {
        if (ret && num_srcs > num_relative_latencies &&
            num_relative_latencies != 0)
        {
            DPRINTF(MinorTiming, "Warning, inst: %s timing extra decode has"
                " more src. regs: %d than relative latencies: %d\n",
                staticInst->disassemble(0), num_srcs, num_relative_latencies);
        }
    }

    return ret;
}

void
Scoreboard::minorTrace() const
{
    std::ostringstream result_stream;

    bool printed_element = false;

    unsigned int i = 0;
    while (i < numRegs) {
        unsigned short int num_results = numResults[i];
        unsigned short int num_unpredictable_results =
            numUnpredictableResults[i];

        if (!(num_results == 0 && num_unpredictable_results == Cycles(0))) {
            if (printed_element)
                result_stream << ',';

            result_stream << '(' << i << ','
                 << num_results << '/'
                 << num_unpredictable_results << '/'
                 << returnCycle[i] << '/'
                 << writingInst[i] << ')';

            printed_element = true;
        }

        i++;
    }

    MINORTRACE("busy=%s\n", result_stream.str());
}

}
