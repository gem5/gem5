/*
 * Copyright (c) 2013-2014, 2016,2018 ARM Limited
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

#include "cpu/minor/dyn_inst.hh"

#include <iomanip>
#include <sstream>

#include "cpu/base.hh"
#include "cpu/minor/trace.hh"
#include "cpu/null_static_inst.hh"
#include "cpu/reg_class.hh"
#include "debug/MinorExecute.hh"
#include "enums/OpClass.hh"

namespace gem5
{

namespace minor
{

const InstSeqNum InstId::firstStreamSeqNum;
const InstSeqNum InstId::firstPredictionSeqNum;
const InstSeqNum InstId::firstLineSeqNum;
const InstSeqNum InstId::firstFetchSeqNum;
const InstSeqNum InstId::firstExecSeqNum;

std::ostream &
operator <<(std::ostream &os, const InstId &id)
{
    os << id.threadId << '/' << id.streamSeqNum << '.'
        << id.predictionSeqNum << '/' << id.lineSeqNum;

    /* Not all structures have fetch and exec sequence numbers */
    if (id.fetchSeqNum != 0) {
        os << '/' << id.fetchSeqNum;
        if (id.execSeqNum != 0)
            os << '.' << id.execSeqNum;
    }

    return os;
}

MinorDynInstPtr MinorDynInst::bubbleInst = []() {
    auto *inst = new MinorDynInst(nullStaticInstPtr);
    assert(inst->isBubble());
    // Make bubbleInst immortal.
    inst->incref();
    return inst;
}();

bool
MinorDynInst::isLastOpInInst() const
{
    assert(staticInst);
    return !(staticInst->isMicroop() && !staticInst->isLastMicroop());
}

bool
MinorDynInst::isNoCostInst() const
{
    return isInst() && staticInst->opClass() == No_OpClass;
}

void
MinorDynInst::reportData(std::ostream &os) const
{
    if (isBubble())
        os << "-";
    else if (isFault())
        os << "F;" << id;
    else if (translationFault != NoFault)
        os << "TF;" << id;
    else
        os << id;
}

std::ostream &
operator <<(std::ostream &os, const MinorDynInst &inst)
{
    if (!inst.pc) {
        os << inst.id << " pc: 0x???????? (bubble)";
        return os;
    }

    os << inst.id << " pc: 0x"
        << std::hex << inst.pc->instAddr() << std::dec << " (";

    if (inst.isFault())
        os << "fault: \"" << inst.fault->name() << '"';
    else if (inst.translationFault != NoFault)
        os << "translation fault: \"" << inst.translationFault->name() << '"';
    else if (inst.staticInst)
        os << inst.staticInst->getName();
    else
        os << "bubble";

    os << ')';

    return os;
}

/** Print a register in the form r<n>, f<n>, m<n>(<name>) for integer,
 *  float, and misc given an 'architectural register number' */
static void
printRegName(std::ostream &os, const RegId& reg)
{
    switch (reg.classValue()) {
      case InvalidRegClass:
        os << 'z';
        break;
      case MiscRegClass:
        {
            RegIndex misc_reg = reg.index();
            os << 'm' << misc_reg << '(' << reg << ')';
        }
        break;
      case FloatRegClass:
        os << 'f' << reg.index();
        break;
      case VecRegClass:
        os << 'v' << reg.index();
        break;
      case VecElemClass:
        os << reg;
        break;
      case IntRegClass:
        os << 'r' << reg.index();
        break;
      case CCRegClass:
        os << 'c' << reg.index();
        break;
      default:
        panic("Unknown register class: %d", (int)reg.classValue());
    }
}

void
MinorDynInst::minorTraceInst(const Named &named_object) const
{
    if (isFault()) {
        minorInst(named_object, "id=F;%s addr=0x%x fault=\"%s\"\n",
            id, pc ? pc->instAddr() : 0, fault->name());
    } else {
        unsigned int num_src_regs = staticInst->numSrcRegs();
        unsigned int num_dest_regs = staticInst->numDestRegs();

        std::ostringstream regs_str;

        /* Format lists of src and dest registers for microops and
         *  'full' instructions */
        if (!staticInst->isMacroop()) {
            regs_str << " srcRegs=";

            unsigned int src_reg = 0;
            while (src_reg < num_src_regs) {
                printRegName(regs_str, staticInst->srcRegIdx(src_reg));

                src_reg++;
                if (src_reg != num_src_regs)
                    regs_str << ',';
            }

            regs_str << " destRegs=";

            unsigned int dest_reg = 0;
            while (dest_reg < num_dest_regs) {
                printRegName(regs_str, staticInst->destRegIdx(dest_reg));

                dest_reg++;
                if (dest_reg != num_dest_regs)
                    regs_str << ',';
            }

            ccprintf(regs_str, " extMachInst=%160x", staticInst->getEMI());
        }

        std::ostringstream flags;
        staticInst->printFlags(flags, " ");

        minorInst(named_object, "id=%s addr=0x%x inst=\"%s\" class=%s"
            " flags=\"%s\"%s%s\n",
            id, pc ? pc->instAddr() : 0,
            (staticInst->opClass() == No_OpClass ?
                "(invalid)" : staticInst->disassemble(0,NULL)),
            enums::OpClassStrings[staticInst->opClass()],
            flags.str(),
            regs_str.str(),
            (predictedTaken ? " predictedTaken" : ""));
    }
}

MinorDynInst::~MinorDynInst()
{
    if (traceData)
        delete traceData;
}

} // namespace minor
} // namespace gem5
