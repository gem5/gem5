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

#include <iomanip>
#include <sstream>

#include "arch/isa.hh"
#include "arch/registers.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/trace.hh"
#include "cpu/base.hh"
#include "cpu/reg_class.hh"
#include "debug/MinorExecute.hh"
#include "enums/OpClass.hh"

namespace Minor
{

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

MinorDynInstPtr MinorDynInst::bubbleInst = NULL;

void
MinorDynInst::init()
{
    if (!bubbleInst) {
        bubbleInst = new MinorDynInst();
        assert(bubbleInst->isBubble());
        /* Make bubbleInst immortal */
        bubbleInst->incref();
    }
}

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
    else
        os << id;
}

std::ostream &
operator <<(std::ostream &os, const MinorDynInst &inst)
{
    os << inst.id << " pc: 0x"
        << std::hex << inst.pc.instAddr() << std::dec << " (";

    if (inst.isFault())
        os << "fault: \"" << inst.fault->name() << '"';
    else if (inst.staticInst)
        os << inst.staticInst->getName();
    else
        os << "bubble";

    os << ')';

    return os;
}

/** Print a register in the form r<n>, f<n>, m<n>(<name>), z for integer,
 *  float, misc and zero registers given an 'architectural register number' */
static void
printRegName(std::ostream &os, TheISA::RegIndex reg)
{
    RegClass reg_class = regIdxToClass(reg);

    switch (reg_class)
    {
      case MiscRegClass:
        {
            TheISA::RegIndex misc_reg = reg - TheISA::Misc_Reg_Base;

        /* This is an ugly test because not all archs. have miscRegName */
#if THE_ISA == ARM_ISA
            os << 'm' << misc_reg << '(' << TheISA::miscRegName[misc_reg] <<
                ')';
#else
            os << 'n' << misc_reg;
#endif
        }
        break;
      case FloatRegClass:
        os << 'f' << static_cast<unsigned int>(reg - TheISA::FP_Reg_Base);
        break;
      case IntRegClass:
        if (reg == TheISA::ZeroReg) {
            os << 'z';
        } else {
            os << 'r' << static_cast<unsigned int>(reg);
        }
        break;
      case CCRegClass:
        os << 'c' << static_cast<unsigned int>(reg - TheISA::CC_Reg_Base);
    }
}

void
MinorDynInst::minorTraceInst(const Named &named_object) const
{
    if (isFault()) {
        MINORINST(&named_object, "id=F;%s addr=0x%x fault=\"%s\"\n",
            id, pc.instAddr(), fault->name());
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

#if THE_ISA == ARM_ISA
            regs_str << " extMachInst=" << std::hex << std::setw(16)
                << std::setfill('0') << staticInst->machInst << std::dec;
#endif
        }

        std::ostringstream flags;
        staticInst->printFlags(flags, " ");

        MINORINST(&named_object, "id=%s addr=0x%x inst=\"%s\" class=%s"
            " flags=\"%s\"%s%s\n",
            id, pc.instAddr(),
            (staticInst->opClass() == No_OpClass ?
                "(invalid)" : staticInst->disassemble(0,NULL)),
            Enums::OpClassStrings[staticInst->opClass()],
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

}
