/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Gabe Black
 *          Kevin Lim
 */

#include "arch/alpha/faults.hh"
#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "base/trace.hh"
#if FULL_SYSTEM
#include "arch/alpha/ev5.hh"
#endif

namespace AlphaISA
{

FaultName MachineCheckFault::_name = "mchk";
FaultVect MachineCheckFault::_vect = 0x0401;
FaultStat MachineCheckFault::_count;

FaultName AlignmentFault::_name = "unalign";
FaultVect AlignmentFault::_vect = 0x0301;
FaultStat AlignmentFault::_count;

FaultName ResetFault::_name = "reset";
FaultVect ResetFault::_vect = 0x0001;
FaultStat ResetFault::_count;

FaultName ArithmeticFault::_name = "arith";
FaultVect ArithmeticFault::_vect = 0x0501;
FaultStat ArithmeticFault::_count;

FaultName InterruptFault::_name = "interrupt";
FaultVect InterruptFault::_vect = 0x0101;
FaultStat InterruptFault::_count;

FaultName NDtbMissFault::_name = "dtb_miss_single";
FaultVect NDtbMissFault::_vect = 0x0201;
FaultStat NDtbMissFault::_count;

FaultName PDtbMissFault::_name = "dtb_miss_double";
FaultVect PDtbMissFault::_vect = 0x0281;
FaultStat PDtbMissFault::_count;

FaultName DtbPageFault::_name = "dfault";
FaultVect DtbPageFault::_vect = 0x0381;
FaultStat DtbPageFault::_count;

FaultName DtbAcvFault::_name = "dfault";
FaultVect DtbAcvFault::_vect = 0x0381;
FaultStat DtbAcvFault::_count;

FaultName DtbAlignmentFault::_name = "unalign";
FaultVect DtbAlignmentFault::_vect = 0x0301;
FaultStat DtbAlignmentFault::_count;

FaultName ItbMissFault::_name = "itbmiss";
FaultVect ItbMissFault::_vect = 0x0181;
FaultStat ItbMissFault::_count;

FaultName ItbPageFault::_name = "itbmiss";
FaultVect ItbPageFault::_vect = 0x0181;
FaultStat ItbPageFault::_count;

FaultName ItbAcvFault::_name = "iaccvio";
FaultVect ItbAcvFault::_vect = 0x0081;
FaultStat ItbAcvFault::_count;

FaultName UnimplementedOpcodeFault::_name = "opdec";
FaultVect UnimplementedOpcodeFault::_vect = 0x0481;
FaultStat UnimplementedOpcodeFault::_count;

FaultName FloatEnableFault::_name = "fen";
FaultVect FloatEnableFault::_vect = 0x0581;
FaultStat FloatEnableFault::_count;

FaultName PalFault::_name = "pal";
FaultVect PalFault::_vect = 0x2001;
FaultStat PalFault::_count;

FaultName IntegerOverflowFault::_name = "intover";
FaultVect IntegerOverflowFault::_vect = 0x0501;
FaultStat IntegerOverflowFault::_count;

#if FULL_SYSTEM

void AlphaFault::invoke(ExecContext * xc)
{
    FaultBase::invoke(xc);
    countStat()++;

    // exception restart address
    if (setRestartAddress() || !xc->inPalMode())
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR, xc->readPC());

    if (skipFaultingInstruction()) {
        // traps...  skip faulting instruction.
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR,
                   xc->readMiscReg(AlphaISA::IPR_EXC_ADDR) + 4);
    }

    xc->setPC(xc->readMiscReg(AlphaISA::IPR_PAL_BASE) + vect());
    xc->setNextPC(xc->readPC() + sizeof(MachInst));
}

void ArithmeticFault::invoke(ExecContext * xc)
{
    FaultBase::invoke(xc);
    panic("Arithmetic traps are unimplemented!");
}

void DtbFault::invoke(ExecContext * xc)
{
    // Set fault address and flags.  Even though we're modeling an
    // EV5, we use the EV6 technique of not latching fault registers
    // on VPTE loads (instead of locking the registers until IPR_VA is
    // read, like the EV5).  The EV6 approach is cleaner and seems to
    // work with EV5 PAL code, but not the other way around.
    if (!xc->misspeculating()
        && !(reqFlags & VPTE) && !(reqFlags & NO_FAULT)) {
        // set VA register with faulting address
        xc->setMiscReg(AlphaISA::IPR_VA, vaddr);

        // set MM_STAT register flags
        xc->setMiscReg(AlphaISA::IPR_MM_STAT,
            (((EV5::Opcode(xc->getInst()) & 0x3f) << 11)
             | ((EV5::Ra(xc->getInst()) & 0x1f) << 6)
             | (flags & 0x3f)));

        // set VA_FORM register with faulting formatted address
        xc->setMiscReg(AlphaISA::IPR_VA_FORM,
            xc->readMiscReg(AlphaISA::IPR_MVPTBR) | (vaddr.vpn() << 3));
    }

    AlphaFault::invoke(xc);
}

void ItbFault::invoke(ExecContext * xc)
{
    if (!xc->misspeculating()) {
        xc->setMiscReg(AlphaISA::IPR_ITB_TAG, pc);
        xc->setMiscReg(AlphaISA::IPR_IFAULT_VA_FORM,
                       xc->readMiscReg(AlphaISA::IPR_IVPTBR) |
                       (AlphaISA::VAddr(pc).vpn() << 3));
    }

    AlphaFault::invoke(xc);
}

#endif

} // namespace AlphaISA

