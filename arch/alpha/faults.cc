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
 */

#include "arch/alpha/faults.hh"
#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "base/trace.hh"
#include "kern/kernel_stats.hh"

namespace AlphaISA
{

FaultVect AlphaMachineCheckFault::_vect = 0x0401;
FaultStat AlphaMachineCheckFault::_stat;

FaultVect AlphaAlignmentFault::_vect = 0x0301;
FaultStat AlphaAlignmentFault::_stat;

FaultName ResetFault::_name = "reset";
FaultVect ResetFault::_vect = 0x0001;
FaultStat ResetFault::_stat;

FaultName ArithmeticFault::_name = "arith";
FaultVect ArithmeticFault::_vect = 0x0501;
FaultStat ArithmeticFault::_stat;

FaultName InterruptFault::_name = "interrupt";
FaultVect InterruptFault::_vect = 0x0101;
FaultStat InterruptFault::_stat;

FaultName NDtbMissFault::_name = "dtb_miss_single";
FaultVect NDtbMissFault::_vect = 0x0201;
FaultStat NDtbMissFault::_stat;

FaultName PDtbMissFault::_name = "dtb_miss_double";
FaultVect PDtbMissFault::_vect = 0x0281;
FaultStat PDtbMissFault::_stat;

FaultName DtbPageFault::_name = "dfault";
FaultVect DtbPageFault::_vect = 0x0381;
FaultStat DtbPageFault::_stat;

FaultName DtbAcvFault::_name = "dfault";
FaultVect DtbAcvFault::_vect = 0x0381;
FaultStat DtbAcvFault::_stat;

FaultName ItbMissFault::_name = "itbmiss";
FaultVect ItbMissFault::_vect = 0x0181;
FaultStat ItbMissFault::_stat;

FaultName ItbPageFault::_name = "itbmiss";
FaultVect ItbPageFault::_vect = 0x0181;
FaultStat ItbPageFault::_stat;

FaultName ItbAcvFault::_name = "iaccvio";
FaultVect ItbAcvFault::_vect = 0x0081;
FaultStat ItbAcvFault::_stat;

FaultName UnimplementedOpcodeFault::_name = "opdec";
FaultVect UnimplementedOpcodeFault::_vect = 0x0481;
FaultStat UnimplementedOpcodeFault::_stat;

FaultName FloatEnableFault::_name = "fen";
FaultVect FloatEnableFault::_vect = 0x0581;
FaultStat FloatEnableFault::_stat;

FaultName PalFault::_name = "pal";
FaultVect PalFault::_vect = 0x2001;
FaultStat PalFault::_stat;

FaultName IntegerOverflowFault::_name = "intover";
FaultVect IntegerOverflowFault::_vect = 0x0501;
FaultStat IntegerOverflowFault::_stat;

#if FULL_SYSTEM

void AlphaFault::invoke(ExecContext * xc)
{
    DPRINTF(Fault, "Fault %s at PC: %#x\n", name(), xc->regs.pc);
    xc->cpu->recordEvent(csprintf("Fault %s", name()));

    assert(!xc->misspeculating());
    xc->kernelStats->fault(this);

    if (isA<ArithmeticFault>())
        panic("Arithmetic traps are unimplemented!");

    // exception restart address
    if (!isA<InterruptFault>() || !xc->inPalMode())
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR, xc->regs.pc);

    if (isA<PalFault>() || isA<ArithmeticFault>()) {
        // traps...  skip faulting instruction.
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR,
                   xc->readMiscReg(AlphaISA::IPR_EXC_ADDR) + 4);
    }

    if (!xc->inPalMode())
        AlphaISA::swap_palshadow(&(xc->regs), true);

    xc->regs.pc = xc->readMiscReg(AlphaISA::IPR_PAL_BASE) + vect();
    xc->regs.npc = xc->regs.pc + sizeof(MachInst);
}

#endif

} // namespace AlphaISA

/*Fault * ListOfFaults[] = {
        (Fault *)&NoFault,
        (Fault *)&ResetFault,
        (Fault *)&MachineCheckFault,
        (Fault *)&ArithmeticFault,
        (Fault *)&InterruptFault,
        (Fault *)&NDtbMissFault,
        (Fault *)&PDtbMissFault,
        (Fault *)&AlignmentFault,
        (Fault *)&DtbPageFault,
        (Fault *)&DtbAcvFault,
        (Fault *)&ItbMissFault,
        (Fault *)&ItbPageFault,
        (Fault *)&ItbAcvFault,
        (Fault *)&UnimplementedOpcodeFault,
        (Fault *)&FloatEnableFault,
        (Fault *)&PalFault,
        (Fault *)&IntegerOverflowFault,
        };

int NumFaults = sizeof(ListOfFaults) / sizeof(Fault *);*/
