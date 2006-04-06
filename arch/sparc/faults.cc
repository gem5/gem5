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

#include "arch/sparc/faults.hh"
#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "base/trace.hh"

namespace SparcISA
{

FaultName     InternalProcessorError::_name = "intprocerr";
TrapType      InternalProcessorError::_trapType = 0x029;
FaultPriority InternalProcessorError::_priority = 4;
FaultStat     InternalProcessorError::_count;

FaultName     MemAddressNotAligned::_name = "unalign";
TrapType      MemAddressNotAligned::_trapType = 0x034;
FaultPriority MemAddressNotAligned::_priority = 10;
FaultStat     MemAddressNotAligned::_count;

FaultName     PowerOnReset::_name = "pow_reset";
TrapType      PowerOnReset::_trapType = 0x001;
FaultPriority PowerOnReset::_priority = 0;
FaultStat     PowerOnReset::_count;

FaultName     WatchDogReset::_name = "watch_dog_reset";
TrapType      WatchDogReset::_trapType = 0x002;
FaultPriority WatchDogReset::_priority = 1;
FaultStat     WatchDogReset::_count;

FaultName     ExternallyInitiatedReset::_name = "extern_reset";
TrapType      ExternallyInitiatedReset::_trapType = 0x003;
FaultPriority ExternallyInitiatedReset::_priority = 1;
FaultStat     ExternallyInitiatedReset::_count;

FaultName     SoftwareInitiatedReset::_name = "software_reset";
TrapType      SoftwareInitiatedReset::_trapType = 0x004;
FaultPriority SoftwareInitiatedReset::_priority = 1;
FaultStat     SoftwareInitiatedReset::_count;

FaultName     REDStateException::_name = "red_counte";
TrapType      REDStateException::_trapType = 0x005;
FaultPriority REDStateException::_priority = 1;
FaultStat     REDStateException::_count;

FaultName     InstructionAccessException::_name = "inst_access";
TrapType      InstructionAccessException::_trapType = 0x008;
FaultPriority InstructionAccessException::_priority = 5;
FaultStat     InstructionAccessException::_count;

FaultName     InstructionAccessMMUMiss::_name = "inst_mmu";
TrapType      InstructionAccessMMUMiss::_trapType = 0x009;
FaultPriority InstructionAccessMMUMiss::_priority = 2;
FaultStat     InstructionAccessMMUMiss::_count;

FaultName     InstructionAccessError::_name = "inst_error";
TrapType      InstructionAccessError::_trapType = 0x00A;
FaultPriority InstructionAccessError::_priority = 3;
FaultStat     InstructionAccessError::_count;

FaultName     IllegalInstruction::_name = "illegal_inst";
TrapType      IllegalInstruction::_trapType = 0x010;
FaultPriority IllegalInstruction::_priority = 7;
FaultStat     IllegalInstruction::_count;

FaultName     PrivilegedOpcode::_name = "priv_opcode";
TrapType      PrivilegedOpcode::_trapType = 0x011;
FaultPriority PrivilegedOpcode::_priority = 6;
FaultStat     PrivilegedOpcode::_count;

FaultName     UnimplementedLDD::_name = "unimp_ldd";
TrapType      UnimplementedLDD::_trapType = 0x012;
FaultPriority UnimplementedLDD::_priority = 6;
FaultStat     UnimplementedLDD::_count;

FaultName     UnimplementedSTD::_name = "unimp_std";
TrapType      UnimplementedSTD::_trapType = 0x013;
FaultPriority UnimplementedSTD::_priority = 6;
FaultStat     UnimplementedSTD::_count;

FaultName     FpDisabled::_name = "fp_disabled";
TrapType      FpDisabled::_trapType = 0x020;
FaultPriority FpDisabled::_priority = 8;
FaultStat     FpDisabled::_count;

FaultName     FpExceptionIEEE754::_name = "fp_754";
TrapType      FpExceptionIEEE754::_trapType = 0x021;
FaultPriority FpExceptionIEEE754::_priority = 11;
FaultStat     FpExceptionIEEE754::_count;

FaultName     FpExceptionOther::_name = "fp_other";
TrapType      FpExceptionOther::_trapType = 0x022;
FaultPriority FpExceptionOther::_priority = 11;
FaultStat     FpExceptionOther::_count;

FaultName     TagOverflow::_name = "tag_overflow";
TrapType      TagOverflow::_trapType = 0x023;
FaultPriority TagOverflow::_priority = 14;
FaultStat     TagOverflow::_count;

FaultName     DivisionByZero::_name = "div_by_zero";
TrapType      DivisionByZero::_trapType = 0x028;
FaultPriority DivisionByZero::_priority = 15;
FaultStat     DivisionByZero::_count;

FaultName     DataAccessException::_name = "data_access";
TrapType      DataAccessException::_trapType = 0x030;
FaultPriority DataAccessException::_priority = 12;
FaultStat     DataAccessException::_count;

FaultName     DataAccessMMUMiss::_name = "data_mmu";
TrapType      DataAccessMMUMiss::_trapType = 0x031;
FaultPriority DataAccessMMUMiss::_priority = 12;
FaultStat     DataAccessMMUMiss::_count;

FaultName     DataAccessError::_name = "data_error";
TrapType      DataAccessError::_trapType = 0x032;
FaultPriority DataAccessError::_priority = 12;
FaultStat     DataAccessError::_count;

FaultName     DataAccessProtection::_name = "data_protection";
TrapType      DataAccessProtection::_trapType = 0x033;
FaultPriority DataAccessProtection::_priority = 12;
FaultStat     DataAccessProtection::_count;

FaultName     LDDFMemAddressNotAligned::_name = "unalign_lddf";
TrapType      LDDFMemAddressNotAligned::_trapType = 0x035;
FaultPriority LDDFMemAddressNotAligned::_priority = 10;
FaultStat     LDDFMemAddressNotAligned::_count;

FaultName     STDFMemAddressNotAligned::_name = "unalign_stdf";
TrapType      STDFMemAddressNotAligned::_trapType = 0x036;
FaultPriority STDFMemAddressNotAligned::_priority = 10;
FaultStat     STDFMemAddressNotAligned::_count;

FaultName     PrivilegedAction::_name = "priv_action";
TrapType      PrivilegedAction::_trapType = 0x037;
FaultPriority PrivilegedAction::_priority = 11;
FaultStat     PrivilegedAction::_count;

FaultName     LDQFMemAddressNotAligned::_name = "unalign_ldqf";
TrapType      LDQFMemAddressNotAligned::_trapType = 0x038;
FaultPriority LDQFMemAddressNotAligned::_priority = 10;
FaultStat     LDQFMemAddressNotAligned::_count;

FaultName     STQFMemAddressNotAligned::_name = "unalign_stqf";
TrapType      STQFMemAddressNotAligned::_trapType = 0x039;
FaultPriority STQFMemAddressNotAligned::_priority = 10;
FaultStat     STQFMemAddressNotAligned::_count;

FaultName     AsyncDataError::_name = "async_data";
TrapType      AsyncDataError::_trapType = 0x040;
FaultPriority AsyncDataError::_priority = 2;
FaultStat     AsyncDataError::_count;

//The enumerated faults

FaultName     CleanWindow::_name = "clean_win";
TrapType      CleanWindow::_baseTrapType = 0x024;
FaultPriority CleanWindow::_priority = 10;
FaultStat     CleanWindow::_count;

FaultName     InterruptLevelN::_name = "interrupt_n";
TrapType      InterruptLevelN::_baseTrapType = 0x041;
FaultStat     InterruptLevelN::_count;

FaultName     SpillNNormal::_name = "spill_n_normal";
TrapType      SpillNNormal::_baseTrapType = 0x080;
FaultPriority SpillNNormal::_priority = 9;
FaultStat     SpillNNormal::_count;

FaultName     SpillNOther::_name = "spill_n_other";
TrapType      SpillNOther::_baseTrapType = 0x0A0;
FaultPriority SpillNOther::_priority = 9;
FaultStat     SpillNOther::_count;

FaultName     FillNNormal::_name = "fill_n_normal";
TrapType      FillNNormal::_baseTrapType = 0x0C0;
FaultPriority FillNNormal::_priority = 9;
FaultStat     FillNNormal::_count;

FaultName     FillNOther::_name = "fill_n_other";
TrapType      FillNOther::_baseTrapType = 0x0E0;
FaultPriority FillNOther::_priority = 9;
FaultStat     FillNOther::_count;

FaultName     TrapInstruction::_name = "trap_inst_n";
TrapType      TrapInstruction::_baseTrapType = 0x100;
FaultPriority TrapInstruction::_priority = 16;
FaultStat     TrapInstruction::_count;

FaultName     UnimpFault::_name = "Unimplemented Simulator feature";
TrapType      UnimpFault::_trapType = 0x000;
FaultPriority UnimpFault::_priority = 0;
FaultStat     UnimpFault::_count;

#if FULL_SYSTEM

void SparcFault::invoke(ExecContext * xc)
{
    FaultBase::invoke(xc);
    countStat()++;

    //Use the SPARC trap state machine
    /*// exception restart address
    if (setRestartAddress() || !xc->inPalMode())
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR, xc->regs.pc);

    if (skipFaultingInstruction()) {
        // traps...  skip faulting instruction.
        xc->setMiscReg(AlphaISA::IPR_EXC_ADDR,
                   xc->readMiscReg(AlphaISA::IPR_EXC_ADDR) + 4);
    }

    if (!xc->inPalMode())
        AlphaISA::swap_palshadow(&(xc->regs), true);

    xc->regs.pc = xc->readMiscReg(AlphaISA::IPR_PAL_BASE) + vect();
    xc->regs.npc = xc->regs.pc + sizeof(MachInst);*/
}

void UnimpFault::invoke(ExecContext * xc)
{
    panic("Unimpfault: %s\n", panicStr.c_str());
}


#endif

} // namespace SparcISA

