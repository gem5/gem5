/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "mem/cache/cache.hh" // for dynamic cast
#include "mem/mem_interface.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/o3/alpha_cpu.hh"
#include "cpu/o3/alpha_params.hh"
#include "cpu/o3/comm.hh"

#if FULL_SYSTEM
#include "arch/alpha/osfpal.hh"
#include "arch/alpha/isa_traits.hh"
#endif

template <class Impl>
AlphaFullCPU<Impl>::AlphaFullCPU(Params &params)
    : FullO3CPU<Impl>(params)
{
    DPRINTF(FullCPU, "AlphaFullCPU: Creating AlphaFullCPU object.\n");

    this->fetch.setCPU(this);
    this->decode.setCPU(this);
    this->rename.setCPU(this);
    this->iew.setCPU(this);
    this->commit.setCPU(this);

    this->rob.setCPU(this);
}

template <class Impl>
void
AlphaFullCPU<Impl>::regStats()
{
    // Register stats for everything that has stats.
    this->fullCPURegStats();
    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
}

#if !FULL_SYSTEM

// Will probably need to know which thread is calling syscall
// Will need to pass that information in to the DynInst when it is constructed,
// so that this call can be made with the proper thread number.
template <class Impl>
void
AlphaFullCPU<Impl>::syscall(short thread_num)
{
    DPRINTF(FullCPU, "AlphaFullCPU: Syscall() called.\n\n");

    // Commit stage needs to run as well.
    this->commit.tick();

    squashStages();

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->funcExeInst);

    // Copy over all important state to xc once all the unrolling is done.
    copyToXC();

    // This is hardcoded to thread 0 while the CPU is only single threaded.
    this->thread[0]->syscall();

    // Copy over all important state back to CPU.
    copyFromXC();

    // Decrease funcExeInst by one as the normal commit will handle
    // incrememnting it.
    --(this->funcExeInst);
}

// This is not a pretty function, and should only be used if it is necessary
// to fake having everything squash all at once (ie for non-full system
// syscalls).  Maybe put this at the FullCPU level?
template <class Impl>
void
AlphaFullCPU<Impl>::squashStages()
{
    InstSeqNum rob_head = this->rob.readHeadSeqNum();

    // Now hack the time buffer to put this sequence number in the places
    // where the stages might read it.
    for (int i = 0; i < 5; ++i)
    {
        this->timeBuffer.access(-i)->commitInfo.doneSeqNum = rob_head;
    }

    this->fetch.squash(this->rob.readHeadNextPC());
    this->fetchQueue.advance();

    this->decode.squash();
    this->decodeQueue.advance();

    this->rename.squash();
    this->renameQueue.advance();
    this->renameQueue.advance();

    // Be sure to advance the IEW queues so that the commit stage doesn't
    // try to set an instruction as completed at the same time that it
    // might be deleting it.
    this->iew.squash();
    this->iewQueue.advance();
    this->iewQueue.advance();
    // Needs to tell the LSQ to write back all of its data
    this->iew.lsqWriteback();

    this->rob.squash(rob_head);
    this->commit.setSquashing();

    // Now hack the time buffer to clear the sequence numbers in the places
    // where the stages might read it.?
    for (int i = 0; i < 5; ++i)
    {
        this->timeBuffer.access(-i)->commitInfo.doneSeqNum = 0;
    }

}

#endif // FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::copyToXC()
{
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i)
    {
        renamed_reg = this->renameMap.lookup(i);
        this->cpuXC->setIntReg(i, this->regFile.readIntReg(renamed_reg));
        DPRINTF(FullCPU, "FullCPU: Copying register %i, has data %lli.\n",
                renamed_reg, this->regFile.intRegFile[renamed_reg]);
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = this->renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        this->cpuXC->setFloatRegBits(i,
            this->regFile.readFloatRegBits(renamed_reg));
    }

    this->cpuXC->setMiscReg(AlphaISA::Fpcr_DepTag,
                            this->regFile.readMiscReg(AlphaISA::Fpcr_DepTag));
    this->cpuXC->setMiscReg(AlphaISA::Uniq_DepTag,
                            this->regFile.readMiscReg(AlphaISA::Uniq_DepTag));
    this->cpuXC->setMiscReg(AlphaISA::Lock_Flag_DepTag,
                            this->regFile.readMiscReg(AlphaISA::Lock_Flag_DepTag));
    this->cpuXC->setMiscReg(AlphaISA::Lock_Addr_DepTag,
                            this->regFile.readMiscReg(AlphaISA::Lock_Addr_DepTag));

    this->cpuXC->setPC(this->rob.readHeadPC());
    this->cpuXC->setNextPC(this->cpuXC->readPC()+4);

#if !FULL_SYSTEM
    this->cpuXC->setFuncExeInst(this->funcExeInst);
#endif
}

// This function will probably mess things up unless the ROB is empty and
// there are no instructions in the pipeline.
template <class Impl>
void
AlphaFullCPU<Impl>::copyFromXC()
{
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i)
    {
        renamed_reg = this->renameMap.lookup(i);

        DPRINTF(FullCPU, "FullCPU: Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                renamed_reg, this->regFile.intRegFile[renamed_reg],
                this->cpuXC->readIntReg(i));

        this->regFile.setIntReg(renamed_reg, this->cpuXC->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = this->renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        this->regFile.setFloatRegBits(renamed_reg,
                                      this->cpuXC->readFloatRegBits(i));
    }

    // Then loop through the misc registers.
    this->regFile.setMiscReg(AlphaISA::Fpcr_DepTag,
                             this->cpuXC->readMiscReg(AlphaISA::Fpcr_DepTag));
    this->regFile.setMiscReg(AlphaISA::Uniq_DepTag,
                             this->cpuXC->readMiscReg(AlphaISA::Uniq_DepTag));
    this->regFile.setMiscReg(AlphaISA::Lock_Flag_DepTag,
                             this->cpuXC->readMiscReg(AlphaISA::Lock_Flag_DepTag));
    this->regFile.setMiscReg(AlphaISA::Lock_Addr_DepTag,
                             this->cpuXC->readMiscReg(AlphaISA::Lock_Addr_DepTag));

    // Then finally set the PC and the next PC.
//    regFile.pc = cpuXC->regs.pc;
//    regFile.npc = cpuXC->regs.npc;
#if !FULL_SYSTEM
    this->funcExeInst = this->cpuXC->readFuncExeInst();
#endif
}

#if FULL_SYSTEM

template <class Impl>
int
AlphaFullCPU<Impl>::readIntrFlag()
{
    return this->regFile.readIntrFlag();
}

template <class Impl>
void
AlphaFullCPU<Impl>::setIntrFlag(int val)
{
    this->regFile.setIntrFlag(val);
}

// Can force commit stage to squash and stuff.
template <class Impl>
Fault
AlphaFullCPU<Impl>::hwrei()
{
    if (!inPalMode())
        return new AlphaISA::UnimplementedOpcodeFault;

    this->setNextPC(this->regFile.miscRegs.readReg(AlphaISA::IPR_EXC_ADDR));

//    kernelStats.hwrei();

    if ((this->regFile.miscRegs.readReg(AlphaISA::IPR_EXC_ADDR) & 1) == 0)
//        AlphaISA::swap_palshadow(&regs, false);

    this->checkInterrupts = true;

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
bool
AlphaFullCPU<Impl>::simPalCheck(int palFunc)
{
//    kernelStats.callpal(palFunc);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            new SimExitEvent("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (this->system->breakpoint())
            return false;
        break;
    }

    return true;
}

// Probably shouldn't be able to switch to the trap handler as quickly as
// this.  Also needs to get the exception restart address from the commit
// stage.
template <class Impl>
void
AlphaFullCPU<Impl>::trap(Fault fault)
{
/*    // Keep in mind that a trap may be initiated by fetch if there's a TLB
    // miss
    uint64_t PC = this->commit.readCommitPC();

    DPRINTF(Fault, "Fault %s\n", fault->name());
    this->recordEvent(csprintf("Fault %s", fault->name()));

    //kernelStats.fault(fault);

    if (fault->isA<ArithmeticFault>())
        panic("Arithmetic traps are unimplemented!");

    // exception restart address - Get the commit PC
    if (!fault->isA<InterruptFault>() || !inPalMode(PC))
        this->regFile.miscRegs.setReg(AlphaISA::IPR_EXC_ADDR, PC);

    if (fault->isA<PalFault>() || fault->isA<ArithmeticFault>())
    //    || fault == InterruptFault && !PC_PAL(regs.pc)
        {
        // traps...  skip faulting instruction
        AlphaISA::MiscReg ipr_exc_addr =
            this->regFile.miscRegs.readReg(AlphaISA::IPR_EXC_ADDR);
        this->regFile.miscRegs.setReg(AlphaISA::IPR_EXC_ADDR,
                                      ipr_exc_addr + 4);
    }

    if (!inPalMode(PC))
        swapPALShadow(true);

    this->regFile.setPC(this->regFile.miscRegs.readReg(AlphaISA::IPR_PAL_BASE) +
                         (dynamic_cast<AlphaFault *>(fault.get()))->vect());
    this->regFile.setNextPC(PC + sizeof(MachInst));*/
}

template <class Impl>
void
AlphaFullCPU<Impl>::processInterrupts()
{
    // Check for interrupts here.  For now can copy the code that exists
    // within isa_fullsys_traits.hh.
}

// swap_palshadow swaps in the values of the shadow registers and
// swaps them with the values of the physical registers that map to the
// same logical index.
template <class Impl>
void
AlphaFullCPU<Impl>::swapPALShadow(bool use_shadow)
{
    if (palShadowEnabled == use_shadow)
        panic("swap_palshadow: wrong PAL shadow state");

    palShadowEnabled = use_shadow;

    // Will have to lookup in rename map to get physical registers, then
    // swap.
}

#endif // FULL_SYSTEM
