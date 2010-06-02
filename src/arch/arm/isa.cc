/*
 * Copyright (c) 2010 ARM Limited
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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/arm/isa.hh"

namespace ArmISA
{

MiscReg
ISA::readMiscRegNoEffect(int misc_reg)
{
    assert(misc_reg < NumMiscRegs);
    if (misc_reg == MISCREG_SPSR) {
        CPSR cpsr = miscRegs[MISCREG_CPSR];
        switch (cpsr.mode) {
          case MODE_USER:
            return miscRegs[MISCREG_SPSR];
          case MODE_FIQ:
            return miscRegs[MISCREG_SPSR_FIQ];
          case MODE_IRQ:
            return miscRegs[MISCREG_SPSR_IRQ];
          case MODE_SVC:
            return miscRegs[MISCREG_SPSR_SVC];
          case MODE_MON:
            return miscRegs[MISCREG_SPSR_MON];
          case MODE_ABORT:
            return miscRegs[MISCREG_SPSR_ABT];
          case MODE_UNDEFINED:
            return miscRegs[MISCREG_SPSR_UND];
          default:
            return miscRegs[MISCREG_SPSR];
        }
    }
    return miscRegs[misc_reg];
}


MiscReg
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    if (misc_reg == MISCREG_CPSR) {
        CPSR cpsr = miscRegs[misc_reg];
        Addr pc = tc->readPC();
        if (pc & (ULL(1) << PcJBitShift))
            cpsr.j = 1;
        else
            cpsr.j = 0;
        if (pc & (ULL(1) << PcTBitShift))
            cpsr.t = 1;
        else
            cpsr.t = 0;
        return cpsr;
    }
    if (misc_reg >= MISCREG_CP15_UNIMP_START &&
        misc_reg < MISCREG_CP15_END) {
        panic("Unimplemented CP15 register %s read.\n",
              miscRegName[misc_reg]);
    }
    switch (misc_reg) {
      case MISCREG_CLIDR:
        warn("The clidr register always reports 0 caches.\n");
        break;
      case MISCREG_CCSIDR:
        warn("The ccsidr register isn't implemented and "
                "always reads as 0.\n");
        break;
      case MISCREG_ID_PFR0:
        return 0x1031; // ThumbEE | !Jazelle | Thumb | ARM
    }
    return readMiscRegNoEffect(misc_reg);
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    assert(misc_reg < NumMiscRegs);
    if (misc_reg == MISCREG_SPSR) {
        CPSR cpsr = miscRegs[MISCREG_CPSR];
        switch (cpsr.mode) {
          case MODE_USER:
            miscRegs[MISCREG_SPSR] = val;
            return;
          case MODE_FIQ:
            miscRegs[MISCREG_SPSR_FIQ] = val;
            return;
          case MODE_IRQ:
            miscRegs[MISCREG_SPSR_IRQ] = val;
            return;
          case MODE_SVC:
            miscRegs[MISCREG_SPSR_SVC] = val;
            return;
          case MODE_MON:
            miscRegs[MISCREG_SPSR_MON] = val;
            return;
          case MODE_ABORT:
            miscRegs[MISCREG_SPSR_ABT] = val;
            return;
          case MODE_UNDEFINED:
            miscRegs[MISCREG_SPSR_UND] = val;
            return;
          default:
            miscRegs[MISCREG_SPSR] = val;
            return;
        }
    }
    miscRegs[misc_reg] = val;
}

void
ISA::setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
{
    MiscReg newVal = val;
    if (misc_reg == MISCREG_CPSR) {
        updateRegMap(val);
        CPSR cpsr = val;
        DPRINTF(Arm, "Updating CPSR to %#x f:%d i:%d a:%d mode:%#x\n",
                cpsr, cpsr.f, cpsr.i, cpsr.a, cpsr.mode);
        Addr npc = tc->readNextPC() & ~PcModeMask;
        if (cpsr.j)
            npc = npc | (ULL(1) << PcJBitShift);
        if (cpsr.t)
            npc = npc | (ULL(1) << PcTBitShift);

        tc->setNextPC(npc);
    }
    if (misc_reg >= MISCREG_CP15_UNIMP_START &&
        misc_reg < MISCREG_CP15_END) {
        panic("Unimplemented CP15 register %s wrote with %#x.\n",
              miscRegName[misc_reg], val);
    }
    switch (misc_reg) {
      case MISCREG_CPACR:
        {
            CPACR newCpacr = 0;
            CPACR valCpacr = val;
            newCpacr.cp10 = valCpacr.cp10;
            newCpacr.cp11 = valCpacr.cp11;
            if (newCpacr.cp10 != 0x3 || newCpacr.cp11 != 3) {
                panic("Disabling coprocessors isn't implemented.\n");
            }
            newVal = newCpacr;
        }
        break;
      case MISCREG_CSSELR:
        warn("The csselr register isn't implemented.\n");
        break;
      case MISCREG_FPSCR:
        {
            const uint32_t ones = (uint32_t)(-1);
            FPSCR fpscrMask = 0;
            fpscrMask.ioc = ones;
            fpscrMask.dzc = ones;
            fpscrMask.ofc = ones;
            fpscrMask.ufc = ones;
            fpscrMask.ixc = ones;
            fpscrMask.idc = ones;
            fpscrMask.len = ones;
            fpscrMask.stride = ones;
            fpscrMask.rMode = ones;
            fpscrMask.fz = ones;
            fpscrMask.dn = ones;
            fpscrMask.ahp = ones;
            fpscrMask.qc = ones;
            fpscrMask.v = ones;
            fpscrMask.c = ones;
            fpscrMask.z = ones;
            fpscrMask.n = ones;
            newVal = (newVal & (uint32_t)fpscrMask) |
                     (miscRegs[MISCREG_FPSCR] & ~(uint32_t)fpscrMask);
        }
        break;
      case MISCREG_FPEXC:
        {
            const uint32_t fpexcMask = 0x60000000;
            newVal = (newVal & fpexcMask) |
                     (miscRegs[MISCREG_FPEXC] & ~fpexcMask);
        }
        break;
      case MISCREG_SCTLR:
        {
            SCTLR sctlr = miscRegs[MISCREG_SCTLR];
            SCTLR new_sctlr = newVal;
            new_sctlr.nmfi =  (bool)sctlr.nmfi;
            miscRegs[MISCREG_SCTLR] = (MiscReg)new_sctlr;
            return;
        }
      case MISCREG_TLBTR:
      case MISCREG_MVFR0:
      case MISCREG_MVFR1:
      case MISCREG_MPIDR:
      case MISCREG_FPSID:
        return;
      case MISCREG_TLBIALLIS:
      case MISCREG_TLBIALL:
        warn("Need to flush all TLBs in MP\n");
        tc->getITBPtr()->flushAll();
        tc->getDTBPtr()->flushAll();
        return;
      case MISCREG_ITLBIALL:
        tc->getITBPtr()->flushAll();
        return;
      case MISCREG_DTLBIALL:
        tc->getDTBPtr()->flushAll();
        return;
      case MISCREG_TLBIMVAIS:
      case MISCREG_TLBIMVA:
        warn("Need to flush all TLBs in MP\n");
        tc->getITBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                bits(newVal, 7,0));
        tc->getDTBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                bits(newVal, 7,0));
        return;
      case MISCREG_TLBIASIDIS:
      case MISCREG_TLBIASID:
        warn("Need to flush all TLBs in MP\n");
        tc->getITBPtr()->flushAsid(bits(newVal, 7,0));
        tc->getDTBPtr()->flushAsid(bits(newVal, 7,0));
        return;
      case MISCREG_TLBIMVAAIS:
      case MISCREG_TLBIMVAA:
        warn("Need to flush all TLBs in MP\n");
        tc->getITBPtr()->flushMva(mbits(newVal, 31,12));
        tc->getDTBPtr()->flushMva(mbits(newVal, 31,12));
        return;
      case MISCREG_ITLBIMVA:
        tc->getITBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                bits(newVal, 7,0));
        return;
      case MISCREG_DTLBIMVA:
        tc->getDTBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                bits(newVal, 7,0));
        return;
      case MISCREG_ITLBIASID:
        tc->getITBPtr()->flushAsid(bits(newVal, 7,0));
        return;
      case MISCREG_DTLBIASID:
        tc->getDTBPtr()->flushAsid(bits(newVal, 7,0));
        return;
    }
    setMiscRegNoEffect(misc_reg, newVal);
}

}
