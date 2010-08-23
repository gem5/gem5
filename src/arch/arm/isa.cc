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

void
ISA::clear()
{
    SCTLR sctlr_rst = miscRegs[MISCREG_SCTLR_RST];

    memset(miscRegs, 0, sizeof(miscRegs));
    CPSR cpsr = 0;
    cpsr.mode = MODE_USER;
    miscRegs[MISCREG_CPSR] = cpsr;
    updateRegMap(cpsr);

    SCTLR sctlr = 0;
    sctlr.te = (bool)sctlr_rst.te;
    sctlr.nmfi = (bool)sctlr_rst.nmfi;
    sctlr.v = (bool)sctlr_rst.v;
    sctlr.u    = 1;
    sctlr.xp = 1;
    sctlr.rao2 = 1;
    sctlr.rao3 = 1;
    sctlr.rao4 = 1;
    miscRegs[MISCREG_SCTLR] = sctlr;
    miscRegs[MISCREG_SCTLR_RST] = sctlr_rst;


    /*
     * Technically this should be 0, but we don't support those
     * settings.
     */
    CPACR cpacr = 0;
    // Enable CP 10, 11
    cpacr.cp10 = 0x3;
    cpacr.cp11 = 0x3;
    miscRegs[MISCREG_CPACR] = cpacr;

    /* Start with an event in the mailbox */
    miscRegs[MISCREG_SEV_MAILBOX] = 1;

    /*
     * Implemented = '5' from "M5",
     * Variant = 0,
     */
    miscRegs[MISCREG_MIDR] =
        (0x35 << 24) | //Implementor is '5' from "M5"
        (0 << 20)    | //Variant
        (0xf << 16)  | //Architecture from CPUID scheme
        (0 << 4)     | //Primary part number
        (0 << 0)     | //Revision
        0;

    // Separate Instruction and Data TLBs.
    miscRegs[MISCREG_TLBTR] = 1;

    MVFR0 mvfr0 = 0;
    mvfr0.advSimdRegisters = 2;
    mvfr0.singlePrecision = 2;
    mvfr0.doublePrecision = 2;
    mvfr0.vfpExceptionTrapping = 0;
    mvfr0.divide = 1;
    mvfr0.squareRoot = 1;
    mvfr0.shortVectors = 1;
    mvfr0.roundingModes = 1;
    miscRegs[MISCREG_MVFR0] = mvfr0;

    MVFR1 mvfr1 = 0;
    mvfr1.flushToZero = 1;
    mvfr1.defaultNaN = 1;
    mvfr1.advSimdLoadStore = 1;
    mvfr1.advSimdInteger = 1;
    mvfr1.advSimdSinglePrecision = 1;
    mvfr1.advSimdHalfPrecision = 1;
    mvfr1.vfpHalfPrecision = 1;
    miscRegs[MISCREG_MVFR1] = mvfr1;

    miscRegs[MISCREG_MPIDR] = 0;

    // Reset values of PRRR and NMRR are implementation dependent

    miscRegs[MISCREG_PRRR] =
        (1 << 19) | // 19
        (0 << 18) | // 18
        (0 << 17) | // 17
        (1 << 16) | // 16
        (2 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (2 << 8)  | // 9:8
        (2 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (1 << 2)  | // 3:2
        0;          // 1:0
    miscRegs[MISCREG_NMRR] =
        (1 << 30) | // 31:30
        (0 << 26) | // 27:26
        (0 << 24) | // 25:24
        (3 << 22) | // 23:22
        (2 << 20) | // 21:20
        (0 << 18) | // 19:18
        (0 << 16) | // 17:16
        (1 << 14) | // 15:14
        (0 << 12) | // 13:12
        (2 << 10) | // 11:10
        (0 << 8)  | // 9:8
        (3 << 6)  | // 7:6
        (2 << 4)  | // 5:4
        (0 << 2)  | // 3:2
        0;          // 1:0

    //XXX We need to initialize the rest of the state.
}

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
        warn("Returning thumbEE disabled for now since we don't support CP14"
             "config registers and jumping to ThumbEE vectors\n");
        return 0x0031; // !ThumbEE | !Jazelle | Thumb | ARM
      case MISCREG_ID_MMFR0:
        return 0x03; //VMSAz7
      case MISCREG_CTR:
        return 0x86468006; // V7, 64 byte cache line, load/exclusive is exact
      case MISCREG_ACTLR:
        warn("Not doing anything for miscreg ACTLR\n");
        break;
      case MISCREG_PMCR:
      case MISCREG_PMCCNTR:
      case MISCREG_PMSELR:
        warn("Not doing anyhting for read to miscreg %s\n",
                miscRegName[misc_reg]);
        break;

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
    } else if (misc_reg >= MISCREG_CP15_UNIMP_START &&
        misc_reg < MISCREG_CP15_END) {
        panic("Unimplemented CP15 register %s wrote with %#x.\n",
              miscRegName[misc_reg], val);
    } else {
        switch (misc_reg) {
          case MISCREG_ITSTATE:
            {
                ITSTATE itstate = newVal;
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                cpsr.it1 = itstate.bottom2;
                cpsr.it2 = itstate.top6;
                miscRegs[MISCREG_CPSR] = cpsr;
                DPRINTF(MiscRegs,
                        "Updating ITSTATE -> %#x in CPSR -> %#x.\n",
                        (uint8_t)itstate, (uint32_t)cpsr);
            }
            break;
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
                DPRINTF(MiscRegs, "Writing SCTLR: %#x\n", newVal);
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
          case MISCREG_ACTLR:
            warn("Not doing anything for write of miscreg ACTLR\n");
            break;
          case MISCREG_PMCR:
          case MISCREG_PMCCNTR:
          case MISCREG_PMSELR:
            warn("Not doing anything for write to miscreg %s\n",
                    miscRegName[misc_reg]);
            break;
          case MISCREG_V2PCWPR:
          case MISCREG_V2PCWPW:
          case MISCREG_V2PCWUR:
          case MISCREG_V2PCWUW:
          case MISCREG_V2POWPR:
          case MISCREG_V2POWPW:
          case MISCREG_V2POWUR:
          case MISCREG_V2POWUW:
            {
              RequestPtr req = new Request;
              unsigned flags;
              BaseTLB::Mode mode;
              Fault fault;
              switch(misc_reg) {
                  case MISCREG_V2PCWPR:
                      flags = TLB::MustBeOne;
                      mode = BaseTLB::Read;
                      break;
                  case MISCREG_V2PCWPW:
                      flags = TLB::MustBeOne;
                      mode = BaseTLB::Write;
                      break;
                  case MISCREG_V2PCWUR:
                      flags = TLB::MustBeOne | TLB::UserMode;
                      mode = BaseTLB::Read;
                      break;
                  case MISCREG_V2PCWUW:
                      flags = TLB::MustBeOne | TLB::UserMode;
                      mode = BaseTLB::Write;
                      break;
                  default:
                      panic("Security Extensions not implemented!");
              }
              req->setVirt(0, val, 1, flags, tc->readPC());
              fault = tc->getDTBPtr()->translateAtomic(req, tc, mode);
              if (fault == NoFault) {
                  miscRegs[MISCREG_PAR] =
                      (req->getPaddr() & 0xfffff000) |
                      (tc->getDTBPtr()->getAttr() );
                  DPRINTF(MiscRegs,
                          "MISCREG: Translated addr 0x%08x: PAR: 0x%08x\n",
                          val, miscRegs[MISCREG_PAR]);
              }
              else {
                  // Set fault bit and FSR
                  FSR fsr = miscRegs[MISCREG_DFSR];
                  miscRegs[MISCREG_PAR] =
                      (fsr.ext << 6) |
                      (fsr.fsHigh << 5) |
                      (fsr.fsLow << 1) |
                      0x1; // F bit
              }
              return;
            }
        }
    }
    setMiscRegNoEffect(misc_reg, newVal);
}

}
