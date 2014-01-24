/*
 * Copyright (c) 2010-2013 ARM Limited
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
#include "arch/arm/system.hh"
#include "cpu/checker/cpu.hh"
#include "debug/Arm.hh"
#include "debug/MiscRegs.hh"
#include "params/ArmISA.hh"
#include "sim/faults.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

namespace ArmISA
{

ISA::ISA(Params *p)
    : SimObject(p)
{
    SCTLR sctlr;
    sctlr = 0;
    miscRegs[MISCREG_SCTLR_RST] = sctlr;
    clear();
}

const ArmISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void
ISA::clear()
{
    const Params *p(params());

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

    /* Start with an event in the mailbox */
    miscRegs[MISCREG_SEV_MAILBOX] = 1;

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

    miscRegs[MISCREG_CPACR] = 0;

    // Initialize configurable default values
    miscRegs[MISCREG_MIDR] = p->midr;

    miscRegs[MISCREG_ID_PFR0] = p->id_pfr0;
    miscRegs[MISCREG_ID_PFR1] = p->id_pfr1;

    miscRegs[MISCREG_ID_MMFR0] = p->id_mmfr0;
    miscRegs[MISCREG_ID_MMFR1] = p->id_mmfr1;
    miscRegs[MISCREG_ID_MMFR2] = p->id_mmfr2;
    miscRegs[MISCREG_ID_MMFR3] = p->id_mmfr3;

    miscRegs[MISCREG_ID_ISAR0] = p->id_isar0;
    miscRegs[MISCREG_ID_ISAR1] = p->id_isar1;
    miscRegs[MISCREG_ID_ISAR2] = p->id_isar2;
    miscRegs[MISCREG_ID_ISAR3] = p->id_isar3;
    miscRegs[MISCREG_ID_ISAR4] = p->id_isar4;
    miscRegs[MISCREG_ID_ISAR5] = p->id_isar5;


    miscRegs[MISCREG_FPSID] = p->fpsid;


    //XXX We need to initialize the rest of the state.
}

MiscReg
ISA::readMiscRegNoEffect(int misc_reg) const
{
    assert(misc_reg < NumMiscRegs);

    int flat_idx;
    if (misc_reg == MISCREG_SPSR)
        flat_idx = flattenMiscIndex(misc_reg);
    else
        flat_idx = misc_reg;
    MiscReg val = miscRegs[flat_idx];

    DPRINTF(MiscRegs, "Reading From misc reg %d (%d) : %#x\n",
            misc_reg, flat_idx, val);
    return val;
}


MiscReg
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    ArmSystem *arm_sys;

    if (misc_reg == MISCREG_CPSR) {
        CPSR cpsr = miscRegs[misc_reg];
        PCState pc = tc->pcState();
        cpsr.j = pc.jazelle() ? 1 : 0;
        cpsr.t = pc.thumb() ? 1 : 0;
        return cpsr;
    }
    if (misc_reg >= MISCREG_CP15_UNIMP_START)
        panic("Unimplemented CP15 register %s read.\n",
              miscRegName[misc_reg]);

    switch (misc_reg) {
      case MISCREG_MPIDR:
        arm_sys = dynamic_cast<ArmSystem*>(tc->getSystemPtr());
        assert(arm_sys);

        if (arm_sys->multiProc) {
            return 0x80000000 | // multiprocessor extensions available
                   tc->cpuId();
        } else {
            return 0x80000000 |  // multiprocessor extensions available
                   0x40000000 |  // in up system
                   tc->cpuId();
        }
        break;
      case MISCREG_CLIDR:
        warn_once("The clidr register always reports 0 caches.\n");
        warn_once("clidr LoUIS field of 0b001 to match current "
                  "ARM implementations.\n");
        return 0x00200000;
      case MISCREG_CCSIDR:
        warn_once("The ccsidr register isn't implemented and "
                "always reads as 0.\n");
        break;
      case MISCREG_CTR:
        {
            //all caches have the same line size in gem5
            //4 byte words in ARM
            unsigned lineSizeWords =
                tc->getSystemPtr()->cacheLineSize() / 4;
            unsigned log2LineSizeWords = 0;

            while (lineSizeWords >>= 1) {
                ++log2LineSizeWords;
            }

            CTR ctr = 0;
            //log2 of minimun i-cache line size (words)
            ctr.iCacheLineSize = log2LineSizeWords;
            //b11 - gem5 uses pipt
            ctr.l1IndexPolicy = 0x3;
            //log2 of minimum d-cache line size (words)
            ctr.dCacheLineSize = log2LineSizeWords;
            //log2 of max reservation size (words)
            ctr.erg = log2LineSizeWords;
            //log2 of max writeback size (words)
            ctr.cwg = log2LineSizeWords;
            //b100 - gem5 format is ARMv7
            ctr.format = 0x4;

            return ctr;
        }
      case MISCREG_ACTLR:
        warn("Not doing anything for miscreg ACTLR\n");
        break;
      case MISCREG_PMCR:
      case MISCREG_PMCCNTR:
      case MISCREG_PMSELR:
        warn("Not doing anything for read to miscreg %s\n",
                miscRegName[misc_reg]);
        break;
      case MISCREG_CPSR_Q:
        panic("shouldn't be reading this register seperately\n");
      case MISCREG_FPSCR_QC:
        return readMiscRegNoEffect(MISCREG_FPSCR) & ~FpscrQcMask;
      case MISCREG_FPSCR_EXC:
        return readMiscRegNoEffect(MISCREG_FPSCR) & ~FpscrExcMask;
      case MISCREG_L2CTLR:
        {
            // mostly unimplemented, just set NumCPUs field from sim and return
            L2CTLR l2ctlr = 0;
            // b00:1CPU to b11:4CPUs
            l2ctlr.numCPUs = tc->getSystemPtr()->numContexts() - 1;
            return l2ctlr;
        }
      case MISCREG_DBGDIDR:
        /* For now just implement the version number.
         * Return 0 as we don't support debug architecture yet.
         */
        return 0;
      case MISCREG_DBGDSCR_INT:
        return 0;
    }
    return readMiscRegNoEffect(misc_reg);
}

void
ISA::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    assert(misc_reg < NumMiscRegs);

    int flat_idx;
    if (misc_reg == MISCREG_SPSR)
        flat_idx = flattenMiscIndex(misc_reg);
    else
        flat_idx = misc_reg;
    miscRegs[flat_idx] = val;

    DPRINTF(MiscRegs, "Writing to misc reg %d (%d) : %#x\n", misc_reg,
            flat_idx, val);
}

void
ISA::setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
{

    MiscReg newVal = val;
    int x;
    System *sys;
    ThreadContext *oc;

    if (misc_reg == MISCREG_CPSR) {
        updateRegMap(val);


        CPSR old_cpsr = miscRegs[MISCREG_CPSR];
        int old_mode = old_cpsr.mode;
        CPSR cpsr = val;
        if (old_mode != cpsr.mode) {
            tc->getITBPtr()->invalidateMiscReg();
            tc->getDTBPtr()->invalidateMiscReg();
        }

        DPRINTF(Arm, "Updating CPSR from %#x to %#x f:%d i:%d a:%d mode:%#x\n",
                miscRegs[misc_reg], cpsr, cpsr.f, cpsr.i, cpsr.a, cpsr.mode);
        PCState pc = tc->pcState();
        pc.nextThumb(cpsr.t);
        pc.nextJazelle(cpsr.j);

        // Follow slightly different semantics if a CheckerCPU object
        // is connected
        CheckerCPU *checker = tc->getCheckerCpuPtr();
        if (checker) {
            tc->pcStateNoRecord(pc);
        } else {
            tc->pcState(pc);
        }
    } else if (misc_reg >= MISCREG_CP15_UNIMP_START &&
        misc_reg < MISCREG_CP15_END) {
        panic("Unimplemented CP15 register %s wrote with %#x.\n",
              miscRegName[misc_reg], val);
    } else {
        switch (misc_reg) {
          case MISCREG_CPACR:
            {

                const uint32_t ones = (uint32_t)(-1);
                CPACR cpacrMask = 0;
                // Only cp10, cp11, and ase are implemented, nothing else should
                // be writable
                cpacrMask.cp10 = ones;
                cpacrMask.cp11 = ones;
                cpacrMask.asedis = ones;
                newVal &= cpacrMask;
                DPRINTF(MiscRegs, "Writing misc reg %s: %#x\n",
                        miscRegName[misc_reg], newVal);
            }
            break;
          case MISCREG_CSSELR:
            warn_once("The csselr register isn't implemented.\n");
            return;
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
                tc->getDecoderPtr()->setContext(newVal);
            }
            break;
          case MISCREG_CPSR_Q:
            {
                assert(!(newVal & ~CpsrMaskQ));
                newVal = miscRegs[MISCREG_CPSR] | newVal;
                misc_reg = MISCREG_CPSR;
            }
            break;
          case MISCREG_FPSCR_QC:
            {
                newVal = miscRegs[MISCREG_FPSCR] | (newVal & FpscrQcMask);
                misc_reg = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPSCR_EXC:
            {
                newVal = miscRegs[MISCREG_FPSCR] | (newVal & FpscrExcMask);
                misc_reg = MISCREG_FPSCR;
            }
            break;
          case MISCREG_FPEXC:
            {
                // vfpv3 architecture, section B.6.1 of DDI04068
                // bit 29 - valid only if fpexc[31] is 0
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
                tc->getITBPtr()->invalidateMiscReg();
                tc->getDTBPtr()->invalidateMiscReg();

                // Check if all CPUs are booted with caches enabled
                // so we can stop enforcing coherency of some kernel
                // structures manually.
                sys = tc->getSystemPtr();
                for (x = 0; x < sys->numContexts(); x++) {
                    oc = sys->getThreadContext(x);
                    SCTLR other_sctlr = oc->readMiscRegNoEffect(MISCREG_SCTLR);
                    if (!other_sctlr.c && oc->status() != ThreadContext::Halted)
                        return;
                }

                for (x = 0; x < sys->numContexts(); x++) {
                    oc = sys->getThreadContext(x);
                    oc->getDTBPtr()->allCpusCaching();
                    oc->getITBPtr()->allCpusCaching();

                    // If CheckerCPU is connected, need to notify it.
                    CheckerCPU *checker = oc->getCheckerCpuPtr();
                    if (checker) {
                        checker->getDTBPtr()->allCpusCaching();
                        checker->getITBPtr()->allCpusCaching();
                    }
                }
                return;
            }

          case MISCREG_MIDR:
          case MISCREG_ID_PFR0:
          case MISCREG_ID_PFR1:
          case MISCREG_ID_MMFR0:
          case MISCREG_ID_MMFR1:
          case MISCREG_ID_MMFR2:
          case MISCREG_ID_MMFR3:
          case MISCREG_ID_ISAR0:
          case MISCREG_ID_ISAR1:
          case MISCREG_ID_ISAR2:
          case MISCREG_ID_ISAR3:
          case MISCREG_ID_ISAR4:
          case MISCREG_ID_ISAR5:

          case MISCREG_MPIDR:
          case MISCREG_FPSID:
          case MISCREG_TLBTR:
          case MISCREG_MVFR0:
          case MISCREG_MVFR1:
            // ID registers are constants.
            return;

          case MISCREG_TLBIALLIS:
          case MISCREG_TLBIALL:
            sys = tc->getSystemPtr();
            for (x = 0; x < sys->numContexts(); x++) {
                oc = sys->getThreadContext(x);
                assert(oc->getITBPtr() && oc->getDTBPtr());
                oc->getITBPtr()->flushAll();
                oc->getDTBPtr()->flushAll();

                // If CheckerCPU is connected, need to notify it of a flush
                CheckerCPU *checker = oc->getCheckerCpuPtr();
                if (checker) {
                    checker->getITBPtr()->flushAll();
                    checker->getDTBPtr()->flushAll();
                }
            }
            return;
          case MISCREG_ITLBIALL:
            tc->getITBPtr()->flushAll();
            return;
          case MISCREG_DTLBIALL:
            tc->getDTBPtr()->flushAll();
            return;
          case MISCREG_TLBIMVAIS:
          case MISCREG_TLBIMVA:
            sys = tc->getSystemPtr();
            for (x = 0; x < sys->numContexts(); x++) {
                oc = sys->getThreadContext(x);
                assert(oc->getITBPtr() && oc->getDTBPtr());
                oc->getITBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                        bits(newVal, 7,0));
                oc->getDTBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                        bits(newVal, 7,0));

                CheckerCPU *checker = oc->getCheckerCpuPtr();
                if (checker) {
                    checker->getITBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                            bits(newVal, 7,0));
                    checker->getDTBPtr()->flushMvaAsid(mbits(newVal, 31, 12),
                            bits(newVal, 7,0));
                }
            }
            return;
          case MISCREG_TLBIASIDIS:
          case MISCREG_TLBIASID:
            sys = tc->getSystemPtr();
            for (x = 0; x < sys->numContexts(); x++) {
                oc = sys->getThreadContext(x);
                assert(oc->getITBPtr() && oc->getDTBPtr());
                oc->getITBPtr()->flushAsid(bits(newVal, 7,0));
                oc->getDTBPtr()->flushAsid(bits(newVal, 7,0));
                CheckerCPU *checker = oc->getCheckerCpuPtr();
                if (checker) {
                    checker->getITBPtr()->flushAsid(bits(newVal, 7,0));
                    checker->getDTBPtr()->flushAsid(bits(newVal, 7,0));
                }
            }
            return;
          case MISCREG_TLBIMVAAIS:
          case MISCREG_TLBIMVAA:
            sys = tc->getSystemPtr();
            for (x = 0; x < sys->numContexts(); x++) {
                oc = sys->getThreadContext(x);
                assert(oc->getITBPtr() && oc->getDTBPtr());
                oc->getITBPtr()->flushMva(mbits(newVal, 31,12));
                oc->getDTBPtr()->flushMva(mbits(newVal, 31,12));

                CheckerCPU *checker = oc->getCheckerCpuPtr();
                if (checker) {
                    checker->getITBPtr()->flushMva(mbits(newVal, 31,12));
                    checker->getDTBPtr()->flushMva(mbits(newVal, 31,12));
                }
            }
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
            {
              // Performance counters not implemented.  Instead, interpret
              //   a reset command to this register to reset the simulator
              //   statistics.
              // PMCR_E | PMCR_P | PMCR_C
              const int ResetAndEnableCounters = 0x7;
              if (newVal == ResetAndEnableCounters) {
                  inform("Resetting all simobject stats\n");
                  Stats::schedStatEvent(false, true);
                  break;
              }
            }
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
              warn("Translating via MISCREG in atomic mode! Fix Me!\n");
              req->setVirt(0, val, 1, flags, tc->pcState().pc(),
                      Request::funcMasterId);
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
          case MISCREG_CONTEXTIDR:
          case MISCREG_PRRR:
          case MISCREG_NMRR:
          case MISCREG_DACR:
            tc->getITBPtr()->invalidateMiscReg();
            tc->getDTBPtr()->invalidateMiscReg();
            break;
          case MISCREG_L2CTLR:
            warn("miscreg L2CTLR (%s) written with %#x. ignored...\n",
                 miscRegName[misc_reg], uint32_t(val));
        }
    }
    setMiscRegNoEffect(misc_reg, newVal);
}

}

ArmISA::ISA *
ArmISAParams::create()
{
    return new ArmISA::ISA(this);
}
