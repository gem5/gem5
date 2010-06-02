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
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 */

#ifndef __ARCH_ARM_ISA_HH__
#define __ARCH_MRM_ISA_HH__

#include "arch/arm/registers.hh"
#include "arch/arm/types.hh"

class ThreadContext;
class Checkpoint;
class EventManager;

namespace ArmISA
{
    class ISA
    {
      protected:
        MiscReg miscRegs[NumMiscRegs];
        const IntRegIndex *intRegMap;

        void
        updateRegMap(CPSR cpsr)
        {
            switch (cpsr.mode) {
              case MODE_USER:
              case MODE_SYSTEM:
                intRegMap = IntRegUsrMap;
                break;
              case MODE_FIQ:
                intRegMap = IntRegFiqMap;
                break;
              case MODE_IRQ:
                intRegMap = IntRegIrqMap;
                break;
              case MODE_SVC:
                intRegMap = IntRegSvcMap;
                break;
              case MODE_MON:
                intRegMap = IntRegMonMap;
                break;
              case MODE_ABORT:
                intRegMap = IntRegAbtMap;
                break;
              case MODE_UNDEFINED:
                intRegMap = IntRegUndMap;
                break;
              default:
                panic("Unrecognized mode setting in CPSR.\n");
            }
        }

      public:
        void clear()
        {
            memset(miscRegs, 0, sizeof(miscRegs));
            CPSR cpsr = 0;
            cpsr.mode = MODE_USER;
            miscRegs[MISCREG_CPSR] = cpsr;
            updateRegMap(cpsr);

            SCTLR sctlr = 0;
            sctlr.nmfi = 1;
            sctlr.rao1 = 1;
            sctlr.rao2 = 1;
            sctlr.rao3 = 1;
            sctlr.rao4 = 1;
            miscRegs[MISCREG_SCTLR] = sctlr;

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

            //XXX We need to initialize the rest of the state.
        }

        MiscReg
        readMiscRegNoEffect(int misc_reg)
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
        readMiscReg(int misc_reg, ThreadContext *tc)
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
            }
            return readMiscRegNoEffect(misc_reg);
        }

        void
        setMiscRegNoEffect(int misc_reg, const MiscReg &val)
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
        setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc)
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
              case MISCREG_TLBTR:
              case MISCREG_MVFR0:
              case MISCREG_MVFR1:
              case MISCREG_MPIDR:
                return;
            }
            return setMiscRegNoEffect(misc_reg, newVal);
        }

        int
        flattenIntIndex(int reg)
        {
            assert(reg >= 0);
            if (reg < NUM_ARCH_INTREGS) {
                return intRegMap[reg];
            } else if (reg < NUM_INTREGS) {
                return reg;
            } else {
                int mode = reg / intRegsPerMode;
                reg = reg % intRegsPerMode;
                switch (mode) {
                  case MODE_USER:
                  case MODE_SYSTEM:
                    return INTREG_USR(reg);
                  case MODE_FIQ:
                    return INTREG_FIQ(reg);
                  case MODE_IRQ:
                    return INTREG_IRQ(reg);
                  case MODE_SVC:
                    return INTREG_SVC(reg);
                  case MODE_MON:
                    return INTREG_MON(reg);
                  case MODE_ABORT:
                    return INTREG_ABT(reg);
                  case MODE_UNDEFINED:
                    return INTREG_UND(reg);
                  default:
                    panic("Flattening into an unknown mode.\n");
                }
            }
        }

        int
        flattenFloatIndex(int reg)
        {
            return reg;
        }

        void serialize(EventManager *em, std::ostream &os)
        {}
        void unserialize(EventManager *em, Checkpoint *cp,
                const std::string &section)
        {}

        ISA()
        {
            clear();
        }
    };
}

#endif
