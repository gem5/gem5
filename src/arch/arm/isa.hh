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
#define __ARCH_ARM_ISA_HH__

#include "arch/arm/registers.hh"
#include "arch/arm/tlb.hh"
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
            SCTLR sctlr_rst = miscRegs[MISCREG_SCTLR_RST];

            memset(miscRegs, 0, sizeof(miscRegs));
            CPSR cpsr = 0;
            cpsr.mode = MODE_USER;
            miscRegs[MISCREG_CPSR] = cpsr;
            updateRegMap(cpsr);

            SCTLR sctlr = 0;
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

            //XXX We need to initialize the rest of the state.
        }

        MiscReg readMiscRegNoEffect(int misc_reg);

        MiscReg readMiscReg(int misc_reg, ThreadContext *tc);

        void setMiscRegNoEffect(int misc_reg, const MiscReg &val);

        void setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc);

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
            SCTLR sctlr;
            sctlr = 0;
            miscRegs[MISCREG_SCTLR_RST] = sctlr;

            clear();
        }
    };
}

#endif
