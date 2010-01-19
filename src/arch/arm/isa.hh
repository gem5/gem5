/*
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
            if (misc_reg == MISCREG_CPSR) {
                updateRegMap(val);
            }
            return setMiscRegNoEffect(misc_reg, val);
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
                reg -= NUM_INTREGS;
                assert(reg < NUM_ARCH_INTREGS);
                return reg;
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
