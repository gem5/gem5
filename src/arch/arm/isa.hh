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
#include "debug/Checkpoint.hh"
#include "sim/sim_object.hh"

struct ArmISAParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace ArmISA
{
    class ISA : public SimObject
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
        void clear();

        MiscReg readMiscRegNoEffect(int misc_reg) const;
        MiscReg readMiscReg(int misc_reg, ThreadContext *tc);
        void setMiscRegNoEffect(int misc_reg, const MiscReg &val);
        void setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc);

        int
        flattenIntIndex(int reg) const
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
        flattenFloatIndex(int reg) const
        {
            return reg;
        }

        // dummy
        int
        flattenCCIndex(int reg) const
        {
            return reg;
        }

        int
        flattenMiscIndex(int reg) const
        {
            if (reg == MISCREG_SPSR) {
                int spsr_idx = NUM_MISCREGS;
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                switch (cpsr.mode) {
                  case MODE_USER:
                    warn("User mode does not have SPSR\n");
                    spsr_idx = MISCREG_SPSR;
                    break;
                  case MODE_FIQ:
                    spsr_idx = MISCREG_SPSR_FIQ;
                    break;
                  case MODE_IRQ:
                    spsr_idx = MISCREG_SPSR_IRQ;
                    break;
                  case MODE_SVC:
                    spsr_idx = MISCREG_SPSR_SVC;
                    break;
                  case MODE_MON:
                    spsr_idx = MISCREG_SPSR_MON;
                    break;
                  case MODE_ABORT:
                    spsr_idx = MISCREG_SPSR_ABT;
                    break;
                  case MODE_UNDEFINED:
                    spsr_idx = MISCREG_SPSR_UND;
                    break;
                  default:
                    warn("Trying to access SPSR in an invalid mode: %d\n",
                         cpsr.mode);
                    spsr_idx = MISCREG_SPSR;
                    break;
                }
                return spsr_idx;
            }
            return reg;
        }

        void serialize(std::ostream &os)
        {
            DPRINTF(Checkpoint, "Serializing Arm Misc Registers\n");
            SERIALIZE_ARRAY(miscRegs, NumMiscRegs);
        }
        void unserialize(Checkpoint *cp, const std::string &section)
        {
            DPRINTF(Checkpoint, "Unserializing Arm Misc Registers\n");
            UNSERIALIZE_ARRAY(miscRegs, NumMiscRegs);
            CPSR tmp_cpsr = miscRegs[MISCREG_CPSR];
            updateRegMap(tmp_cpsr);
        }

        void startup(ThreadContext *tc) {}

        /// Explicitly import the otherwise hidden startup
        using SimObject::startup;

        typedef ArmISAParams Params;

        const Params *params() const;

        ISA(Params *p);
    };
}

#endif
