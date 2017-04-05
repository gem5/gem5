/*
 * Copyright (c) 2010, 2012-2016 ARM Limited
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

#include "arch/arm/isa_device.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/types.hh"
#include "arch/generic/traits.hh"
#include "debug/Checkpoint.hh"
#include "enums/VecRegRenameMode.hh"
#include "sim/sim_object.hh"
#include "enums/DecoderFlavour.hh"

struct ArmISAParams;
struct DummyArmISADeviceParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace ArmISA
{
    class ISA : public SimObject
    {
      protected:
        // Parent system
        ArmSystem *system;

        // Micro Architecture
        const Enums::DecoderFlavour _decoderFlavour;
        const Enums::VecRegRenameMode _vecRegRenameMode;

        /** Dummy device for to handle non-existing ISA devices */
        DummyISADevice dummyDevice;

        // PMU belonging to this ISA
        BaseISADevice *pmu;

        // Generic timer interface belonging to this ISA
        std::unique_ptr<BaseISADevice> timer;

        // Cached copies of system-level properties
        bool highestELIs64;
        bool haveSecurity;
        bool haveLPAE;
        bool haveVirtualization;
        bool haveLargeAsid64;
        uint8_t physAddrRange64;

        /** Register translation entry used in lookUpMiscReg */
        struct MiscRegLUTEntry {
            uint32_t lower;
            uint32_t upper;
        };

        struct MiscRegInitializerEntry {
            uint32_t index;
            struct MiscRegLUTEntry entry;
        };

        /** Register table noting all translations */
        static const struct MiscRegInitializerEntry MiscRegSwitch[];

        /** Translation table accessible via the value of the register */
        std::vector<struct MiscRegLUTEntry> lookUpMiscReg;

        MiscReg miscRegs[NumMiscRegs];
        const IntRegIndex *intRegMap;

        void
        updateRegMap(CPSR cpsr)
        {
            if (cpsr.width == 0) {
                intRegMap = IntReg64Map;
            } else {
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
                  case MODE_HYP:
                    intRegMap = IntRegHypMap;
                    break;
                  case MODE_UNDEFINED:
                    intRegMap = IntRegUndMap;
                    break;
                  default:
                    panic("Unrecognized mode setting in CPSR.\n");
                }
            }
        }

        BaseISADevice &getGenericTimer(ThreadContext *tc);


      private:
        inline void assert32(ThreadContext *tc) {
            CPSR cpsr M5_VAR_USED = readMiscReg(MISCREG_CPSR, tc);
            assert(cpsr.width);
        }

        inline void assert64(ThreadContext *tc) {
            CPSR cpsr M5_VAR_USED = readMiscReg(MISCREG_CPSR, tc);
            assert(!cpsr.width);
        }

        void tlbiVA(ThreadContext *tc, MiscReg newVal, uint16_t asid,
                    bool secure_lookup, uint8_t target_el);

        void tlbiALL(ThreadContext *tc, bool secure_lookup, uint8_t target_el);

        void tlbiALLN(ThreadContext *tc, bool hyp, uint8_t target_el);

        void tlbiMVA(ThreadContext *tc, MiscReg newVal, bool secure_lookup,
                     bool hyp, uint8_t target_el);

      public:
        void clear();
        void clear64(const ArmISAParams *p);

        MiscReg readMiscRegNoEffect(int misc_reg) const;
        MiscReg readMiscReg(int misc_reg, ThreadContext *tc);
        void setMiscRegNoEffect(int misc_reg, const MiscReg &val);
        void setMiscReg(int misc_reg, const MiscReg &val, ThreadContext *tc);

        RegId
        flattenRegId(const RegId& regId) const
        {
            switch (regId.classValue()) {
              case IntRegClass:
                return RegId(IntRegClass, flattenIntIndex(regId.index()));
              case FloatRegClass:
                return RegId(FloatRegClass, flattenFloatIndex(regId.index()));
              case VecRegClass:
                return RegId(VecRegClass, flattenVecIndex(regId.index()));
              case VecElemClass:
                return RegId(VecElemClass, flattenVecElemIndex(regId.index()));
              case CCRegClass:
                return RegId(CCRegClass, flattenCCIndex(regId.index()));
              case MiscRegClass:
                return RegId(MiscRegClass, flattenMiscIndex(regId.index()));
            }
            return RegId();
        }

        int
        flattenIntIndex(int reg) const
        {
            assert(reg >= 0);
            if (reg < NUM_ARCH_INTREGS) {
                return intRegMap[reg];
            } else if (reg < NUM_INTREGS) {
                return reg;
            } else if (reg == INTREG_SPX) {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                ExceptionLevel el = opModeToEL(
                    (OperatingMode) (uint8_t) cpsr.mode);
                if (!cpsr.sp && el != EL0)
                    return INTREG_SP0;
                switch (el) {
                  case EL3:
                    return INTREG_SP3;
                  case EL2:
                    return INTREG_SP2;
                  case EL1:
                    return INTREG_SP1;
                  case EL0:
                    return INTREG_SP0;
                  default:
                    panic("Invalid exception level");
                    break;
                }
            } else {
                return flattenIntRegModeIndex(reg);
            }
        }

        int
        flattenFloatIndex(int reg) const
        {
            assert(reg >= 0);
            return reg;
        }

        int
        flattenVecIndex(int reg) const
        {
            assert(reg >= 0);
            return reg;
        }

        int
        flattenVecElemIndex(int reg) const
        {
            assert(reg >= 0);
            return reg;
        }

        int
        flattenCCIndex(int reg) const
        {
            assert(reg >= 0);
            return reg;
        }

        int
        flattenMiscIndex(int reg) const
        {
            assert(reg >= 0);
            int flat_idx = reg;

            if (reg == MISCREG_SPSR) {
                CPSR cpsr = miscRegs[MISCREG_CPSR];
                switch (cpsr.mode) {
                  case MODE_EL0T:
                    warn("User mode does not have SPSR\n");
                    flat_idx = MISCREG_SPSR;
                    break;
                  case MODE_EL1T:
                  case MODE_EL1H:
                    flat_idx = MISCREG_SPSR_EL1;
                    break;
                  case MODE_EL2T:
                  case MODE_EL2H:
                    flat_idx = MISCREG_SPSR_EL2;
                    break;
                  case MODE_EL3T:
                  case MODE_EL3H:
                    flat_idx = MISCREG_SPSR_EL3;
                    break;
                  case MODE_USER:
                    warn("User mode does not have SPSR\n");
                    flat_idx = MISCREG_SPSR;
                    break;
                  case MODE_FIQ:
                    flat_idx = MISCREG_SPSR_FIQ;
                    break;
                  case MODE_IRQ:
                    flat_idx = MISCREG_SPSR_IRQ;
                    break;
                  case MODE_SVC:
                    flat_idx = MISCREG_SPSR_SVC;
                    break;
                  case MODE_MON:
                    flat_idx = MISCREG_SPSR_MON;
                    break;
                  case MODE_ABORT:
                    flat_idx = MISCREG_SPSR_ABT;
                    break;
                  case MODE_HYP:
                    flat_idx = MISCREG_SPSR_HYP;
                    break;
                  case MODE_UNDEFINED:
                    flat_idx = MISCREG_SPSR_UND;
                    break;
                  default:
                    warn("Trying to access SPSR in an invalid mode: %d\n",
                         cpsr.mode);
                    flat_idx = MISCREG_SPSR;
                    break;
                }
            } else if (miscRegInfo[reg][MISCREG_MUTEX]) {
                // Mutually exclusive CP15 register
                switch (reg) {
                  case MISCREG_PRRR_MAIR0:
                  case MISCREG_PRRR_MAIR0_NS:
                  case MISCREG_PRRR_MAIR0_S:
                    {
                        TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                        // If the muxed reg has been flattened, work out the
                        // offset and apply it to the unmuxed reg
                        int idxOffset = reg - MISCREG_PRRR_MAIR0;
                        if (ttbcr.eae)
                            flat_idx = flattenMiscIndex(MISCREG_MAIR0 +
                                                        idxOffset);
                        else
                            flat_idx = flattenMiscIndex(MISCREG_PRRR +
                                                        idxOffset);
                    }
                    break;
                  case MISCREG_NMRR_MAIR1:
                  case MISCREG_NMRR_MAIR1_NS:
                  case MISCREG_NMRR_MAIR1_S:
                    {
                        TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                        // If the muxed reg has been flattened, work out the
                        // offset and apply it to the unmuxed reg
                        int idxOffset = reg - MISCREG_NMRR_MAIR1;
                        if (ttbcr.eae)
                            flat_idx = flattenMiscIndex(MISCREG_MAIR1 +
                                                        idxOffset);
                        else
                            flat_idx = flattenMiscIndex(MISCREG_NMRR +
                                                        idxOffset);
                    }
                    break;
                  case MISCREG_PMXEVTYPER_PMCCFILTR:
                    {
                        PMSELR pmselr = miscRegs[MISCREG_PMSELR];
                        if (pmselr.sel == 31)
                            flat_idx = flattenMiscIndex(MISCREG_PMCCFILTR);
                        else
                            flat_idx = flattenMiscIndex(MISCREG_PMXEVTYPER);
                    }
                    break;
                  default:
                    panic("Unrecognized misc. register.\n");
                    break;
                }
            } else {
                if (miscRegInfo[reg][MISCREG_BANKED]) {
                    bool secureReg = haveSecurity && !highestELIs64 &&
                                     inSecureState(miscRegs[MISCREG_SCR],
                                                   miscRegs[MISCREG_CPSR]);
                    flat_idx += secureReg ? 2 : 1;
                }
            }
            return flat_idx;
        }

        std::pair<int,int> getMiscIndices(int misc_reg) const
        {
            // Note: indexes of AArch64 registers are left unchanged
            int flat_idx = flattenMiscIndex(misc_reg);

            if (lookUpMiscReg[flat_idx].lower == 0) {
                return std::make_pair(flat_idx, 0);
            }

            // do additional S/NS flattenings if mapped to NS while in S
            bool S = haveSecurity && !highestELIs64 &&
                     inSecureState(miscRegs[MISCREG_SCR],
                                   miscRegs[MISCREG_CPSR]);
            int lower = lookUpMiscReg[flat_idx].lower;
            int upper = lookUpMiscReg[flat_idx].upper;
            // upper == 0, which is CPSR, is not MISCREG_BANKED_CHILD (no-op)
            lower += S && miscRegInfo[lower][MISCREG_BANKED_CHILD];
            upper += S && miscRegInfo[upper][MISCREG_BANKED_CHILD];
            return std::make_pair(lower, upper);
        }

        void serialize(CheckpointOut &cp) const
        {
            DPRINTF(Checkpoint, "Serializing Arm Misc Registers\n");
            SERIALIZE_ARRAY(miscRegs, NumMiscRegs);

            SERIALIZE_SCALAR(highestELIs64);
            SERIALIZE_SCALAR(haveSecurity);
            SERIALIZE_SCALAR(haveLPAE);
            SERIALIZE_SCALAR(haveVirtualization);
            SERIALIZE_SCALAR(haveLargeAsid64);
            SERIALIZE_SCALAR(physAddrRange64);
        }
        void unserialize(CheckpointIn &cp)
        {
            DPRINTF(Checkpoint, "Unserializing Arm Misc Registers\n");
            UNSERIALIZE_ARRAY(miscRegs, NumMiscRegs);
            CPSR tmp_cpsr = miscRegs[MISCREG_CPSR];
            updateRegMap(tmp_cpsr);

            UNSERIALIZE_SCALAR(highestELIs64);
            UNSERIALIZE_SCALAR(haveSecurity);
            UNSERIALIZE_SCALAR(haveLPAE);
            UNSERIALIZE_SCALAR(haveVirtualization);
            UNSERIALIZE_SCALAR(haveLargeAsid64);
            UNSERIALIZE_SCALAR(physAddrRange64);
        }

        void startup(ThreadContext *tc) {}

        Enums::DecoderFlavour decoderFlavour() const { return _decoderFlavour; }

        Enums::VecRegRenameMode
        vecRegRenameMode() const
        {
            return _vecRegRenameMode;
        }

        /// Explicitly import the otherwise hidden startup
        using SimObject::startup;

        typedef ArmISAParams Params;

        const Params *params() const;

        ISA(Params *p);
    };
}

template<>
struct initRenameMode<ArmISA::ISA>
{
    static Enums::VecRegRenameMode mode(const ArmISA::ISA* isa)
    {
        return isa->vecRegRenameMode();
    }
    static bool equals(const ArmISA::ISA* isa1, const ArmISA::ISA* isa2)
    {
        return mode(isa1) == mode(isa2);
    }
};

#endif
