/*
 * Copyright (c) 2010, 2012-2020 ARM Limited
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
 */

#ifndef __ARCH_ARM_ISA_HH__
#define __ARCH_ARM_ISA_HH__

#include "arch/arm/isa_device.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/registers.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/types.hh"
#include "arch/generic/isa.hh"
#include "arch/generic/traits.hh"
#include "debug/Checkpoint.hh"
#include "enums/DecoderFlavor.hh"
#include "enums/VecRegRenameMode.hh"
#include "sim/sim_object.hh"

struct ArmISAParams;
struct DummyArmISADeviceParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace ArmISA
{
    class ISA : public BaseISA
    {
      protected:
        // Parent system
        ArmSystem *system;

        // Micro Architecture
        const Enums::DecoderFlavor _decoderFlavor;
        const Enums::VecRegRenameMode _vecRegRenameMode;

        /** Dummy device for to handle non-existing ISA devices */
        DummyISADevice dummyDevice;

        // PMU belonging to this ISA
        BaseISADevice *pmu;

        // Generic timer interface belonging to this ISA
        std::unique_ptr<BaseISADevice> timer;

        // GICv3 CPU interface belonging to this ISA
        std::unique_ptr<BaseISADevice> gicv3CpuInterface;

        // Cached copies of system-level properties
        bool highestELIs64;
        bool haveSecurity;
        bool haveLPAE;
        bool haveVirtualization;
        bool haveCrypto;
        bool haveLargeAsid64;
        uint8_t physAddrRange;
        bool haveSVE;
        bool haveLSE;
        bool havePAN;

        /** SVE vector length in quadwords */
        unsigned sveVL;

        /**
         * If true, accesses to IMPLEMENTATION DEFINED registers are treated
         * as NOP hence not causing UNDEFINED INSTRUCTION.
         */
        bool impdefAsNop;

        bool afterStartup;

        /** MiscReg metadata **/
        struct MiscRegLUTEntry {
            uint32_t lower;  // Lower half mapped to this register
            uint32_t upper;  // Upper half mapped to this register
            uint64_t _reset; // value taken on reset (i.e. initialization)
            uint64_t _res0;  // reserved
            uint64_t _res1;  // reserved
            uint64_t _raz;   // read as zero (fixed at 0)
            uint64_t _rao;   // read as one (fixed at 1)
          public:
            MiscRegLUTEntry() :
              lower(0), upper(0),
              _reset(0), _res0(0), _res1(0), _raz(0), _rao(0) {}
            uint64_t reset() const { return _reset; }
            uint64_t res0()  const { return _res0; }
            uint64_t res1()  const { return _res1; }
            uint64_t raz()   const { return _raz; }
            uint64_t rao()   const { return _rao; }
            // raz/rao implies writes ignored
            uint64_t wi()    const { return _raz | _rao; }
        };

        /** Metadata table accessible via the value of the register */
        static std::vector<struct MiscRegLUTEntry> lookUpMiscReg;

        class MiscRegLUTEntryInitializer {
            struct MiscRegLUTEntry &entry;
            std::bitset<NUM_MISCREG_INFOS> &info;
            typedef const MiscRegLUTEntryInitializer& chain;
          public:
            chain mapsTo(uint32_t l, uint32_t u = 0) const {
                entry.lower = l;
                entry.upper = u;
                return *this;
            }
            chain res0(uint64_t mask) const {
                entry._res0 = mask;
                return *this;
            }
            chain res1(uint64_t mask) const {
                entry._res1 = mask;
                return *this;
            }
            chain raz(uint64_t mask) const {
                entry._raz  = mask;
                return *this;
            }
            chain rao(uint64_t mask) const {
                entry._rao  = mask;
                return *this;
            }
            chain implemented(bool v = true) const {
                info[MISCREG_IMPLEMENTED] = v;
                return *this;
            }
            chain unimplemented() const {
                return implemented(false);
            }
            chain unverifiable(bool v = true) const {
                info[MISCREG_UNVERIFIABLE] = v;
                return *this;
            }
            chain warnNotFail(bool v = true) const {
                info[MISCREG_WARN_NOT_FAIL] = v;
                return *this;
            }
            chain mutex(bool v = true) const {
                info[MISCREG_MUTEX] = v;
                return *this;
            }
            chain banked(bool v = true) const {
                info[MISCREG_BANKED] = v;
                return *this;
            }
            chain banked64(bool v = true) const {
                info[MISCREG_BANKED64] = v;
                return *this;
            }
            chain bankedChild(bool v = true) const {
                info[MISCREG_BANKED_CHILD] = v;
                return *this;
            }
            chain userNonSecureRead(bool v = true) const {
                info[MISCREG_USR_NS_RD] = v;
                return *this;
            }
            chain userNonSecureWrite(bool v = true) const {
                info[MISCREG_USR_NS_WR] = v;
                return *this;
            }
            chain userSecureRead(bool v = true) const {
                info[MISCREG_USR_S_RD] = v;
                return *this;
            }
            chain userSecureWrite(bool v = true) const {
                info[MISCREG_USR_S_WR] = v;
                return *this;
            }
            chain user(bool v = true) const {
                userNonSecureRead(v);
                userNonSecureWrite(v);
                userSecureRead(v);
                userSecureWrite(v);
                return *this;
            }
            chain privNonSecureRead(bool v = true) const {
                info[MISCREG_PRI_NS_RD] = v;
                return *this;
            }
            chain privNonSecureWrite(bool v = true) const {
                info[MISCREG_PRI_NS_WR] = v;
                return *this;
            }
            chain privNonSecure(bool v = true) const {
                privNonSecureRead(v);
                privNonSecureWrite(v);
                return *this;
            }
            chain privSecureRead(bool v = true) const {
                info[MISCREG_PRI_S_RD] = v;
                return *this;
            }
            chain privSecureWrite(bool v = true) const {
                info[MISCREG_PRI_S_WR] = v;
                return *this;
            }
            chain privSecure(bool v = true) const {
                privSecureRead(v);
                privSecureWrite(v);
                return *this;
            }
            chain priv(bool v = true) const {
                privSecure(v);
                privNonSecure(v);
                return *this;
            }
            chain privRead(bool v = true) const {
                privSecureRead(v);
                privNonSecureRead(v);
                return *this;
            }
            chain hypE2HRead(bool v = true) const {
                info[MISCREG_HYP_E2H_RD] = v;
                return *this;
            }
            chain hypE2HWrite(bool v = true) const {
                info[MISCREG_HYP_E2H_WR] = v;
                return *this;
            }
            chain hypE2H(bool v = true) const {
                hypE2HRead(v);
                hypE2HWrite(v);
                return *this;
            }
            chain hypRead(bool v = true) const {
                hypE2HRead(v);
                info[MISCREG_HYP_RD] = v;
                return *this;
            }
            chain hypWrite(bool v = true) const {
                hypE2HWrite(v);
                info[MISCREG_HYP_WR] = v;
                return *this;
            }
            chain hyp(bool v = true) const {
                hypRead(v);
                hypWrite(v);
                return *this;
            }
            chain monE2HRead(bool v = true) const {
                info[MISCREG_MON_E2H_RD] = v;
                return *this;
            }
            chain monE2HWrite(bool v = true) const {
                info[MISCREG_MON_E2H_WR] = v;
                return *this;
            }
            chain monE2H(bool v = true) const {
                monE2HRead(v);
                monE2HWrite(v);
                return *this;
            }
            chain monSecureRead(bool v = true) const {
                monE2HRead(v);
                info[MISCREG_MON_NS0_RD] = v;
                return *this;
            }
            chain monSecureWrite(bool v = true) const {
                monE2HWrite(v);
                info[MISCREG_MON_NS0_WR] = v;
                return *this;
            }
            chain monNonSecureRead(bool v = true) const {
                monE2HRead(v);
                info[MISCREG_MON_NS1_RD] = v;
                return *this;
            }
            chain monNonSecureWrite(bool v = true) const {
                monE2HWrite(v);
                info[MISCREG_MON_NS1_WR] = v;
                return *this;
            }
            chain mon(bool v = true) const {
                monSecureRead(v);
                monSecureWrite(v);
                monNonSecureRead(v);
                monNonSecureWrite(v);
                return *this;
            }
            chain monSecure(bool v = true) const {
                monSecureRead(v);
                monSecureWrite(v);
                return *this;
            }
            chain monNonSecure(bool v = true) const {
                monNonSecureRead(v);
                monNonSecureWrite(v);
                return *this;
            }
            chain allPrivileges(bool v = true) const {
                userNonSecureRead(v);
                userNonSecureWrite(v);
                userSecureRead(v);
                userSecureWrite(v);
                privNonSecureRead(v);
                privNonSecureWrite(v);
                privSecureRead(v);
                privSecureWrite(v);
                hypRead(v);
                hypWrite(v);
                monSecureRead(v);
                monSecureWrite(v);
                monNonSecureRead(v);
                monNonSecureWrite(v);
                return *this;
            }
            chain nonSecure(bool v = true) const {
                userNonSecureRead(v);
                userNonSecureWrite(v);
                privNonSecureRead(v);
                privNonSecureWrite(v);
                hypRead(v);
                hypWrite(v);
                monNonSecureRead(v);
                monNonSecureWrite(v);
                return *this;
            }
            chain secure(bool v = true) const {
                userSecureRead(v);
                userSecureWrite(v);
                privSecureRead(v);
                privSecureWrite(v);
                monSecureRead(v);
                monSecureWrite(v);
                return *this;
            }
            chain reads(bool v) const {
                userNonSecureRead(v);
                userSecureRead(v);
                privNonSecureRead(v);
                privSecureRead(v);
                hypRead(v);
                monSecureRead(v);
                monNonSecureRead(v);
                return *this;
            }
            chain writes(bool v) const {
                userNonSecureWrite(v);
                userSecureWrite(v);
                privNonSecureWrite(v);
                privSecureWrite(v);
                hypWrite(v);
                monSecureWrite(v);
                monNonSecureWrite(v);
                return *this;
            }
            chain exceptUserMode() const {
                user(0);
                return *this;
            }
            chain highest(ArmSystem *const sys) const;
            MiscRegLUTEntryInitializer(struct MiscRegLUTEntry &e,
                                       std::bitset<NUM_MISCREG_INFOS> &i)
              : entry(e),
                info(i)
            {
                // force unimplemented registers to be thusly declared
                implemented(1);
            }
        };

        const MiscRegLUTEntryInitializer InitReg(uint32_t reg) {
            return MiscRegLUTEntryInitializer(lookUpMiscReg[reg],
                                              miscRegInfo[reg]);
        }

        void initializeMiscRegMetadata();

        RegVal miscRegs[NumMiscRegs];
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
        BaseISADevice &getGICv3CPUInterface(ThreadContext *tc);


      private:
        inline void assert32(ThreadContext *tc) {
            CPSR cpsr M5_VAR_USED = readMiscReg(MISCREG_CPSR, tc);
            assert(cpsr.width);
        }

        inline void assert64(ThreadContext *tc) {
            CPSR cpsr M5_VAR_USED = readMiscReg(MISCREG_CPSR, tc);
            assert(!cpsr.width);
        }

      public:
        void clear(ThreadContext *tc);

      protected:
        void clear();
        void clear32(const ArmISAParams *p, const SCTLR &sctlr_rst);
        void clear64(const ArmISAParams *p);
        void initID32(const ArmISAParams *p);
        void initID64(const ArmISAParams *p);

      public:
        RegVal readMiscRegNoEffect(int misc_reg) const;
        RegVal readMiscReg(int misc_reg, ThreadContext *tc);
        void setMiscRegNoEffect(int misc_reg, RegVal val);
        void setMiscReg(int misc_reg, RegVal val, ThreadContext *tc);

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
                return RegId(VecElemClass, flattenVecElemIndex(regId.index()),
                             regId.elemIndex());
              case VecPredRegClass:
                return RegId(VecPredRegClass,
                             flattenVecPredIndex(regId.index()));
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
                    return 0;  // Never happens.
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
        flattenVecPredIndex(int reg) const
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
                } else {
                    flat_idx = snsBankedIndex64((MiscRegIndex)reg,
                        !inSecureState(miscRegs[MISCREG_SCR],
                                       miscRegs[MISCREG_CPSR]));
                }
            }
            return flat_idx;
        }

        int
        snsBankedIndex64(MiscRegIndex reg, bool ns) const
        {
            int reg_as_int = static_cast<int>(reg);
            if (miscRegInfo[reg][MISCREG_BANKED64]) {
                reg_as_int += (haveSecurity && !ns) ? 2 : 1;
            }
            return reg_as_int;
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

        unsigned getCurSveVecLenInBits(ThreadContext *tc) const;

        unsigned getCurSveVecLenInBitsAtReset() const { return sveVL * 128; }

        static void zeroSveVecRegUpperPart(VecRegContainer &vc,
                                           unsigned eCount);

        void
        serialize(CheckpointOut &cp) const override
        {
            DPRINTF(Checkpoint, "Serializing Arm Misc Registers\n");
            SERIALIZE_ARRAY(miscRegs, NUM_PHYS_MISCREGS);
        }

        void
        unserialize(CheckpointIn &cp) override
        {
            DPRINTF(Checkpoint, "Unserializing Arm Misc Registers\n");
            UNSERIALIZE_ARRAY(miscRegs, NUM_PHYS_MISCREGS);
            CPSR tmp_cpsr = miscRegs[MISCREG_CPSR];
            updateRegMap(tmp_cpsr);
        }

        void startup(ThreadContext *tc);

        void takeOverFrom(ThreadContext *new_tc,
                          ThreadContext *old_tc) override;

        Enums::DecoderFlavor decoderFlavor() const { return _decoderFlavor; }

        /** Returns true if the ISA has a GICv3 cpu interface */
        bool haveGICv3CpuIfc() const
        {
            // gicv3CpuInterface is initialized at startup time, hence
            // trying to read its value before the startup stage will lead
            // to an error
            assert(afterStartup);
            return gicv3CpuInterface != nullptr;
        }

        Enums::VecRegRenameMode
        vecRegRenameMode() const
        {
            return _vecRegRenameMode;
        }

        /// Explicitly import the otherwise hidden startup
        using BaseISA::startup;

        typedef ArmISAParams Params;

        const Params *params() const;

        ISA(Params *p);
    };
}

template<>
struct RenameMode<ArmISA::ISA>
{
    static Enums::VecRegRenameMode
    init(const BaseISA* isa)
    {
        auto arm_isa = dynamic_cast<const ArmISA::ISA *>(isa);
        assert(arm_isa);
        return arm_isa->vecRegRenameMode();
    }

    static Enums::VecRegRenameMode
    mode(const ArmISA::PCState& pc)
    {
        if (pc.aarch64()) {
            return Enums::Full;
        } else {
            return Enums::Elem;
        }
    }

    static bool
    equalsInit(const BaseISA* isa1, const BaseISA* isa2)
    {
        return init(isa1) == init(isa2);
    }
};

#endif
