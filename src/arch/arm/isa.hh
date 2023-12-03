/*
 * Copyright (c) 2010, 2012-2023 Arm Limited
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
#include "arch/arm/mmu.hh"
#include "arch/arm/pcstate.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/mat.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/system.hh"
#include "arch/arm/types.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/isa.hh"
#include "debug/Checkpoint.hh"
#include "enums/DecoderFlavor.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct ArmISAParams;
struct DummyArmISADeviceParams;
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
    const enums::DecoderFlavor _decoderFlavor;

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
    bool haveLargeAsid64;
    uint8_t physAddrRange;

    /** SVE vector length in quadwords */
    unsigned sveVL;

    /** SME vector length in quadwords */
    unsigned smeVL;

    /** This could be either a FS or a SE release */
    const ArmRelease *release;

    /**
     * If true, accesses to IMPLEMENTATION DEFINED registers are treated
     * as NOP hence not causing UNDEFINED INSTRUCTION.
     */
    bool impdefAsNop;

    SelfDebug *selfDebug;

    const MiscRegLUTEntryInitializer
    InitReg(uint32_t reg)
    {
        return MiscRegLUTEntryInitializer(lookUpMiscReg[reg]);
    }

    void initializeMiscRegMetadata();

    BaseISADevice &getGenericTimer();
    BaseISADevice &getGICv3CPUInterface();
    BaseISADevice *getGICv3CPUInterface(ThreadContext *tc);

    RegVal miscRegs[NUM_MISCREGS];
    const RegId *intRegMap;

    void
    updateRegMap(CPSR cpsr)
    {
        if (cpsr.width == 0) {
            intRegMap = int_reg::Reg64Map;
        } else {
            switch (cpsr.mode) {
            case MODE_USER:
            case MODE_SYSTEM:
                intRegMap = int_reg::RegUsrMap;
                break;
            case MODE_FIQ:
                intRegMap = int_reg::RegFiqMap;
                break;
            case MODE_IRQ:
                intRegMap = int_reg::RegIrqMap;
                break;
            case MODE_SVC:
                intRegMap = int_reg::RegSvcMap;
                break;
            case MODE_MON:
                intRegMap = int_reg::RegMonMap;
                break;
            case MODE_ABORT:
                intRegMap = int_reg::RegAbtMap;
                break;
            case MODE_HYP:
                intRegMap = int_reg::RegHypMap;
                break;
            case MODE_UNDEFINED:
                intRegMap = int_reg::RegUndMap;
                break;
            default:
                panic("Unrecognized mode setting in CPSR.\n");
            }
        }
    }

  public:
    const RegId &
    mapIntRegId(RegIndex idx) const
    {
        return intRegMap[idx];
    }

  public:
    void clear() override;

  protected:
    void addressTranslation(MMU::ArmTranslationType tran_type,
                            BaseMMU::Mode mode, Request::Flags flags,
                            RegVal val);
    void addressTranslation64(MMU::ArmTranslationType tran_type,
                              BaseMMU::Mode mode, Request::Flags flags,
                              RegVal val);

  public:
    SelfDebug *
    getSelfDebug() const
    {
        return selfDebug;
    }

    static SelfDebug *
    getSelfDebug(ThreadContext *tc)
    {
        auto *arm_isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
        return arm_isa->getSelfDebug();
    }

    const ArmRelease *
    getRelease() const
    {
        return release;
    }

    RegVal readMiscRegNoEffect(RegIndex idx) const override;
    RegVal readMiscReg(RegIndex idx) override;
    void setMiscRegNoEffect(RegIndex idx, RegVal val) override;
    void setMiscReg(RegIndex, RegVal val) override;

    RegVal readMiscRegReset(RegIndex) const;
    void setMiscRegReset(RegIndex, RegVal val);

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
        } else if (lookUpMiscReg[reg].info[MISCREG_MUTEX]) {
            // Mutually exclusive CP15 register
            switch (reg) {
            case MISCREG_PRRR_MAIR0:
            case MISCREG_PRRR_MAIR0_NS:
            case MISCREG_PRRR_MAIR0_S: {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                // If the muxed reg has been flattened, work out the
                // offset and apply it to the unmuxed reg
                int idxOffset = reg - MISCREG_PRRR_MAIR0;
                if (ttbcr.eae)
                    flat_idx = flattenMiscIndex(MISCREG_MAIR0 + idxOffset);
                else
                    flat_idx = flattenMiscIndex(MISCREG_PRRR + idxOffset);
            } break;
            case MISCREG_NMRR_MAIR1:
            case MISCREG_NMRR_MAIR1_NS:
            case MISCREG_NMRR_MAIR1_S: {
                TTBCR ttbcr = readMiscRegNoEffect(MISCREG_TTBCR);
                // If the muxed reg has been flattened, work out the
                // offset and apply it to the unmuxed reg
                int idxOffset = reg - MISCREG_NMRR_MAIR1;
                if (ttbcr.eae)
                    flat_idx = flattenMiscIndex(MISCREG_MAIR1 + idxOffset);
                else
                    flat_idx = flattenMiscIndex(MISCREG_NMRR + idxOffset);
            } break;
            case MISCREG_PMXEVTYPER_PMCCFILTR: {
                PMSELR pmselr = miscRegs[MISCREG_PMSELR];
                if (pmselr.sel == 31)
                    flat_idx = flattenMiscIndex(MISCREG_PMCCFILTR);
                else
                    flat_idx = flattenMiscIndex(MISCREG_PMXEVTYPER);
            } break;
            default:
                panic("Unrecognized misc. register.\n");
                break;
            }
        } else {
            if (lookUpMiscReg[reg].info[MISCREG_BANKED]) {
                bool secure_reg = !highestELIs64 && inSecureState();
                flat_idx += secure_reg ? 2 : 1;
            } else {
                flat_idx =
                    snsBankedIndex64((MiscRegIndex)reg, !inSecureState());
            }
        }
        return flat_idx;
    }

    /**
     * Returns the enconcing equivalent when VHE is implemented and
     * HCR_EL2.E2H is enabled and executing at EL2
     */
    int redirectRegVHE(int misc_reg);

    int
    snsBankedIndex64(MiscRegIndex reg, bool ns) const
    {
        int reg_as_int = static_cast<int>(reg);
        if (lookUpMiscReg[reg].info[MISCREG_BANKED64]) {
            reg_as_int +=
                (release->has(ArmExtension::SECURITY) && !ns) ? 2 : 1;
        }
        return reg_as_int;
    }

    std::pair<int, int>
    getMiscIndices(int misc_reg) const
    {
        // Note: indexes of AArch64 registers are left unchanged
        int flat_idx = flattenMiscIndex(misc_reg);

        if (lookUpMiscReg[flat_idx].lower == 0) {
            return std::make_pair(flat_idx, 0);
        }

        // do additional S/NS flattenings if mapped to NS while in S
        bool S = !highestELIs64 && inSecureState();

        int lower = lookUpMiscReg[flat_idx].lower;
        int upper = lookUpMiscReg[flat_idx].upper;
        // upper == 0, which is CPSR, is not MISCREG_BANKED_CHILD (no-op)
        lower += S && lookUpMiscReg[lower].info[MISCREG_BANKED_CHILD];
        upper += S && lookUpMiscReg[upper].info[MISCREG_BANKED_CHILD];
        return std::make_pair(lower, upper);
    }

    /** Return true if the PE is in Secure state */
    bool inSecureState() const;

    /**
     * Returns the current Exception Level (EL) of the ISA object
     */
    ExceptionLevel currEL() const;

    unsigned getCurSveVecLenInBits() const;

    unsigned
    getCurSveVecLenInBitsAtReset() const
    {
        return sveVL * 128;
    }

    unsigned getCurSmeVecLenInBits() const;

    unsigned
    getCurSmeVecLenInBitsAtReset() const
    {
        return smeVL * 128;
    }

    template <typename Elem>
    static void
    zeroSveVecRegUpperPart(Elem *v, unsigned eCount)
    {
        static_assert(sizeof(Elem) <= sizeof(uint64_t),
                      "Elem type is too large.");
        eCount *= (sizeof(uint64_t) / sizeof(Elem));
        for (int i = 16 / sizeof(Elem); i < eCount; ++i) {
            v[i] = 0;
        }
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void startup() override;

    void setupThreadContext();

    PCStateBase *
    newPCState(Addr new_inst_addr = 0) const override
    {
        return new PCState(new_inst_addr);
    }

    void takeOverFrom(ThreadContext *new_tc, ThreadContext *old_tc) override;

    enums::DecoderFlavor
    decoderFlavor() const
    {
        return _decoderFlavor;
    }

    PARAMS(ArmISA);

    ISA(const Params &p);

    uint64_t
    getExecutingAsid() const override
    {
        return readMiscRegNoEffect(MISCREG_CONTEXTIDR);
    }

    bool
    inUserMode() const override
    {
        CPSR cpsr = miscRegs[MISCREG_CPSR];
        return ArmISA::inUserMode(cpsr);
    }

    void copyRegsFrom(ThreadContext *src) override;

    void handleLockedRead(const RequestPtr &req) override;
    void handleLockedRead(ExecContext *xc, const RequestPtr &req) override;

    bool handleLockedWrite(const RequestPtr &req,
                           Addr cacheBlockMask) override;
    bool handleLockedWrite(ExecContext *xc, const RequestPtr &req,
                           Addr cacheBlockMask) override;

    void handleLockedSnoop(PacketPtr pkt, Addr cacheBlockMask) override;
    void handleLockedSnoop(ExecContext *xc, PacketPtr pkt,
                           Addr cacheBlockMask) override;
    void handleLockedSnoopHit() override;
    void handleLockedSnoopHit(ExecContext *xc) override;

    void globalClearExclusive() override;
    void globalClearExclusive(ExecContext *xc) override;
};

} // namespace ArmISA
} // namespace gem5

#endif
