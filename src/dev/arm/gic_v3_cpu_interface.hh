/*
 * Copyright (c) 2019, 2022 ARM Limited
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
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#ifndef __DEV_ARM_GICV3_CPU_INTERFACE_H__
#define __DEV_ARM_GICV3_CPU_INTERFACE_H__

#include "arch/arm/isa_device.hh"
#include "dev/arm/gic_v3.hh"

namespace gem5
{

class Gicv3Distributor;
class Gicv3Redistributor;

namespace ArmISA
{
class ISA;
}

class Gicv3CPUInterface : public ArmISA::BaseISADevice, public Serializable
{
  private:
    friend class Gicv3Distributor;
    friend class Gicv3Redistributor;
    friend class ArmISA::ISA;

  protected:
    Gicv3 *gic;
    Gicv3Redistributor *redistributor;
    Gicv3Distributor *distributor;

    ThreadContext *tc;
    ArmInterruptPin *maintenanceInterrupt;
    uint32_t cpuId;

    BitUnion64(ICC_CTLR_EL1)
        Bitfield<63, 20> res0_3;
        Bitfield<19> ExtRange;
        Bitfield<18> RSS;
        Bitfield<17, 16> res0_2;
        Bitfield<15> A3V;
        Bitfield<14> SEIS;
        Bitfield<13, 11> IDbits;
        Bitfield<10, 8> PRIbits;
        Bitfield<7> res0_1;
        Bitfield<6> PMHE;
        Bitfield<5, 2> res0_0;
        Bitfield<1> EOImode;
        Bitfield<0> CBPR;
    EndBitUnion(ICC_CTLR_EL1)

    BitUnion64(ICC_CTLR_EL3)
        Bitfield<63, 20> res0_2;
        Bitfield<19> ExtRange;
        Bitfield<18> RSS;
        Bitfield<17> nDS;
        Bitfield<16> res0_1;
        Bitfield<15> A3V;
        Bitfield<14> SEIS;
        Bitfield<13, 11> IDbits;
        Bitfield<10, 8> PRIbits;
        Bitfield<7> res0_0;
        Bitfield<6> PMHE;
        Bitfield<5> RM;
        Bitfield<4> EOImode_EL1NS;
        Bitfield<3> EOImode_EL1S;
        Bitfield<2> EOImode_EL3;
        Bitfield<1> CBPR_EL1NS;
        Bitfield<0> CBPR_EL1S;
    EndBitUnion(ICC_CTLR_EL3)

    BitUnion64(ICC_IGRPEN0_EL1)
        Bitfield<63, 1> res0;
        Bitfield<0> Enable;
    EndBitUnion(ICC_IGRPEN0_EL1)

    BitUnion64(ICC_IGRPEN1_EL1)
        Bitfield<63, 1> res0;
        Bitfield<0> Enable;
    EndBitUnion(ICC_IGRPEN1_EL1)

    BitUnion64(ICC_IGRPEN1_EL3)
        Bitfield<63, 2> res0;
        Bitfield<1> EnableGrp1S;
        Bitfield<0> EnableGrp1NS;
    EndBitUnion(ICC_IGRPEN1_EL3)

    BitUnion64(ICC_SRE_EL1)
        Bitfield<63, 3> res0;
        Bitfield<2> DIB;
        Bitfield<1> DFB;
        Bitfield<0> SRE;
    EndBitUnion(ICC_SRE_EL1)

    BitUnion64(ICC_SRE_EL2)
        Bitfield<63, 4> res0;
        Bitfield<3> Enable;
        Bitfield<2> DIB;
        Bitfield<1> DFB;
        Bitfield<0> SRE;
    EndBitUnion(ICC_SRE_EL2)

    BitUnion64(ICC_SRE_EL3)
        Bitfield<63, 4> res0;
        Bitfield<3> Enable;
        Bitfield<2> DIB;
        Bitfield<1> DFB;
        Bitfield<0> SRE;
    EndBitUnion(ICC_SRE_EL3)

    static const uint8_t PRIORITY_BITS = 5;

    // Minimum BPR for Secure, or when security not enabled
    static const uint8_t GIC_MIN_BPR = 2;
    //  Minimum BPR for Nonsecure when security is enabled
    static const uint8_t GIC_MIN_BPR_NS = GIC_MIN_BPR + 1;

    static const uint8_t VIRTUAL_PRIORITY_BITS = 5;
    static const uint8_t VIRTUAL_PREEMPTION_BITS = 5;
    static const uint8_t VIRTUAL_NUM_LIST_REGS = 16;

    static const uint8_t GIC_MIN_VBPR = 7 - VIRTUAL_PREEMPTION_BITS;

    struct hppi_t
    {
        uint32_t intid;
        uint8_t prio;
        Gicv3::GroupId group;
    };

    hppi_t hppi;

    // GIC CPU interface memory mapped control registers (legacy)
    enum
    {
        GICC_CTLR = 0x0000,
        GICC_PMR = 0x0004,
        GICC_BPR = 0x0008,
        GICC_IAR = 0x000C,
        GICC_EOIR = 0x0010,
        GICC_RPR = 0x0014,
        GICC_HPPI = 0x0018,
        GICC_ABPR = 0x001C,
        GICC_AIAR = 0x0020,
        GICC_AEOIR = 0x0024,
        GICC_AHPPIR = 0x0028,
        GICC_STATUSR = 0x002C,
        GICC_IIDR = 0x00FC,
    };

    static const AddrRange GICC_APR;
    static const AddrRange GICC_NSAPR;

    // GIC CPU virtual interface memory mapped control registers (legacy)
    enum
    {
        GICH_HCR = 0x0000,
        GICH_VTR = 0x0004,
        GICH_VMCR = 0x0008,
        GICH_MISR = 0x0010,
        GICH_EISR = 0x0020,
        GICH_ELRSR = 0x0030,
    };

    static const AddrRange GICH_APR;
    static const AddrRange GICH_LR;

  public:
    BitUnion64(ICH_HCR_EL2)
        Bitfield<63, 32> res0_2;
        Bitfield<31, 27> EOIcount;
        Bitfield<26, 15> res0_1;
        Bitfield<14> TDIR;
        Bitfield<13> TSEI;
        Bitfield<12> TALL1;
        Bitfield<11> TALL0;
        Bitfield<10> TC;
        Bitfield<9, 8> res0_0;
        Bitfield<7> VGrp1DIE;
        Bitfield<6> VGrp1EIE;
        Bitfield<5> VGrp0DIE;
        Bitfield<4> VGrp0EIE;
        Bitfield<3> NPIE;
        Bitfield<2> LRENPIE;
        Bitfield<1> UIE;
        Bitfield<0> En;
    EndBitUnion(ICH_HCR_EL2)

  protected:
    BitUnion64(ICH_LR_EL2)
        Bitfield<63, 62> State;
        Bitfield<61> HW;
        Bitfield<60> Group;
        Bitfield<59, 56> res0_1;
        Bitfield<55, 48> Priority;
        Bitfield<47, 45> res0_0;
        Bitfield<44, 32> pINTID;
        Bitfield<41> EOI;
        Bitfield<31, 0> vINTID;
    EndBitUnion(ICH_LR_EL2)

    static const uint64_t ICH_LR_EL2_STATE_INVALID = 0;
    static const uint64_t ICH_LR_EL2_STATE_PENDING = 1;
    static const uint64_t ICH_LR_EL2_STATE_ACTIVE = 2;
    static const uint64_t ICH_LR_EL2_STATE_ACTIVE_PENDING = 3;

    BitUnion32(ICH_LRC)
        Bitfield<31, 30> State;
        Bitfield<29> HW;
        Bitfield<28> Group;
        Bitfield<27, 24> res0_1;
        Bitfield<23, 16> Priority;
        Bitfield<15, 13> res0_0;
        Bitfield<12, 0> pINTID;
        Bitfield<9> EOI;
    EndBitUnion(ICH_LRC)

    BitUnion64(ICH_MISR_EL2)
        Bitfield<63, 8> res0;
        Bitfield<7> VGrp1D;
        Bitfield<6> VGrp1E;
        Bitfield<5> VGrp0D;
        Bitfield<4> VGrp0E;
        Bitfield<3> NP;
        Bitfield<2> LRENP;
        Bitfield<1> U;
        Bitfield<0> EOI;
    EndBitUnion(ICH_MISR_EL2)

    BitUnion64(ICH_VMCR_EL2)
        Bitfield<63, 32> res0_2;
        Bitfield<31, 24> VPMR;
        Bitfield<23, 21> VBPR0;
        Bitfield<20, 18> VBPR1;
        Bitfield<17, 10> res0_1;
        Bitfield<9> VEOIM;
        Bitfield<8, 5> res0_0;
        Bitfield<4> VCBPR;
        Bitfield<3> VFIQEn;
        Bitfield<2> VAckCtl;
        Bitfield<1> VENG1;
        Bitfield<0> VENG0;
    EndBitUnion(ICH_VMCR_EL2)

    BitUnion64(ICH_VTR_EL2)
        Bitfield<63, 32> res0_1;
        Bitfield<31, 29> PRIbits;
        Bitfield<28, 26> PREbits;
        Bitfield<25, 23> IDbits;
        Bitfield<22> SEIS;
        Bitfield<21> A3V;
        Bitfield<20> res1;
        Bitfield<19> TDS;
        Bitfield<18, 5> res0_0;
        Bitfield<4, 0> ListRegs;
    EndBitUnion(ICH_VTR_EL2)

    BitUnion64(ICV_CTLR_EL1)
        Bitfield<63, 19> res0_2;
        Bitfield<18> RSS;
        Bitfield<17, 16> res0_1;
        Bitfield<15> A3V;
        Bitfield<14> SEIS;
        Bitfield<13, 11> IDbits;
        Bitfield<10, 8> PRIbits;
        Bitfield<7, 2> res0_0;
        Bitfield<1> EOImode;
        Bitfield<0> CBPR;
    EndBitUnion(ICV_CTLR_EL1)

  protected:
    void activateIRQ(uint32_t intid, Gicv3::GroupId group);
    void generateSGI(RegVal val, Gicv3::GroupId group);
    ArmISA::ExceptionLevel currEL() const;
    void deactivateIRQ(uint32_t intid, Gicv3::GroupId group);
    void dropPriority(Gicv3::GroupId group);
    uint64_t eoiMaintenanceInterruptStatus() const;
    bool getHCREL2FMO() const;
    bool getHCREL2IMO() const;
    uint32_t getHPPIR0() const;
    uint32_t getHPPIR1() const;
    int getHPPVILR() const;
    bool groupEnabled(Gicv3::GroupId group) const;
    uint32_t groupPriorityMask(Gicv3::GroupId group);
    bool haveEL(ArmISA::ExceptionLevel el) const;
    int highestActiveGroup() const;
    uint8_t highestActivePriority() const;
    bool hppiCanPreempt();
    bool hppviCanPreempt(int lrIdx) const;
    bool inSecureState() const;
    ArmISA::InterruptTypes intSignalType(Gicv3::GroupId group) const;
    bool isAA64() const;
    bool isEL3OrMon() const;
    bool isEOISplitMode() const;
    bool isSecureBelowEL3() const;
    ICH_MISR_EL2 maintenanceInterruptStatus() const;
    void resetHppi(uint32_t intid);
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void update();
    void updateDistributor();
    void virtualActivateIRQ(uint32_t lrIdx);
    void virtualDeactivateIRQ(int lrIdx);
    uint8_t virtualDropPriority();
    int virtualFindActive(uint32_t intid) const;
    uint32_t virtualGroupPriorityMask(Gicv3::GroupId group) const;
    uint8_t virtualHighestActivePriority() const;
    void virtualIncrementEOICount();
    bool virtualIsEOISplitMode() const;
    void virtualUpdate();
    RegVal bpr1(Gicv3::GroupId group);
    bool havePendingInterrupts(void) const;
    void clearPendingInterrupts(void);
    void assertWakeRequest(void);
    void deassertWakeRequest(void);

    RegVal readBankedMiscReg(ArmISA::MiscRegIndex misc_reg) const;
    void setBankedMiscReg(ArmISA::MiscRegIndex misc_reg, RegVal val) const;

  public:
    Gicv3CPUInterface(Gicv3 *gic, ThreadContext *tc);

    void init();

  public:
    void copy(Gicv3Registers *from, Gicv3Registers *to);

  public: // BaseISADevice
    RegVal readMiscReg(int misc_reg) override;
    void setMiscReg(int misc_reg, RegVal val) override;
    void setThreadContext(ThreadContext *tc) override;
};

} // namespace gem5

#endif //__DEV_ARM_GICV3_CPU_INTERFACE_H__
