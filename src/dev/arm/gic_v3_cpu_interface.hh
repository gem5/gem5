/*
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
 *
 * Authors: Jairo Balart
 */

#ifndef __DEV_ARM_GICV3_CPU_INTERFACE_H__
#define __DEV_ARM_GICV3_CPU_INTERFACE_H__

#include "arch/arm/isa_device.hh"
#include "dev/arm/gic_v3.hh"

class Gicv3Redistributor;
class Gicv3Distributor;

class Gicv3CPUInterface : public ArmISA::BaseISADevice, public Serializable
{
  private:

    friend class Gicv3Redistributor;
    friend class Gicv3Distributor;

  protected:

    Gicv3 * gic;
    Gicv3Redistributor * redistributor;
    Gicv3Distributor * distributor;
    uint32_t cpuId;

    static const uint32_t ICC_SRE_EL1_SRE = 1 << 0;
    static const uint32_t ICC_SRE_EL1_DFB = 1 << 1;
    static const uint32_t ICC_SRE_EL1_DIB = 1 << 2;

    static const uint32_t ICC_SRE_EL2_SRE = 1 << 0;
    static const uint32_t ICC_SRE_EL2_DFB = 1 << 1;
    static const uint32_t ICC_SRE_EL2_DIB = 1 << 2;
    static const uint32_t ICC_SRE_EL2_ENABLE = 1 << 3;

    static const uint32_t ICC_SRE_EL3_SRE = 1 << 0;
    static const uint32_t ICC_SRE_EL3_DFB = 1 << 1;
    static const uint32_t ICC_SRE_EL3_DIB = 1 << 2;
    static const uint32_t ICC_SRE_EL3_ENABLE = 1 << 3;

    static const uint32_t ICC_CTLR_EL3_CBPR_EL1S = 1 << 0;
    static const uint32_t ICC_CTLR_EL3_CBPR_EL1NS = 1 << 1;
    static const uint32_t ICC_CTLR_EL3_EOIMODE_EL3 = 1 << 2;
    static const uint32_t ICC_CTLR_EL3_EOIMODE_EL1S = 1 << 3;
    static const uint32_t ICC_CTLR_EL3_EOIMODE_EL1NS = 1 << 4;
    static const uint32_t ICC_CTLR_EL3_RM = 1 << 5;
    static const uint32_t ICC_CTLR_EL3_PMHE = 1 << 6;
    static const uint32_t ICC_CTLR_EL3_PRIBITS_SHIFT = 8;
    static const uint32_t ICC_CTLR_EL3_IDBITS_SHIFT = 11;
    static const uint32_t ICC_CTLR_EL3_SEIS = 1 << 14;
    static const uint32_t ICC_CTLR_EL3_A3V = 1 << 15;
    static const uint32_t ICC_CTLR_EL3_nDS = 1 << 17;
    static const uint32_t ICC_CTLR_EL3_RSS = 1 << 18;

    static const uint32_t ICC_CTLR_EL1_CBPR = 1 << 0;
    static const uint32_t ICC_CTLR_EL1_EOIMODE = 1 << 1;
    static const uint32_t ICC_CTLR_EL1_PMHE = 1 << 6;
    static const uint32_t ICC_CTLR_EL1_SEIS = 1 << 14;
    static const uint32_t ICC_CTLR_EL1_A3V = 1 << 15;
    static const uint32_t ICC_CTLR_EL1_RSS = 1 << 18;
    static const uint32_t ICC_CTLR_EL1_PRIBITS_SHIFT = 8;
    static const uint32_t ICC_CTLR_EL1_PRIBITS_MASK =
        7U << ICC_CTLR_EL1_PRIBITS_SHIFT;
    static const uint32_t ICC_CTLR_EL1_IDBITS_SHIFT = 11;

    static const uint32_t ICC_IGRPEN0_EL1_ENABLE = 1 << 0;
    static const uint32_t ICC_IGRPEN1_EL1_ENABLE = 1 << 0;

    static const uint32_t ICC_IGRPEN1_EL3_ENABLEGRP1NS = 1 << 0;
    static const uint32_t ICC_IGRPEN1_EL3_ENABLEGRP1S = 1 << 1;

    static const uint8_t PRIORITY_BITS = 5;

    /* Minimum BPR for Secure, or when security not enabled */
    static const uint8_t GIC_MIN_BPR = 2;
    /* Minimum BPR for Nonsecure when security is enabled */
    static const uint8_t GIC_MIN_BPR_NS = GIC_MIN_BPR + 1;

    static const uint8_t VIRTUAL_PRIORITY_BITS = 5;
    static const uint8_t VIRTUAL_PREEMPTION_BITS = 5;
    static const uint8_t VIRTUAL_NUM_LIST_REGS = 16;

    static const uint8_t GIC_MIN_VBPR = 7 - VIRTUAL_PREEMPTION_BITS;

    typedef struct {
        uint32_t intid;
        uint8_t prio;
        Gicv3::GroupId group;
    } hppi_t;

    hppi_t hppi;

    // GIC CPU interface memory mapped control registers (legacy)
    enum {
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
    enum {
        GICH_HCR = 0x0000,
        GICH_VTR = 0x0004,
        GICH_VMCR = 0x0008,
        GICH_MISR = 0x0010,
        GICH_EISR = 0x0020,
        GICH_ELRSR = 0x0030,
    };

    static const AddrRange GICH_APR;
    static const AddrRange GICH_LR;

    static const uint32_t ICH_HCR_EL2_EN = 1 << 0;
    static const uint32_t ICH_HCR_EL2_UIE = 1 << 1;
    static const uint32_t ICH_HCR_EL2_LRENPIE = 1 << 2;
    static const uint32_t ICH_HCR_EL2_NPIE = 1 << 3;
    static const uint32_t ICH_HCR_EL2_VGRP0EIE = 1 << 4;
    static const uint32_t ICH_HCR_EL2_VGRP0DIE = 1 << 5;
    static const uint32_t ICH_HCR_EL2_VGRP1EIE = 1 << 6;
    static const uint32_t ICH_HCR_EL2_VGRP1DIE = 1 << 7;
    static const uint32_t ICH_HCR_EL2_TC = 1 << 10;
    static const uint32_t ICH_HCR_EL2_TALL0 = 1 << 11;
    static const uint32_t ICH_HCR_EL2_TALL1 = 1 << 12;
    static const uint32_t ICH_HCR_EL2_TSEI = 1 << 13;
    static const uint32_t ICH_HCR_EL2_TDIR = 1 << 14;
    static const uint32_t ICH_HCR_EL2_EOICOUNT_MASK = 0x1fU << 27;

    static const uint64_t ICH_LR_EL2_VINTID_SHIFT = 0;
    static const uint64_t ICH_LR_EL2_VINTID_LENGTH = 32;
    static const uint64_t ICH_LR_EL2_VINTID_MASK =
        (0xffffffffULL << ICH_LR_EL2_VINTID_SHIFT);
    static const uint64_t ICH_LR_EL2_PINTID_SHIFT = 32;
    static const uint64_t ICH_LR_EL2_PINTID_LENGTH = 10;
    static const uint64_t ICH_LR_EL2_PINTID_MASK =
        (0x3ffULL << ICH_LR_EL2_PINTID_SHIFT);
    /* Note that EOI shares with the top bit of the pINTID field */
    static const uint64_t ICH_LR_EL2_EOI = (1ULL << 41);
    static const uint64_t ICH_LR_EL2_PRIORITY_SHIFT = 48;
    static const uint64_t ICH_LR_EL2_PRIORITY_LENGTH = 8;
    static const uint64_t ICH_LR_EL2_PRIORITY_MASK =
        (0xffULL << ICH_LR_EL2_PRIORITY_SHIFT);
    static const uint64_t ICH_LR_EL2_GROUP = (1ULL << 60);
    static const uint64_t ICH_LR_EL2_HW = (1ULL << 61);
    static const uint64_t ICH_LR_EL2_STATE_SHIFT = 62;
    static const uint64_t ICH_LR_EL2_STATE_LENGTH = 2;
    static const uint64_t ICH_LR_EL2_STATE_MASK =
        (3ULL << ICH_LR_EL2_STATE_SHIFT);
    /* values for the state field: */
    static const uint64_t ICH_LR_EL2_STATE_INVALID = 0;
    static const uint64_t ICH_LR_EL2_STATE_PENDING = 1;
    static const uint64_t ICH_LR_EL2_STATE_ACTIVE = 2;
    static const uint64_t ICH_LR_EL2_STATE_ACTIVE_PENDING = 3;
    static const uint64_t ICH_LR_EL2_STATE_PENDING_BIT =
        (1ULL << ICH_LR_EL2_STATE_SHIFT);
    static const uint64_t ICH_LR_EL2_STATE_ACTIVE_BIT =
        (2ULL << ICH_LR_EL2_STATE_SHIFT);

    static const uint64_t ICH_LRC_PRIORITY_SHIFT =
        ICH_LR_EL2_PRIORITY_SHIFT - 32;
    static const uint64_t ICH_LRC_PRIORITY_LENGTH =
        ICH_LR_EL2_PRIORITY_LENGTH;

    static const uint32_t ICH_MISR_EL2_EOI = (1 << 0);
    static const uint32_t ICH_MISR_EL2_U = (1 << 1);
    static const uint32_t ICH_MISR_EL2_LRENP = (1 << 2);
    static const uint32_t ICH_MISR_EL2_NP = (1 << 3);
    static const uint32_t ICH_MISR_EL2_VGRP0E = (1 << 4);
    static const uint32_t ICH_MISR_EL2_VGRP0D = (1 << 5);
    static const uint32_t ICH_MISR_EL2_VGRP1E = (1 << 6);
    static const uint32_t ICH_MISR_EL2_VGRP1D = (1 << 7);

    static const uint32_t ICH_VMCR_EL2_VENG0_SHIFT = 0;
    static const uint32_t ICH_VMCR_EL2_VENG0 =
        (1 << ICH_VMCR_EL2_VENG0_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VENG1_SHIFT = 1;
    static const uint32_t ICH_VMCR_EL2_VENG1 =
        (1 << ICH_VMCR_EL2_VENG1_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VACKCTL = (1 << 2);
    static const uint32_t ICH_VMCR_EL2_VFIQEN = (1 << 3);
    static const uint32_t ICH_VMCR_EL2_VCBPR_SHIFT = 4;
    static const uint32_t ICH_VMCR_EL2_VCBPR =
        (1 << ICH_VMCR_EL2_VCBPR_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VEOIM_SHIFT = 9;
    static const uint32_t ICH_VMCR_EL2_VEOIM =
        (1 << ICH_VMCR_EL2_VEOIM_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VBPR1_SHIFT = 18;
    static const uint32_t ICH_VMCR_EL2_VBPR1_LENGTH = 3;
    static const uint32_t ICH_VMCR_EL2_VBPR1_MASK =
        (0x7U << ICH_VMCR_EL2_VBPR1_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VBPR0_SHIFT = 21;
    static const uint32_t ICH_VMCR_EL2_VBPR0_LENGTH = 3;
    static const uint32_t ICH_VMCR_EL2_VBPR0_MASK =
        (0x7U << ICH_VMCR_EL2_VBPR0_SHIFT);
    static const uint32_t ICH_VMCR_EL2_VPMR_SHIFT = 24;
    static const uint32_t ICH_VMCR_EL2_VPMR_LENGTH = 8;
    static const uint32_t ICH_VMCR_EL2_VPMR_MASK =
        (0xffU << ICH_VMCR_EL2_VPMR_SHIFT);

    static const uint32_t ICH_VTR_EL2_LISTREGS_SHIFT = 0;
    static const uint32_t ICH_VTR_EL2_TDS = 1 << 19;
    static const uint32_t ICH_VTR_EL2_NV4 = 1 << 20;
    static const uint32_t ICH_VTR_EL2_A3V = 1 << 21;
    static const uint32_t ICH_VTR_EL2_SEIS = 1 << 22;
    static const uint32_t ICH_VTR_EL2_IDBITS_SHIFT = 23;
    static const uint32_t ICH_VTR_EL2_PREBITS_SHIFT = 26;
    static const uint32_t ICH_VTR_EL2_PRIBITS_SHIFT = 29;

  public:

    Gicv3CPUInterface(Gicv3 * gic, uint32_t cpu_id);
    ~Gicv3CPUInterface();
    void init();
    void initState();

    RegVal readMiscReg(int misc_reg) override;
    void setMiscReg(int misc_reg, RegVal val) override;
    void update();
    void virtualUpdate();

    void serialize(CheckpointOut & cp) const override;
    void unserialize(CheckpointIn & cp) override;

  protected:

    void reset();
    bool hppiCanPreempt();
    bool hppviCanPreempt(int lrIdx);
    bool groupEnabled(Gicv3::GroupId group);
    uint8_t highestActivePriority();
    uint8_t virtualHighestActivePriority();
    bool inSecureState();
    int currEL();
    bool haveEL(ArmISA::ExceptionLevel el);
    void activateIRQ(uint32_t intid, Gicv3::GroupId group);
    void virtualActivateIRQ(uint32_t lrIdx);
    void deactivateIRQ(uint32_t intid, Gicv3::GroupId group);
    void virtualDeactivateIRQ(int lrIdx);
    uint32_t groupPriorityMask(Gicv3::GroupId group);
    uint32_t virtualGroupPriorityMask(Gicv3::GroupId group);
    void dropPriority(Gicv3::GroupId group);
    uint8_t virtualDropPriority();
    ArmISA::InterruptTypes intSignalType(Gicv3::GroupId group);
    bool isEOISplitMode();
    bool virtualIsEOISplitMode();
    bool isSecureBelowEL3();
    bool inSecureState2();
    uint32_t eoiMaintenanceInterruptStatus(uint32_t * misr);
    uint32_t maintenanceInterruptStatus();
    int highestActiveGroup();
    bool getHCREL2FMO();
    bool getHCREL2IMO();
    uint32_t getHPPIR1();
    uint32_t getHPPIR0();
    int getHPPVILR();
    int virtualFindActive(uint32_t intid);
    void virtualIncrementEOICount();
    bool isEL3OrMon();
    bool isAA64();
};

#endif //__DEV_ARM_GICV3_CPU_INTERFACE_H__
