/*
 * Copyright (c) 2013 ARM Limited
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
 * Authors: Matt Evans
 */


/** @file
 * Implementiation of a GIC-400 List Register-based VGIC interface.
 * The VGIC is, in this implementation, completely separate from the GIC itself.
 * Only a VIRQ line to the CPU and a PPI line to the GIC (for a HV maintenance IRQ)
 * is required.
 *
 * The mode in which the List Registers may flag (via LR.HW) that a hardware EOI
 * is to be performed is NOT supported.  (This requires tighter integration with
 * the GIC.)
 */

#ifndef __DEV_ARM_VGIC_H__
#define __DEV_ARM_VGIC_H__

#include <algorithm>
#include <array>

#include "base/addr_range.hh"
#include "base/bitunion.hh"
#include "cpu/intr_control.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/VGic.hh"

class VGic : public PioDevice
{
  private:
    static const int VGIC_CPU_MAX       = 256;
    static const int NUM_LR             = 4;

    static const int GICH_SIZE          = 0x200;
    static const int GICH_REG_SIZE      = 0x2000;

    static const int GICH_HCR           = 0x000;
    static const int GICH_VTR           = 0x004;
    static const int GICH_VMCR          = 0x008;
    static const int GICH_MISR          = 0x010;
    static const int GICH_EISR0         = 0x020;
    static const int GICH_EISR1         = 0x024;
    static const int GICH_ELSR0         = 0x030;
    static const int GICH_ELSR1         = 0x034;
    static const int GICH_APR0          = 0x0f0;
    static const int GICH_LR0           = 0x100;
    static const int GICH_LR1           = 0x104;
    static const int GICH_LR2           = 0x108;
    static const int GICH_LR3           = 0x10c;

    static const int GICV_SIZE          = 0x2000;
    static const int GICV_CTLR          = 0x000;
    static const int GICV_PMR           = 0x004;
    static const int GICV_BPR           = 0x008;
    static const int GICV_IAR           = 0x00c;
    static const int GICV_EOIR          = 0x010;
    static const int GICV_RPR           = 0x014;
    static const int GICV_HPPIR         = 0x018;
    static const int GICV_ABPR          = 0x01c;
    static const int GICV_AIAR          = 0x020;
    static const int GICV_AEOIR         = 0x024;
    static const int GICV_AHPPIR        = 0x028;
    static const int GICV_APR0          = 0x0d0;
    static const int GICV_IIDR          = 0x0fc;
    static const int GICV_DIR           = 0x1000;

    static const uint32_t LR_PENDING    = 1;
    static const uint32_t LR_ACTIVE     = 2;

    /** Post interrupt to CPU */
    void processPostVIntEvent(uint32_t cpu);

    EventFunctionWrapper *postVIntEvent[VGIC_CPU_MAX];
    bool        maintIntPosted[VGIC_CPU_MAX];
    bool        vIntPosted[VGIC_CPU_MAX];

    Platform *platform;
    BaseGic *gic;

    Addr vcpuAddr;
    Addr hvAddr;
    Tick pioDelay;
    int maintInt;

    BitUnion32(ListReg)
    Bitfield<31> HW;
    Bitfield<30> Grp1;
    Bitfield<29,28> State;
    Bitfield<27,23> Priority;
    Bitfield<19> EOI;
    Bitfield<12,10> CpuID;
    Bitfield<9,0> VirtualID;
    EndBitUnion(ListReg)

    BitUnion32(HCR)
    Bitfield<31,27> EOICount;
    Bitfield<7> VGrp1DIE;
    Bitfield<6> VGrp1EIE;
    Bitfield<5> VGrp0DIE;
    Bitfield<4> VGrp0EIE;
    Bitfield<3> NPIE;
    Bitfield<2> LRENPIE;
    Bitfield<1> UIE;
    Bitfield<0> En;
    EndBitUnion(HCR)

    BitUnion32(VCTLR)
    Bitfield<9> EOImode;
    Bitfield<4> CPBR;
    Bitfield<3> FIQEn;
    Bitfield<2> AckCtl;
    Bitfield<1> EnGrp1;
    Bitfield<0> En;     // This gets written to enable, not group 1.
    EndBitUnion(VCTLR)

    /* State per CPU.  EVERYTHING should be in this struct and simply replicated
     * N times.
     */
    struct vcpuIntData : public Serializable {
        vcpuIntData()
            : vctrl(0), hcr(0), eisr(0), VMGrp0En(0), VMGrp1En(0),
              VMAckCtl(0), VMFiqEn(0), VMCBPR(0), VEM(0), VMABP(0), VMBP(0),
              VMPriMask(0)
        {
            std::fill(LR.begin(), LR.end(), 0);
        }
        virtual ~vcpuIntData() {}

        std::array<ListReg, NUM_LR> LR;
        VCTLR vctrl;

        HCR hcr;
        uint64_t eisr;

        /* Host info, guest info (should be 100% accessible via GICH_* regs!) */
        uint8_t VMGrp0En;
        uint8_t VMGrp1En;
        uint8_t VMAckCtl;
        uint8_t VMFiqEn;
        uint8_t VMCBPR;
        uint8_t VEM;
        uint8_t VMABP;
        uint8_t VMBP;
        uint8_t VMPriMask;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    struct std::array<vcpuIntData, VGIC_CPU_MAX>  vcpuData;

  public:
   typedef VGicParams Params;
   const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    VGic(const Params *p);
    ~VGic();

    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    Tick readVCpu(PacketPtr pkt);
    Tick readCtrl(PacketPtr pkt);

    Tick writeVCpu(PacketPtr pkt);
    Tick writeCtrl(PacketPtr pkt);

    void updateIntState(ContextID ctx_id);
    uint32_t getMISR(struct vcpuIntData *vid);
    void postVInt(uint32_t cpu, Tick when);
    void unPostVInt(uint32_t cpu);
    void postMaintInt(uint32_t cpu);
    void unPostMaintInt(uint32_t cpu);

    unsigned int lrPending(struct vcpuIntData *vid)
    {
        unsigned int pend = 0;
        for (int i = 0; i < NUM_LR; i++) {
            if (vid->LR[i].State & LR_PENDING)
                pend++;
        }
        return pend;
    }
    unsigned int lrValid(struct vcpuIntData *vid)
    {
        unsigned int valid = 0;
        for (int i = 0; i < NUM_LR; i++) {
            if (vid->LR[i].State)
                valid++;
        }
        return valid;
    }

    /** Returns LR index or -1 if none pending */
    int findHighestPendingLR(struct vcpuIntData *vid)
    {
        unsigned int prio = 0xff;
        int p = -1;
        for (int i = 0; i < NUM_LR; i++) {
            if ((vid->LR[i].State & LR_PENDING) && (vid->LR[i].Priority < prio)) {
                p = i;
                prio = vid->LR[i].Priority;
            }
        }
        return p;
    }

    int findLRForVIRQ(struct vcpuIntData *vid, int virq, int vcpu)
    {
        for (int i = 0; i < NUM_LR; i++) {
            if (vid->LR[i].State &&
                vid->LR[i].VirtualID == virq &&
                vid->LR[i].CpuID == vcpu)
                return i;
        }
        return -1;
    }
};

#endif
