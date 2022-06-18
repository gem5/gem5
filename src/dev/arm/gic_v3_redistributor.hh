/*
 * Copyright (c) 2019-2022 Arm Limited
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

#ifndef __DEV_ARM_GICV3_REDISTRIBUTOR_H__
#define __DEV_ARM_GICV3_REDISTRIBUTOR_H__

#include "base/addr_range.hh"
#include "dev/arm/gic_v3.hh"
#include "sim/serialize.hh"

namespace gem5
{

class Gicv3CPUInterface;
class Gicv3Distributor;
class Gicv3Its;

class Gicv3Redistributor : public Serializable
{
  private:

    friend class Gicv3CPUInterface;
    friend class Gicv3Distributor;
    friend class Gicv3Its;

  protected:

    Gicv3 * gic;
    Gicv3Distributor * distributor;
    Gicv3CPUInterface * cpuInterface;
    uint32_t cpuId;
    PortProxy * memProxy;

    /*
     * GICv3 defines 2 contiguous 64KB frames for each redistributor.
     * Order of frames must be RD_base, SGI_base.
     */
    static const uint32_t RD_base  = 0x0;
    static const uint32_t SGI_base = 0x10000;

    enum
    {
        // Control Register
        GICR_CTLR  = RD_base + 0x0000,
        // Implementer Identification Register
        GICR_IIDR  = RD_base + 0x0004,
        // Type Register
        GICR_TYPER = RD_base + 0x0008,
        // Wake Register
        GICR_WAKER = RD_base + 0x0014,
        // Peripheral ID0 Register
        GICR_PIDR0 = RD_base + 0xffe0,
        // Peripheral ID1 Register
        GICR_PIDR1 = RD_base + 0xffe4,
        // Peripheral ID2 Register
        GICR_PIDR2 = RD_base + 0xffe8,
        // Peripheral ID3 Register
        GICR_PIDR3 = RD_base + 0xffec,
        // Peripheral ID4 Register
        GICR_PIDR4 = RD_base + 0xffd0,
        // Peripheral ID5 Register
        GICR_PIDR5 = RD_base + 0xffd4,
        // Peripheral ID6 Register
        GICR_PIDR6 = RD_base + 0xffd8,
        // Peripheral ID7 Register
        GICR_PIDR7 = RD_base + 0xffdc,
    };

    static const uint32_t GICR_WAKER_ProcessorSleep = 1 << 1;
    static const uint32_t GICR_WAKER_ChildrenAsleep = 1 << 2;

    bool peInLowPowerState;

    enum
    {
        // Interrupt Group Register 0
        GICR_IGROUPR0   = SGI_base + 0x0080,
        // Interrupt Set-Enable Register 0
        GICR_ISENABLER0 = SGI_base + 0x0100,
        // Interrupt Clear-Enable Register 0
        GICR_ICENABLER0 = SGI_base + 0x0180,
        // Interrupt Set-Pending Register 0
        GICR_ISPENDR0   = SGI_base + 0x0200,
        // Interrupt Clear-Pending Register 0
        GICR_ICPENDR0   = SGI_base + 0x0280,
        // Interrupt Set-Active Register 0
        GICR_ISACTIVER0 = SGI_base + 0x0300,
        // Interrupt Clear-Active Register 0
        GICR_ICACTIVER0 = SGI_base + 0x0380,
        // SGI Configuration Register
        GICR_ICFGR0     = SGI_base + 0x0c00,
        // PPI Configuration Register
        GICR_ICFGR1     = SGI_base + 0x0c04,
        // Interrupt Group Modifier Register 0
        GICR_IGRPMODR0  = SGI_base + 0x0d00,
        // Non-secure Access Control Register
        GICR_NSACR      = SGI_base + 0x0e00,
    };

    // Interrupt Priority Registers
    static const AddrRange GICR_IPRIORITYR;

    // GIC physical LPI Redistributor register
    enum
    {
        // Set LPI Pending Register
        GICR_SETLPIR = RD_base + 0x0040,
        // Clear LPI Pending Register
        GICR_CLRLPIR = RD_base + 0x0048,
        //Redistributor Properties Base Address Register
        GICR_PROPBASER = RD_base + 0x0070,
        // Redistributor LPI Pending Table Base Address Register
        GICR_PENDBASER = RD_base + 0x0078,
        // Redistributor Invalidate LPI Register
        GICR_INVLPIR = RD_base + 0x00A0,
        // Redistributor Invalidate All Register
        GICR_INVALLR = RD_base + 0x00B0,
        // Redistributor Synchronize Register
        GICR_SYNCR = RD_base + 0x00C0,
    };

    std::vector <uint8_t> irqGroup;
    std::vector <bool> irqEnabled;
    std::vector <bool> irqPending;
    std::vector <bool> irqPendingIspendr;
    std::vector <bool> irqActive;
    std::vector <uint8_t> irqPriority;
    std::vector <Gicv3::IntTriggerType> irqConfig;
    std::vector <uint8_t> irqGrpmod;
    std::vector <uint8_t> irqNsacr;

    bool DPG1S;
    bool DPG1NS;
    bool DPG0;
    bool EnableLPIs;

    Addr lpiConfigurationTablePtr;
    uint8_t lpiIDBits;
    Addr lpiPendingTablePtr;

    BitUnion8(LPIConfigurationTableEntry)
        Bitfield<7, 2> priority;
        Bitfield<1> res1;
        Bitfield<0> enable;
    EndBitUnion(LPIConfigurationTableEntry)

    static const uint32_t GICR_CTLR_ENABLE_LPIS = 1 << 0;
    static const uint32_t GICR_CTLR_DPG0   = 1 << 24;
    static const uint32_t GICR_CTLR_DPG1NS = 1 << 25;
    static const uint32_t GICR_CTLR_DPG1S  = 1 << 26;

  public:

    /*
     * GICv3 defines only 2 64K consecutive frames for the redistributor
     * (RD_base and SGI_base) but we are using 2 extra 64K stride frames
     * to match GICv4 that defines 4 64K consecutive frames for them.
     * Note this must match with DTB/DTS GIC node definition and boot
     * loader code.
     */
    const uint32_t addrRangeSize;

    static const uint32_t SMALLEST_LPI_ID = 8192;


    void activateIRQ(uint32_t int_id);
    bool canBeSelectedFor1toNInterrupt(Gicv3::GroupId group) const;
    void deactivateIRQ(uint32_t int_id);

    inline Gicv3CPUInterface *
    getCPUInterface() const
    {
        return cpuInterface;
    }

    uint32_t
    processorNumber() const
    {
        return cpuId;
    }

    Gicv3::GroupId getIntGroup(int int_id) const;
    Gicv3::IntStatus intStatus(uint32_t int_id) const;
    uint8_t readEntryLPI(uint32_t intid);
    void writeEntryLPI(uint32_t intid, uint8_t lpi_entry);
    bool isPendingLPI(uint32_t intid);
    void setClrLPI(uint64_t data, bool set);
    void sendSGI(uint32_t int_id, Gicv3::GroupId group, bool ns);
    void serialize(CheckpointOut & cp) const override;
    void unserialize(CheckpointIn & cp) override;
    void update();
    void updateDistributor();

  protected:

    bool isLevelSensitive(uint32_t int_id) const
    {
        return irqConfig[int_id] == Gicv3::INT_LEVEL_SENSITIVE;
    }

    /**
     * This helper is used to check if an interrupt should be treated as
     * edge triggered in the following scenarios:
     *
     * a) While activating the interrupt
     * b) While clearing an interrupt via ICPENDR
     *
     * In fact, in these two situations, a level sensitive interrupt
     * which had been made pending via a write to ISPENDR, will be
     * treated as it if was edge triggered.
     */
    bool treatAsEdgeTriggered(uint32_t int_id) const
    {
        return !isLevelSensitive(int_id) || irqPendingIspendr[int_id];
    }

  public:

    Gicv3Redistributor(Gicv3 * gic, uint32_t cpu_id);
    uint32_t getAffinity() const;
    void init();
    uint64_t read(Addr addr, size_t size, bool is_secure_access);
    void sendPPInt(uint32_t int_id);
    void clearPPInt(uint32_t int_id);
    void write(Addr addr, uint64_t data, size_t size, bool is_secure_access);

    void copy(Gicv3Registers *from, Gicv3Registers *to);
};

} // namespace gem5

#endif //__DEV_ARM_GICV3_REDISTRIBUTOR_H__
