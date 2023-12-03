/*
 * Copyright (c) 2010, 2013, 2015-2022 Arm Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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

/** @file
 * Implementation of a GICv2
 */

#ifndef __DEV_ARM_GICV2_H__
#define __DEV_ARM_GICV2_H__

#include <vector>

#include "arch/arm/interrupts.hh"
#include "base/addr_range.hh"
#include "base/bitunion.hh"
#include "dev/arm/base_gic.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/GicV2.hh"

namespace gem5
{

class GicV2Registers
{
  public:
    virtual uint32_t readDistributor(ContextID ctx, Addr daddr) = 0;
    virtual uint32_t readCpu(ContextID ctx, Addr daddr) = 0;

    virtual void writeDistributor(ContextID ctx, Addr daddr,
                                  uint32_t data) = 0;
    virtual void writeCpu(ContextID ctx, Addr daddr, uint32_t data) = 0;

  protected:
    static void copyDistRegister(GicV2Registers *from, GicV2Registers *to,
                                 ContextID ctx, Addr daddr);
    static void copyCpuRegister(GicV2Registers *from, GicV2Registers *to,
                                ContextID ctx, Addr daddr);
    static void copyBankedDistRange(System *sys, GicV2Registers *from,
                                    GicV2Registers *to, Addr daddr,
                                    size_t size);
    static void clearBankedDistRange(System *sys, GicV2Registers *to,
                                     Addr daddr, size_t size);
    static void copyDistRange(GicV2Registers *from, GicV2Registers *to,
                              Addr daddr, size_t size);
    static void clearDistRange(GicV2Registers *to, Addr daddr, size_t size);
};

class GicV2 : public BaseGic, public GicV2Registers
{
  protected:
    // distributor memory addresses
    enum
    {
        GICD_CTLR = 0x000,  // control register
        GICD_TYPER = 0x004, // controller type
        GICD_IIDR = 0x008,  // implementer id
        GICD_SGIR = 0xf00,  // software generated interrupt
        GICD_PIDR0 = 0xfe0, // distributor peripheral ID0
        GICD_PIDR1 = 0xfe4, // distributor peripheral ID1
        GICD_PIDR2 = 0xfe8, // distributor peripheral ID2
        GICD_PIDR3 = 0xfec, // distributor peripheral ID3

        DIST_SIZE = 0x1000,
    };

    const uint32_t gicdPIDR;
    const uint32_t gicdIIDR;
    const uint32_t giccIIDR;

    static const AddrRange GICD_IGROUPR;    // interrupt group (unimplemented)
    static const AddrRange GICD_ISENABLER;  // interrupt set enable
    static const AddrRange GICD_ICENABLER;  // interrupt clear enable
    static const AddrRange GICD_ISPENDR;    // set pending interrupt
    static const AddrRange GICD_ICPENDR;    // clear pending interrupt
    static const AddrRange GICD_ISACTIVER;  // active bit registers
    static const AddrRange GICD_ICACTIVER;  // clear bit registers
    static const AddrRange GICD_IPRIORITYR; // interrupt priority registers
    static const AddrRange GICD_ITARGETSR;  // processor target registers
    static const AddrRange GICD_ICFGR;      // interrupt config registers

    // cpu memory addresses
    enum
    {
        GICC_CTLR = 0x00,  // CPU control register
        GICC_PMR = 0x04,   // Interrupt priority mask
        GICC_BPR = 0x08,   // binary point register
        GICC_IAR = 0x0C,   // interrupt ack register
        GICC_EOIR = 0x10,  // end of interrupt
        GICC_RPR = 0x14,   // running priority
        GICC_HPPIR = 0x18, // highest pending interrupt
        GICC_ABPR = 0x1c,  // aliased binary point
        GICC_APR0 = 0xd0,  // active priority register 0
        GICC_APR1 = 0xd4,  // active priority register 1
        GICC_APR2 = 0xd8,  // active priority register 2
        GICC_APR3 = 0xdc,  // active priority register 3
        GICC_IIDR = 0xfc,  // cpu interface id register
        GICC_DIR = 0x1000, // deactive interrupt register
    };

    static const int SGI_MAX = 16; // Number of Software Gen Interrupts
    static const int PPI_MAX = 16; // Number of Private Peripheral Interrupts

    /** Mask off SGI's when setting/clearing pending bits */
    static const int SGI_MASK = 0xFFFF0000;

    /** Mask for bits that config N:N mode in GICD_ICFGR's */
    static const int NN_CONFIG_MASK = 0x55555555;

    static const int CPU_MAX = 256; // Max number of supported CPU interfaces
    static const int SPURIOUS_INT = 1023;
    static const int INT_BITS_MAX = 32;
    static const int INT_LINES_MAX = 1020;
    static const int GLOBAL_INT_LINES = INT_LINES_MAX - SGI_MAX - PPI_MAX;

    /** minimum value for Binary Point Register ("IMPLEMENTATION DEFINED");
        chosen for consistency with Linux's in-kernel KVM GIC model */
    static const int GICC_BPR_MINIMUM = 2;

    BitUnion32(SWI)
        Bitfield<3, 0> sgi_id;
        Bitfield<23, 16> cpu_list;
        Bitfield<25, 24> list_type;
    EndBitUnion(SWI)

    BitUnion32(IAR)
        Bitfield<9, 0> ack_id;
        Bitfield<12, 10> cpu_id;
    EndBitUnion(IAR)

    BitUnion32(CTLR)
        Bitfield<3> fiqEn;
        Bitfield<1> enableGrp1;
        Bitfield<0> enableGrp0;
    EndBitUnion(CTLR)

  protected: /* Params */
    /** Address range for the distributor interface */
    const AddrRange distRange;

    /** Address range for the CPU interfaces */
    const AddrRange cpuRange;

    /** All address ranges used by this GIC */
    const AddrRangeList addrRanges;

    /** Latency for a distributor operation */
    const Tick distPioDelay;

    /** Latency for a cpu operation */
    const Tick cpuPioDelay;

    /** Latency for a interrupt to get to CPU */
    const Tick intLatency;

  protected:
    /** Gic enabled */
    bool enabled;

    /** Are gem5 extensions available? */
    const bool haveGem5Extensions;

    /** gem5 many-core extension enabled by driver */
    bool gem5ExtensionsEnabled;

    /** Number of itLines enabled */
    uint32_t itLines;

    /** Registers "banked for each connected processor" per ARM IHI0048B */
    struct BankedRegs : public Serializable
    {
        /** GICD_I{S,C}ENABLER0
         * interrupt enable bits for first 32 interrupts, 1b per interrupt */
        uint32_t intEnabled;

        /** GICD_I{S,C}PENDR0
         * interrupt pending bits for first 32 interrupts, 1b per interrupt */
        uint32_t pendingInt;

        /** GICD_I{S,C}ACTIVER0
         * interrupt active bits for first 32 interrupts, 1b per interrupt */
        uint32_t activeInt;

        /** GICD_IGROUPR0
         * interrupt group bits for first 32 interrupts, 1b per interrupt */
        uint32_t intGroup;

        /** GICD_ICFGR0, GICD_ICFGR1
         * interrupt config bits for first 32 interrupts, 2b per interrupt */
        uint32_t intConfig[2];

        /** GICD_IPRIORITYR{0..7}
         * interrupt priority for SGIs and PPIs */
        uint8_t intPriority[SGI_MAX + PPI_MAX];

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        BankedRegs()
            : intEnabled(0),
              pendingInt(0),
              activeInt(0),
              intGroup(0),
              intConfig{ 0 },
              intPriority{ 0 }
        {}
    };

    std::vector<BankedRegs *> bankedRegs;

    BankedRegs &getBankedRegs(ContextID);

    /** GICD_I{S,C}ENABLER{1..31}
     * interrupt enable bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t intEnabled[INT_BITS_MAX - 1];

    uint32_t &
    getIntEnabled(ContextID ctx, uint32_t ix)
    {
        if (ix == 0) {
            return getBankedRegs(ctx).intEnabled;
        } else {
            return intEnabled[ix - 1];
        }
    }

    /** GICD_I{S,C}PENDR{1..31}
     * interrupt pending bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t pendingInt[INT_BITS_MAX - 1];

    uint32_t &
    getPendingInt(ContextID ctx, uint32_t ix)
    {
        assert(ix < INT_BITS_MAX);
        if (ix == 0) {
            return getBankedRegs(ctx).pendingInt;
        } else {
            return pendingInt[ix - 1];
        }
    }

    /** GICD_I{S,C}ACTIVER{1..31}
     * interrupt active bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t activeInt[INT_BITS_MAX - 1];

    uint32_t &
    getActiveInt(ContextID ctx, uint32_t ix)
    {
        assert(ix < INT_BITS_MAX);
        if (ix == 0) {
            return getBankedRegs(ctx).activeInt;
        } else {
            return activeInt[ix - 1];
        }
    }

    /** GICD_IGROUPR{1..31}
     * interrupt group bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t intGroup[INT_BITS_MAX - 1];

    uint32_t &
    getIntGroup(ContextID ctx, uint32_t ix)
    {
        assert(ix < INT_BITS_MAX);
        if (ix == 0) {
            return getBankedRegs(ctx).intGroup;
        } else {
            return intGroup[ix - 1];
        }
    }

    /** read only running priority register, 1 per cpu*/
    uint32_t iccrpr[CPU_MAX];

    /** GICD_IPRIORITYR{8..255}
     * an 8 bit priority (lower is higher priority) for each
     * of the global (not replicated per CPU) interrupts.
     */
    uint8_t intPriority[GLOBAL_INT_LINES];

    uint8_t &
    getIntPriority(ContextID ctx, uint32_t ix)
    {
        assert(ix < INT_LINES_MAX);
        if (ix < SGI_MAX + PPI_MAX) {
            return getBankedRegs(ctx).intPriority[ix];
        } else {
            return intPriority[ix - (SGI_MAX + PPI_MAX)];
        }
    }

    /**
     * GICD_ICFGR{2...63}
     * 2 bit per interrupt signaling if it's level or edge sensitive
     * and if it is 1:N or N:N */
    uint32_t intConfig[INT_BITS_MAX * 2 - 2];

    /**
     * Reads the GICD_ICFGRn register.
     * @param ctx context id (PE specific)
     * @param ix interrupt word index
     * @returns the interrupt config word
     */
    uint32_t &
    getIntConfig(ContextID ctx, uint32_t ix)
    {
        assert(ix < INT_BITS_MAX * 2);
        if (ix < 2) {
            /** SGIs and PPIs **/
            return getBankedRegs(ctx).intConfig[ix];
        } else {
            return intConfig[ix - 2];
        }
    }

    /** GICD_ITARGETSR{8..255}
     * an 8 bit cpu target id for each global interrupt.
     */
    uint8_t cpuTarget[GLOBAL_INT_LINES];

    uint8_t
    getCpuTarget(ContextID ctx, uint32_t ix) const
    {
        assert(ctx < sys->threads.numRunning());
        assert(ix < INT_LINES_MAX);
        if (ix < SGI_MAX + PPI_MAX) {
            // "GICD_ITARGETSR0 to GICD_ITARGETSR7 are read-only, and each
            // field returns a value that corresponds only to the processor
            // reading the register."
            uint32_t ctx_mask;
            if (gem5ExtensionsEnabled) {
                ctx_mask = ctx;
            } else {
                fatal_if(
                    ctx >= 8,
                    "%s requires the gem5_extensions parameter to support "
                    "more than 8 cores\n",
                    name());
                // convert the CPU id number into a bit mask
                ctx_mask = 1 << ctx;
            }
            return ctx_mask;
        } else {
            return cpuTarget[ix - 32];
        }
    }

    bool
    isLevelSensitive(ContextID ctx, uint32_t int_num)
    {
        if (int_num == SPURIOUS_INT) {
            return false;
        } else {
            const auto ix = intNumToWord(int_num * 2);
            const uint8_t cfg_hi = intNumToBit(int_num * 2) + 1;
            return bits(getIntConfig(ctx, ix), cfg_hi) == 0;
        }
    }

    bool
    isGroup0(ContextID ctx, uint32_t int_num)
    {
        const uint32_t group_reg = getIntGroup(ctx, intNumToWord(int_num));
        return !bits(group_reg, intNumToBit(int_num));
    }

    /**
     * This method checks if an interrupt ID must be signaled or has been
     * signaled as a FIQ to the cpu. It does that by reading:
     *
     * 1) GICD_IGROUPR: controls if the interrupt is part of group0 or
     * group1. Only group0 interrupts can be signaled as FIQs.
     *
     * 2) GICC_CTLR.FIQEn: controls whether the CPU interface signals Group 0
     * interrupts to a target processor using the FIQ or the IRQ signal
     */
    bool
    isFiq(ContextID ctx, uint32_t int_num)
    {
        const bool is_group0 = isGroup0(ctx, int_num);
        const bool use_fiq = cpuControl[ctx].fiqEn;

        if (is_group0 && use_fiq) {
            return true;
        } else {
            return false;
        }
    }

    /** CPU enabled:
     * Checks if GICC_CTLR.EnableGrp0 or EnableGrp1 are set
     */
    bool
    cpuEnabled(ContextID ctx) const
    {
        return cpuControl[ctx].enableGrp0 || cpuControl[ctx].enableGrp1;
    }

    /** GICC_CTLR:
     * CPU interface control register
     */
    CTLR cpuControl[CPU_MAX];

    /** CPU priority */
    uint8_t cpuPriority[CPU_MAX];
    uint8_t getCpuPriority(unsigned cpu); // BPR-adjusted priority value

    /** Binary point registers */
    uint8_t cpuBpr[CPU_MAX];

    /** highest interrupt that is interrupting CPU */
    uint32_t cpuHighestInt[CPU_MAX];

    /** One bit per cpu per software interrupt that is pending for each
     * possible sgi source. Indexed by SGI number. Each byte in generating cpu
     * id and bits in position is destination id. e.g. 0x4 = CPU 0 generated
     * interrupt for CPU 2. */
    uint64_t cpuSgiPending[SGI_MAX];
    uint64_t cpuSgiActive[SGI_MAX];

    /** SGI pending arrays for gem5 GIC extension mode, which instead keeps
     * 16 SGI pending bits for each of the (large number of) CPUs.
     */
    uint32_t cpuSgiPendingExt[CPU_MAX];
    uint32_t cpuSgiActiveExt[CPU_MAX];

    /** One bit per private peripheral interrupt. Only upper 16 bits
     * will be used since PPI interrupts are numberred from 16 to 32 */
    uint32_t cpuPpiPending[CPU_MAX];
    uint32_t cpuPpiActive[CPU_MAX];

    /** software generated interrupt
     * @param data data to decode that indicates which cpus to interrupt
     */
    void softInt(ContextID ctx, SWI swi);

    /** See if some processor interrupt flags need to be enabled/disabled
     * @param hint which set of interrupts needs to be checked
     */
    void updateIntState(int hint);

    /** Update the register that records priority of the highest priority
     *  active interrupt*/
    void updateRunPri();

    /** generate a bit mask to check cpuSgi for an interrupt. */
    uint64_t genSwiMask(int cpu);

    int
    intNumToWord(int num) const
    {
        return num >> 5;
    }

    int
    intNumToBit(int num) const
    {
        return num % 32;
    }

    /** Clears a cpu IRQ or FIQ signal */
    void clearInt(ContextID ctx, uint32_t int_num);

    /**
     * Post an interrupt to a CPU with a delay
     */
    void postInt(uint32_t cpu, Tick when);
    void postFiq(uint32_t cpu, Tick when);

    /**
     * Deliver a delayed interrupt to the target CPU
     */
    void postDelayedInt(uint32_t cpu);
    void postDelayedFiq(uint32_t cpu);

    EventFunctionWrapper *postIntEvent[CPU_MAX];
    EventFunctionWrapper *postFiqEvent[CPU_MAX];
    int pendingDelayedInterrupts;

  public:
    using Params = GicV2Params;
    GicV2(const Params &p);
    ~GicV2();

    DrainState drain() override;
    void drainResume() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: /* PioDevice */
    AddrRangeList
    getAddrRanges() const override
    {
        return addrRanges;
    }

    /** A PIO read to the device, immediately split up into
     * readDistributor() or readCpu()
     */
    Tick read(PacketPtr pkt) override;

    /** A PIO read to the device, immediately split up into
     * writeDistributor() or writeCpu()
     */
    Tick write(PacketPtr pkt) override;

  public: /* BaseGic */
    void sendInt(uint32_t number) override;
    void clearInt(uint32_t number) override;

    void sendPPInt(uint32_t num, uint32_t cpu) override;
    void clearPPInt(uint32_t num, uint32_t cpu) override;

    bool supportsVersion(GicVersion version) override;

  protected: /** GIC state transfer */
    void copyGicState(GicV2Registers *from, GicV2Registers *to);

    /** Handle a read to the distributor portion of the GIC
     * @param pkt packet to respond to
     */
    Tick readDistributor(PacketPtr pkt);
    uint32_t readDistributor(ContextID ctx, Addr daddr, size_t resp_sz);

    uint32_t
    readDistributor(ContextID ctx, Addr daddr) override
    {
        return readDistributor(ctx, daddr, 4);
    }

    /** Handle a read to the cpu portion of the GIC
     * @param pkt packet to respond to
     */
    Tick readCpu(PacketPtr pkt);
    uint32_t readCpu(ContextID ctx, Addr daddr) override;

    /** Handle a write to the distributor portion of the GIC
     * @param pkt packet to respond to
     */
    Tick writeDistributor(PacketPtr pkt);
    void writeDistributor(ContextID ctx, Addr daddr, uint32_t data,
                          size_t data_sz);

    void
    writeDistributor(ContextID ctx, Addr daddr, uint32_t data) override
    {
        return writeDistributor(ctx, daddr, data, 4);
    }

    /** Handle a write to the cpu portion of the GIC
     * @param pkt packet to respond to
     */
    Tick writeCpu(PacketPtr pkt);
    void writeCpu(ContextID ctx, Addr daddr, uint32_t data) override;
};

} // namespace gem5

#endif //__DEV_ARM_GIC_H__
