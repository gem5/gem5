/*
 * Copyright (c) 2010, 2013, 2015-2016 ARM Limited
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
 *
 * Authors: Ali Saidi
 */


/** @file
 * Implementation of a PL390 GIC
 */

#ifndef __DEV_ARM_GIC_PL390_H__
#define __DEV_ARM_GIC_PL390_H__

#include <vector>

#include "base/addr_range.hh"
#include "base/bitunion.hh"
#include "cpu/intr_control.hh"
#include "dev/arm/base_gic.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/Pl390.hh"

class Pl390 : public BaseGic
{
  protected:
    // distributor memory addresses
    enum {
        GICD_CTLR          = 0x000, // control register
        GICD_TYPER         = 0x004, // controller type
        GICD_IIDR          = 0x008, // implementer id
        GICD_SGIR          = 0xf00, // software generated interrupt

        DIST_SIZE          = 0xfff
    };

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
    enum {
        GICC_CTLR  = 0x00, // CPU control register
        GICC_PMR   = 0x04, // Interrupt priority mask
        GICC_BPR   = 0x08, // binary point register
        GICC_IAR   = 0x0C, // interrupt ack register
        GICC_EOIR  = 0x10, // end of interrupt
        GICC_RPR   = 0x14, // runing priority
        GICC_HPPIR = 0x18, // highest pending interrupt
        GICC_ABPR  = 0x1c, // aliased binary point
        GICC_IIDR  = 0xfc, // cpu interface id register

        CPU_SIZE   = 0xff
    };

    static const int SGI_MAX = 16;  // Number of Software Gen Interrupts
    static const int PPI_MAX = 16;  // Number of Private Peripheral Interrupts

    /** Mask off SGI's when setting/clearing pending bits */
    static const int SGI_MASK = 0xFFFF0000;

    /** Mask for bits that config N:N mode in GICD_ICFGR's */
    static const int NN_CONFIG_MASK = 0x55555555;

    static const int CPU_MAX = 256;   // Max number of supported CPU interfaces
    static const int SPURIOUS_INT = 1023;
    static const int INT_BITS_MAX = 32;
    static const int INT_LINES_MAX = 1020;
    static const int GLOBAL_INT_LINES = INT_LINES_MAX - SGI_MAX - PPI_MAX;

    BitUnion32(SWI)
        Bitfield<3,0> sgi_id;
        Bitfield<23,16> cpu_list;
        Bitfield<25,24> list_type;
    EndBitUnion(SWI)

    BitUnion32(IAR)
        Bitfield<9,0> ack_id;
        Bitfield<12,10> cpu_id;
    EndBitUnion(IAR)

    /** Distributor address GIC listens at */
    Addr distAddr;

    /** CPU address GIC listens at */
    /** @todo is this one per cpu? */
    Addr cpuAddr;

    /** Latency for a distributor operation */
    Tick distPioDelay;

    /** Latency for a cpu operation */
    Tick cpuPioDelay;

    /** Latency for a interrupt to get to CPU */
    Tick intLatency;

    /** Gic enabled */
    bool enabled;

    /** Are gem5 extensions available? */
    const bool haveGem5Extensions;

    /** gem5 many-core extension enabled by driver */
    bool gem5ExtensionsEnabled;

    /** Number of itLines enabled */
    uint32_t itLines;

    /** Registers "banked for each connected processor" per ARM IHI0048B */
    struct BankedRegs : public Serializable {
        /** GICD_I{S,C}ENABLER0
         * interrupt enable bits for first 32 interrupts, 1b per interrupt */
        uint32_t intEnabled;

        /** GICD_I{S,C}PENDR0
         * interrupt pending bits for first 32 interrupts, 1b per interrupt */
        uint32_t pendingInt;

        /** GICD_I{S,C}ACTIVER0
         * interrupt active bits for first 32 interrupts, 1b per interrupt */
        uint32_t activeInt;

        /** GICD_IPRIORITYR{0..7}
         * interrupt priority for SGIs and PPIs */
        uint8_t intPriority[SGI_MAX + PPI_MAX];

        /** GICD_ITARGETSR{0..7}
         * 8b CPU target ID for each SGI and PPI */
        uint8_t cpuTarget[SGI_MAX + PPI_MAX];

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        BankedRegs() :
            intEnabled(0), pendingInt(0), activeInt(0),
            intPriority {0}, cpuTarget {0}
          {}
    };
    std::vector<BankedRegs*> bankedRegs;

    BankedRegs& getBankedRegs(ContextID);

    /** GICD_I{S,C}ENABLER{1..31}
     * interrupt enable bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t intEnabled[INT_BITS_MAX-1];

    uint32_t& getIntEnabled(ContextID ctx, uint32_t ix) {
        if (ix == 0) {
            return getBankedRegs(ctx).intEnabled;
        } else {
            return intEnabled[ix - 1];
        }
    }

    /** GICD_I{S,C}PENDR{1..31}
     * interrupt pending bits for global interrupts
     * 1b per interrupt, 32 bits per word, 31 words */
    uint32_t pendingInt[INT_BITS_MAX-1];

    uint32_t& getPendingInt(ContextID ctx, uint32_t ix) {
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
    uint32_t activeInt[INT_BITS_MAX-1];

    uint32_t& getActiveInt(ContextID ctx, uint32_t ix) {
        assert(ix < INT_BITS_MAX);
        if (ix == 0) {
            return getBankedRegs(ctx).activeInt;
        } else {
            return activeInt[ix - 1];
        }
    }

    /** read only running priority register, 1 per cpu*/
    uint32_t iccrpr[CPU_MAX];

    /** GICD_IPRIORITYR{8..255}
     * an 8 bit priority (lower is higher priority) for each
     * of the global (not replicated per CPU) interrupts.
     */
    uint8_t intPriority[GLOBAL_INT_LINES];

    uint8_t& getIntPriority(ContextID ctx, uint32_t ix) {
        assert(ix < INT_LINES_MAX);
        if (ix < SGI_MAX + PPI_MAX) {
            return getBankedRegs(ctx).intPriority[ix];
        } else {
            return intPriority[ix - (SGI_MAX + PPI_MAX)];
        }
    }

    /** GICD_ITARGETSR{8..255}
     * an 8 bit cpu target id for each global interrupt.
     */
    uint8_t cpuTarget[GLOBAL_INT_LINES];

    uint8_t& getCpuTarget(ContextID ctx, uint32_t ix) {
        assert(ix < INT_LINES_MAX);
        if (ix < SGI_MAX + PPI_MAX) {
            return getBankedRegs(ctx).cpuTarget[ix];
        } else {
            return cpuTarget[ix - (SGI_MAX + PPI_MAX)];
        }
    }

    /** 2 bit per interrupt signaling if it's level or edge sensitive
     * and if it is 1:N or N:N */
    uint32_t intConfig[INT_BITS_MAX*2];

    /** CPU enabled */
    bool cpuEnabled[CPU_MAX];

    /** CPU priority */
    uint8_t cpuPriority[CPU_MAX];

    /** Binary point registers */
    uint8_t cpuBpr[CPU_MAX];

    /** highest interrupt that is interrupting CPU */
    uint32_t cpuHighestInt[CPU_MAX];

    /** One bit per cpu per software interrupt that is pending for each possible
     * sgi source. Indexed by SGI number. Each byte in generating cpu id and
     * bits in position is destination id. e.g. 0x4 = CPU 0 generated interrupt
     * for CPU 2. */
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

    /** IRQ Enable Used for debug */
    bool irqEnable;

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

    int intNumToWord(int num) const { return num >> 5; }
    int intNumToBit(int num) const { return num % 32; }

    /** Post an interrupt to a CPU
     */
    void postInt(uint32_t cpu, Tick when);

    /** Event definition to post interrupt to CPU after a delay
    */
    class PostIntEvent : public Event
    {
      private:
        uint32_t cpu;
        Platform *platform;
      public:
        PostIntEvent( uint32_t c, Platform* p)
            : cpu(c), platform(p)
        { }
        void process() { platform->intrctrl->post(cpu, ArmISA::INT_IRQ, 0);}
        const char *description() const { return "Post Interrupt to CPU"; }
    };
    PostIntEvent *postIntEvent[CPU_MAX];

  public:
    typedef Pl390Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    Pl390(const Params *p);

    /** @{ */
    /** Return the address ranges used by the Gic
     * This is the distributor address + all cpu addresses
     */
    AddrRangeList getAddrRanges() const override;

    /** A PIO read to the device, immediately split up into
     * readDistributor() or readCpu()
     */
    Tick read(PacketPtr pkt) override;

    /** A PIO read to the device, immediately split up into
     * writeDistributor() or writeCpu()
     */
    Tick write(PacketPtr pkt) override;
    /** @} */

    /** @{ */
    /** Post an interrupt from a device that is connected to the Gic.
     * Depending on the configuration, the gic will pass this interrupt
     * on through to a CPU.
     * @param number number of interrupt to send */
    void sendInt(uint32_t number) override;

    /** Interface call for private peripheral interrupts  */
    void sendPPInt(uint32_t num, uint32_t cpu) override;

    /** Clear an interrupt from a device that is connected to the Gic
     * Depending on the configuration, the gic may de-assert it's cpu line
     * @param number number of interrupt to send */
    void clearInt(uint32_t number) override;

    /** Clear a (level-sensitive) PPI */
    void clearPPInt(uint32_t num, uint32_t cpu) override;
    /** @} */

    /** @{ */
    /* Various functions fer testing and debugging */
    void driveSPI(uint32_t spi);
    void driveLegIRQ(bool state);
    void driveLegFIQ(bool state);
    void driveIrqEn(bool state);
    /** @} */

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    /** Handle a read to the distributor poriton of the GIC
     * @param pkt packet to respond to
     */
    Tick readDistributor(PacketPtr pkt);

    /** Handle a read to the cpu poriton of the GIC
     * @param pkt packet to respond to
     */
    Tick readCpu(PacketPtr pkt);

    /** Handle a write to the distributor poriton of the GIC
     * @param pkt packet to respond to
     */
    Tick writeDistributor(PacketPtr pkt);

    /** Handle a write to the cpu poriton of the GIC
     * @param pkt packet to respond to
     */
    Tick writeCpu(PacketPtr pkt);
};

#endif //__DEV_ARM_GIC_H__
