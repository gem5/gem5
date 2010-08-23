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
 * Implementiation of a PL390 GIC
 */

#ifndef __DEV_ARM_GIC_H__
#define __DEV_ARM_GIC_H__

#include "base/bitunion.hh"
#include "base/range.hh"
#include "dev/io_device.hh"
#include "params/Gic.hh"

/** @todo this code only assumes one processor for now. Low word
 * of intEnabled and pendingInt need to be replicated per CPU.
 * bottom 31 interrupts (7 words) need to be replicated for
 * for interrupt priority register, processor target registers
 * interrupt config registers  */

class Gic : public PioDevice
{
  protected:
    // distributor memory addresses
    static const int ICDDCR     = 0x000; // control register
    static const int ICDICTR    = 0x004; // controller type
    static const int ICDIIDR    = 0x008; // implementer id
    static const int ICDISER_ST = 0x100; // interrupt set enable
    static const int ICDISER_ED = 0x17c;
    static const int ICDICER_ST = 0x180; // interrupt clear enable
    static const int ICDICER_ED = 0x1fc;
    static const int ICDISPR_ST = 0x200; // set pending interrupt
    static const int ICDISPR_ED = 0x27c;
    static const int ICDICPR_ST = 0x280; // clear pending interrupt
    static const int ICDICPR_ED = 0x2fc;
    static const int ICDABR_ST  = 0x300; // active bit registers
    static const int ICDABR_ED  = 0x37c;
    static const int ICDIPR_ST  = 0x400; // interrupt priority registers
    static const int ICDIPR_ED  = 0x7f8;
    static const int ICDIPTR_ST = 0x800; // processor target registers
    static const int ICDIPTR_ED = 0xbf8;
    static const int ICDICFR_ST = 0xc00; // interrupt config registers
    static const int ICDICFR_ED = 0xcfc;
    static const int ICDSGIR    = 0xf00; // software generated interrupt
    static const int DIST_SIZE  = 0xfff;

    // cpu memory addressesa
    static const int ICCICR  = 0x00; // CPU control register
    static const int ICCPMR  = 0x04; // Interrupt priority mask
    static const int ICCBPR  = 0x08; // binary point register
    static const int ICCIAR  = 0x0C; // interrupt ack register
    static const int ICCEOIR = 0x10; // end of interrupt
    static const int ICCRPR  = 0x14; // runing priority
    static const int ICCHPIR = 0x18; // highest pending interrupt
    static const int ICCABPR = 0x1c; // aliased binary point
    static const int ICCIIDR = 0xfc; // cpu interface id register
    static const int CPU_SIZE  = 0xff;

    static const int SPURIOUS_INT = 1023;

    BitUnion32(SWI)
        Bitfield<3,0> sgi_id;
        Bitfield<23,16> cpu_list;
        Bitfield<25,24> list_type;
    EndBitUnion(SWI)

    /** Distributor address GIC listens at */
    Addr distAddr;

    /** CPU address GIC listens at */
    /** @todo is this one per cpu? */
    Addr cpuAddr;

    /** Latency for a distributor operation */
    Tick distPioDelay;

    /** Latency for a cpu operation */
    Tick cpuPioDelay;

    /** Gic enabled */
    bool enabled;

    /** Number of itLines enabled */
    uint32_t itLines;

    uint32_t itLinesLog2;

    /** interrupt enable bits for all possible 1020 interupts.
     * one bit per interrupt, 32 bit per word = 32 words */
    uint32_t intEnabled[32];

    /** interrupt pending bits for all possible 1020 interupts.
     * one bit per interrupt, 32 bit per word = 32 words */
    uint32_t pendingInt[32];

    /** interrupt active bits for all possible 1020 interupts.
     * one bit per interrupt, 32 bit per word = 32 words */
    uint32_t activeInt[32];

    /** an 8 bit priority (lower is higher priority) for each
     * of the 1020 possible supported interrupts.
     */
    uint8_t intPriority[1020];

    /** an 8 bit cpu target id for each shared peripheral interrupt
     * of the 1020 possible supported interrupts.
     */
    uint8_t cpuTarget[1020];

    /** 2 bit per interrupt signaling if it's level or edge sensitive
     * and if it is 1:N or N:N */
    uint32_t intConfig[64];

    /** CPU enabled */
    bool cpuEnabled[8];

    /** CPU priority */
    uint8_t cpuPriority[8];

    /** Binary point registers */
    uint8_t cpuBpr[8];

    /** highest interrupt that is interrupting CPU */
    uint32_t cpuHighestInt[8];

    /** software generated interrupt
     * @param data data to decode that indicates which cpus to interrupt
     */
    void softInt(SWI swi);

    /** See if some processor interrupt flags need to be enabled/disabled
     * @param hint which set of interrupts needs to be checked
     */
    void updateIntState(int hint);

    int intNumToWord(int num) const { return num >> 5; }
    int intNumToBit(int num) const { return num % 32; }

  public:
   typedef GicParams Params;
   const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    Gic(const Params *p);

    /** Return the address ranges used by the Gic
     * This is the distributor address + all cpu addresses
     */
    virtual void addressRanges(AddrRangeList &range_list);

    /** A PIO read to the device, immediately split up into
     * readDistributor() or readCpu()
     */
    virtual Tick read(PacketPtr pkt);

    /** A PIO read to the device, immediately split up into
     * writeDistributor() or writeCpu()
     */
    virtual Tick write(PacketPtr pkt);

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

    /** Post an interrupt from a device that is connected to the Gic.
     * Depending on the configuration, the gic will pass this interrupt
     * on through to a CPU.
     * @param number number of interrupt to send */
    void sendInt(uint32_t number);

    /** Clear an interrupt from a device that is connected to the Gic
     * Depending on the configuration, the gic may de-assert it's cpu line
     * @param number number of interrupt to send */
    void clearInt(uint32_t number);


    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif //__DEV_ARM_GIC_H__
