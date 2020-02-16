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
 */

/** @file
 * Implementiation of a GICv2m MSI shim.
 *
 * See gic_v2m.cc for an instantiation example.
 */

#ifndef __DEV_ARM_GIC_V2M_H__
#define __DEV_ARM_GIC_V2M_H__

#include "base/bitunion.hh"
#include "cpu/intr_control.hh"
#include "dev/arm/base_gic.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/Gicv2m.hh"
#include "params/Gicv2mFrame.hh"

/**
 * Ultimately this class should be embedded in the Gicv2m class, but
 * this confuses Python as 'Gicv2m::Frame' gets interpreted as 'Frame'
 * in namespace Gicv2m.
 */
class Gicv2mFrame : public SimObject
{
  public:
    const Addr          addr;
    const unsigned int  spi_base;
    const unsigned int  spi_len;

    typedef Gicv2mFrameParams Params;
    Gicv2mFrame(const Params *p) :
        SimObject(p), addr(p->addr), spi_base(p->spi_base), spi_len(p->spi_len)
    {}
};

class Gicv2m : public PioDevice
{
  private:
    static const int FRAME_SIZE         = 0x10000;

    static const int MSI_TYPER          = 0x0008;
    static const int MSI_SETSPI_NSR     = 0x0040;
    static const int PER_ID4            = 0x0fd0;

    /** Latency for an MMIO operation */
    const Tick pioDelay;

    /** A set of configured hardware frames */
    std::vector<Gicv2mFrame *> frames;

    /** Gic to which we fire interrupts */
    BaseGic *gic;

    /** Count of number of configured frames, as log2(frames) */
    unsigned int log2framenum;

  public:
    typedef Gicv2mParams Params;
    Gicv2m(const Params *p);

    /** @{ */
    /** Return the address ranges used by the Gicv2m
     * This is the set of frame addresses
     */
    virtual AddrRangeList getAddrRanges() const;

    /** A PIO read to the device
     */
    virtual Tick read(PacketPtr pkt);

    /** A PIO read to the device
     */
    virtual Tick write(PacketPtr pkt);
    /** @} */

  private:
    /** Determine which frame a PIO access lands in
     */
    int frameFromAddr(Addr a) const;
};

#endif //__DEV_ARM_GIC_V2M_H__
