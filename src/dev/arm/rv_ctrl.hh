/*
 * Copyright (c) 2010,2013 ARM Limited
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
 * Authors: Ali Saidi
 */

#ifndef __DEV_ARM_RV_HH__
#define __DEV_ARM_RV_HH__

#include "base/bitunion.hh"
#include "dev/io_device.hh"
#include "params/RealViewCtrl.hh"

/** @file
 * This implements the simple real view registers on a PBXA9
 */

class RealViewCtrl : public BasicPioDevice
{
  protected:
    enum {
        IdReg      = 0x00,
        SwReg      = 0x04,
        Led        = 0x08,
        Osc0       = 0x0C,
        Osc1       = 0x10,
        Osc2       = 0x14,
        Osc3       = 0x18,
        Osc4       = 0x1C,
        Lock       = 0x20,
        Clock100   = 0x24,
        CfgData1   = 0x28,
        CfgData2   = 0x2C,
        Flags      = 0x30,
        FlagsClr   = 0x34,
        NvFlags    = 0x38,
        NvFlagsClr = 0x3C,
        ResetCtl   = 0x40,
        PciCtl     = 0x44,
        MciCtl     = 0x48,
        Flash      = 0x4C,
        Clcd       = 0x50,
        ClcdSer    = 0x54,
        Bootcs     = 0x58,
        Clock24    = 0x5C,
        Misc       = 0x60,
        IoSel      = 0x70,
        ProcId0    = 0x84,
        ProcId1    = 0x88,
        CfgData    = 0xA0,
        CfgCtrl    = 0xA4,
        CfgStat    = 0xA8,
        TestOsc0   = 0xC0,
        TestOsc1   = 0xC4,
        TestOsc2   = 0xC8,
        TestOsc3   = 0xCC,
        TestOsc4   = 0xD0
    };

    // system lock value
    BitUnion32(SysLockReg)
        Bitfield<15,0> lockVal;
        Bitfield<16> locked;
    EndBitUnion(SysLockReg)

    SysLockReg sysLock;

    /** This register is used for smp booting.
     * The primary cpu writes the secondary start address here before
     * sends it a soft interrupt. The secondary cpu reads this register and if
     * it's non-zero it jumps to the address
     */
    uint32_t flags;

    /** This register contains the result from a system control reg access
     */
    uint32_t scData;

  public:
    typedef RealViewCtrlParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    /**
      * The constructor for RealView just registers itself with the MMU.
      * @param p params structure
      */
    RealViewCtrl(Params *p);

    /**
     * Handle a read to the device
     * @param pkt The memory request.
     * @param data Where to put the data.
     */
    virtual Tick read(PacketPtr pkt);

    /**
     * All writes are simply ignored.
     * @param pkt The memory request.
     * @param data the data
     */
    virtual Tick write(PacketPtr pkt);


    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};


#endif // __DEV_ARM_RV_HH__
