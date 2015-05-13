/*
 * Copyright (c) 2010,2013,2015 ARM Limited
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
#include "params/RealViewOsc.hh"
#include "params/RealViewTemperatureSensor.hh"

/** @file
 * This implements the simple real view registers on a PBXA9
 */

class RealViewCtrl : public BasicPioDevice
{
  public:
    enum DeviceFunc {
        FUNC_OSC      = 1,
        FUNC_VOLT     = 2,
        FUNC_AMP      = 3,
        FUNC_TEMP     = 4,
        FUNC_RESET    = 5,
        FUNC_SCC      = 6,
        FUNC_MUXFPGA  = 7,
        FUNC_SHUTDOWN = 8,
        FUNC_REBOOT   = 9,
        FUNC_DVIMODE  = 11,
        FUNC_POWER    = 12,
        FUNC_ENERGY   = 13,
    };

    class Device
    {
      public:
        Device(RealViewCtrl &parent, DeviceFunc func,
               uint8_t site, uint8_t pos, uint8_t dcc, uint16_t dev)
        {
            parent.registerDevice(func, site, pos, dcc, dev,  this);
        }

        virtual ~Device() {}

        virtual uint32_t read() const = 0;
        virtual void write(uint32_t value) = 0;
    };

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

    BitUnion32(CfgCtrlReg)
        Bitfield<11, 0> dev;
        Bitfield<15, 12> pos;
        Bitfield<17, 16> site;
        Bitfield<25, 20> func;
        Bitfield<29, 26> dcc;
        Bitfield<30> wr;
        Bitfield<31> start;
    EndBitUnion(CfgCtrlReg)

    static const uint32_t CFG_CTRL_ADDR_MASK = 0x3fffffffUL;

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
    Tick read(PacketPtr pkt) override;

    /**
     * All writes are simply ignored.
     * @param pkt The memory request.
     * @param data the data
     */
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    void registerDevice(DeviceFunc func, uint8_t site, uint8_t pos,
                        uint8_t dcc, uint16_t dev,
                        Device *handler);

  protected:
    std::map<uint32_t, Device *> devices;
};

/**
 * This is an implementation of a programmable oscillator on the that
 * can be configured through the RealView/Versatile Express
 * configuration interface.
 *
 * See ARM DUI 0447J (ARM  Motherboard Express uATX -- V2M-P1).
 */
class RealViewOsc
    : public ClockDomain, RealViewCtrl::Device
{
  public:
    RealViewOsc(RealViewOscParams *p);
    virtual ~RealViewOsc() {};

    void startup() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // RealViewCtrl::Device interface
    uint32_t read() const override;
    void write(uint32_t freq) override;

  protected:
    void clockPeriod(Tick clock_period);
};

/**
 * This device implements the temperature sensor used in the
 * RealView/Versatile Express platform.
 *
 * See ARM DUI 0447J (ARM  Motherboard Express uATX -- V2M-P1).
 */
class RealViewTemperatureSensor
    : public SimObject, RealViewCtrl::Device
{
  public:
    RealViewTemperatureSensor(RealViewTemperatureSensorParams *p)
    : SimObject(p),
      RealViewCtrl::Device(*p->parent, RealViewCtrl::FUNC_TEMP,
                           p->site, p->position, p->dcc, p->device),
      system(p->system)
    {}
    virtual ~RealViewTemperatureSensor() {};

  public: // RealViewCtrl::Device interface
    uint32_t read() const override;
    void write(uint32_t temp) override {}

  protected:
    /** The system this RV device belongs to */
    System * system;
};


#endif // __DEV_ARM_RV_HH__
