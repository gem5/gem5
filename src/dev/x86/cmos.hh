/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#ifndef __DEV_X86_CMOS_HH__
#define __DEV_X86_CMOS_HH__

#include "base/bitunion.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"
#include "dev/mc146818.hh"
#include "params/Cmos.hh"

namespace gem5
{

namespace X86ISA
{

class Cmos : public BasicPioDevice
{
  protected:
    Tick latency;

    BitUnion8(CmosAddress)
        Bitfield<6, 0> regNum;
        Bitfield<7> nmiMask;
    EndBitUnion(CmosAddress)

    CmosAddress address;

    static const int numRegs = 128;

    uint8_t regs[numRegs];

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t val);

    class X86RTC : public MC146818
    {
      public:
        std::vector<IntSourcePin<X86RTC> *> intPin;

        X86RTC(EventManager *em, const std::string &n, const struct tm time,
                bool bcd, Tick frequency, int int_pin_count) :
            MC146818(em, n, time, bcd, frequency)
        {
            for (int i = 0; i < int_pin_count; i++) {
                intPin.push_back(new IntSourcePin<X86RTC>(
                            csprintf("%s.int_pin[%d]", n, i), i, this));
            }
        }
      protected:
        void handleEvent();
    } rtc;

  public:
    typedef CmosParams Params;

    Cmos(const Params &p) : BasicPioDevice(p, 2), latency(p.pio_latency),
        rtc(this, name() + ".rtc", p.time, true, 5000000000ULL,
                p.port_int_pin_connection_count)
    {
        memset(regs, 0, numRegs * sizeof(uint8_t));
        address = 0;
    }

    Port &
    getPort(const std::string &if_name, PortID idx=InvalidPortID) override
    {
        if (if_name == "int_pin")
            return *rtc.intPin.at(idx);
        else
            return BasicPioDevice::getPort(if_name, idx);
    }

    Tick read(PacketPtr pkt) override;

    Tick write(PacketPtr pkt) override;

    void startup() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace X86ISA
} // namespace gem5

#endif //__DEV_X86_CMOS_HH__
