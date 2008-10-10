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
 *
 * Authors: Gabe Black
 */

#ifndef __DEV_X86_SOUTH_BRIDGE_CMOS_HH__
#define __DEV_X86_SOUTH_BRIDGE_CMOS_HH__

#include "arch/x86/x86_traits.hh"
#include "base/range.hh"
#include "dev/mc146818.hh"
#include "dev/x86/south_bridge/sub_device.hh"

namespace X86ISA
{

class Cmos : public SubDevice
{
  protected:
    uint8_t address;

    struct tm foo_time;

    static const int numRegs = 128;

    uint8_t regs[numRegs];

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t val);

    class X86RTC : public MC146818
    {
      public:
        X86RTC(EventManager *em, const std::string &n, const struct tm time,
                bool bcd, Tick frequency) :
            MC146818(em, n, time, bcd, frequency)
        {
        }
      protected:
        void handleEvent()
        {
            return;
        }
    } rtc;

  public:

    Cmos(EventManager *em) : rtc(em, "rtc", foo_time, true, ULL(5000000000))
    {
        memset(regs, 0, numRegs * sizeof(uint8_t));
        address = 0;
    }

    Cmos(EventManager *em, Tick _latency) : SubDevice(_latency),
        rtc(em, "rtc", foo_time, true, ULL(5000000000))
    {
        memset(regs, 0, numRegs * sizeof(uint8_t));
        address = 0;
    }

    Cmos(EventManager *em, Addr start, Addr size, Tick _latency) :
        SubDevice(start, size, _latency),
        rtc(em, "rtc", foo_time, true, ULL(5000000000))
    {
        memset(regs, 0, numRegs * sizeof(uint8_t));
        address = 0;
    }

    Tick read(PacketPtr pkt);

    Tick write(PacketPtr pkt);
};

}; // namespace X86ISA

#endif //__DEV_X86_SOUTH_BRIDGE_CMOS_HH__
