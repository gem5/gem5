/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __DEV_X86_SOUTH_BRIDGE_SOUTH_BRIDGE_HH__
#define __DEV_X86_SOUTH_BRIDGE_SOUTH_BRIDGE_HH__

#include "base/range_map.hh"
#include "dev/io_device.hh"
#include "dev/x86/south_bridge/cmos.hh"
#include "dev/x86/south_bridge/i8254.hh"
#include "dev/x86/south_bridge/i8259.hh"
#include "dev/x86/south_bridge/speaker.hh"
#include "dev/x86/south_bridge/sub_device.hh"
#include "params/SouthBridge.hh"

class SouthBridge : public PioDevice
{
  protected:
    // PICs
    X86ISA::I8259 pic1;
    X86ISA::I8259 pic2;

    // I8254 Programmable Interval Timer
    X86ISA::I8254 pit;

    // CMOS apperature
    X86ISA::Cmos cmos;

    // PC speaker
    X86ISA::Speaker speaker;

    AddrRangeList rangeList;

    typedef range_map<Addr, X86ISA::SubDevice *> RangeMap;
    typedef RangeMap::iterator RangeMapIt;
    RangeMap rangeMap;


    void addDevice(X86ISA::SubDevice &);

  public:
    void addressRanges(AddrRangeList &range_list);

    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);

    typedef SouthBridgeParams Params;
    SouthBridge(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
};

#endif //__DEV_X86_SOUTH_BRIDGE_SOUTH_BRIDGE_HH__
