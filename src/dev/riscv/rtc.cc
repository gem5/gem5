/*
 * Copyright (c) 2021 Huawei International
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

#include "dev/riscv/rtc.hh"

#include "dev/mc146818.hh"
#include "params/RiscvRTC.hh"

namespace gem5
{

RiscvRTC::RiscvRTC(const Params &params)
    : SimObject(params),
      rtc(this, params.name, params.time, params.bcd, params.frequency,
          params.port_int_pin_connection_count)
{}

RiscvRTC::RTC::RTC(EventManager *em, const std::string &n,
                   const struct tm time, bool bcd, Tick frequency,
                   int int_pin_count)
    : MC146818(em, n, time, bcd, frequency)
{
    for (int i = 0; i < int_pin_count; i++) {
        intPin.emplace_back(
            new IntSourcePin<RTC>(csprintf("%s.int_pin[%d]", n, i), i, this));
    }
}

void
RiscvRTC::RTC::handleEvent()
{
    for (auto &pin : intPin) {
        pin->raise();
        pin->lower();
    }
}

Port &
RiscvRTC::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "int_pin")
        return *rtc.intPin.at(idx);
    else
        panic("Getting invalid port " + if_name);
}

void
RiscvRTC::startup()
{
    rtc.startup();
}

void
RiscvRTC::serialize(CheckpointOut &cp) const
{
    // Serialize the timer
    rtc.serialize("rtc", cp);
}

void
RiscvRTC::unserialize(CheckpointIn &cp)
{
    // Serialize the timer
    rtc.unserialize("rtc", cp);
}

} // namespace gem5
