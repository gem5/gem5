/*
 * Copyright (c) 2021 The Regents of the University of California
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

#include "dev/lupio/lupio_rtc.hh"

#include "base/time.hh"
#include "debug/LupioRTC.hh"
#include "mem/packet_access.hh"
#include "params/LupioRTC.hh"

namespace gem5
{

LupioRTC::LupioRTC(const Params &params)
    : BasicPioDevice(params, params.pio_size), time(params.time)
{
    start = mktime(&time);
}

uint8_t
LupioRTC::lupioRTCRead(uint8_t addr)
{
    uint8_t r = 0;
    uint32_t time_sec;
    time_t total_time;

    /**
     * Get simulated time in seconds
     */
    time_sec = (curTick() / sim_clock::as_int::s);
    total_time = (time_t)time_sec + start;

    /**
     * Transform into broken-down time representation
     */
    gmtime_r(&total_time, &time);

    switch (addr) {
    case LUPIO_RTC_SECD:
        r = time.tm_sec; /* 0-60 (for leap seconds) */
        break;
    case LUPIO_RTC_MINT:
        r = time.tm_min; /* 0-59 */
        break;
    case LUPIO_RTC_HOUR:
        r = time.tm_hour; /* 0-23 */
        break;
    case LUPIO_RTC_DYMO:
        r = time.tm_mday; /* 1-31 */
        break;
    case LUPIO_RTC_MNTH:
        r = time.tm_mon + 1; /* 1-12 */
        break;
    case LUPIO_RTC_YEAR:
        r = (time.tm_year + 1900) % 100; /* 0-99 */
        break;
    case LUPIO_RTC_CENT:
        r = (time.tm_year + 1900) / 100; /* 0-99 */
        break;
    case LUPIO_RTC_DYWK:
        r = 1 + (time.tm_wday + 6) % 7; /* 1-7 (Monday is 1) */
        break;
    case LUPIO_RTC_DYYR:
        r = time.tm_yday + 1; /* 1-366 (for leap years) */
        break;
    }
    return r;
}

Tick
LupioRTC::read(PacketPtr pkt)
{
    bool is_atomic = pkt->isAtomicOp() && pkt->cmd == MemCmd::SwapReq;
    DPRINTF(LupioRTC, "Read request - addr: %#x, size: %#x, atomic:%d\n",
            pkt->getAddr(), pkt->getSize(), is_atomic);

    Addr rtc_addr = pkt->getAddr() - pioAddr;
    uint8_t time_val = lupioRTCRead(rtc_addr);
    DPRINTF(LupioRTC, "time value: %d\n", time_val);

    pkt->setData(&time_val);

    if (is_atomic) {
        pkt->makeAtomicResponse();
    } else {
        pkt->makeResponse();
    }

    return pioDelay;
}

Tick
LupioRTC::write(PacketPtr pkt)
{
    panic("Unexpected write to the LupioRTC device!");
    return pioDelay;
}

} // namespace gem5
