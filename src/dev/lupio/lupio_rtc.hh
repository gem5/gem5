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

#ifndef __DEV_LUPIO_LUPIO_RTC_HH__
#define __DEV_LUPIO_LUPIO_RTC_HH__

#include "debug/LupioRTC.hh"
#include "dev/io_device.hh"
#include "params/LupioRTC.hh"
#include "sim/system.hh"

namespace gem5
{

/**
 * LupioRTC:
 * A Real-Time Clock Virtual Device that returns the current date and time
 * in ISO 8601 format.
 */
class LupioRTC : public BasicPioDevice
{
  protected:
    time_t start;
    struct tm time;

    // Register map
  private:
    enum
    {
        LUPIO_RTC_SECD,
        LUPIO_RTC_MINT,
        LUPIO_RTC_HOUR,
        LUPIO_RTC_DYMO,
        LUPIO_RTC_MNTH,
        LUPIO_RTC_YEAR,
        LUPIO_RTC_CENT,
        LUPIO_RTC_DYWK,
        LUPIO_RTC_DYYR,

        // Max offset
        LUPIO_RTC_MAX,
    };

    /**
     * Function to return current time and date using system's wall-clock time
     */
    uint8_t lupioRTCRead(const uint8_t addr);

  public:
    PARAMS(LupioRTC);
    LupioRTC(const Params &params);

    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_LUPIO_LUPIO_RTC_HH__
