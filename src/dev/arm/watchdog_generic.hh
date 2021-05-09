/*
 * Copyright (c) 2020 ARM Limited
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

#ifndef __DEV_ARM_WATCHDOG_GENERIC_HH__
#define __DEV_ARM_WATCHDOG_GENERIC_HH__

#include "dev/arm/generic_timer.hh"
#include "dev/io_device.hh"

namespace gem5
{

class ArmInterruptPin;
struct GenericWatchdogParams;

/**
 * @file
 * Arm SBSA Generic Watchdog
 * Reference:
 *     Arm Server Base System Architecture (SBSA)
 *     Doc. ID: ARM-DEN-0029A Version 3.1
 */
class GenericWatchdog : public PioDevice
{
  public:
    GenericWatchdog(const GenericWatchdogParams &params);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    bool enabled() const { return controlStatus.enabled; }

  protected:
    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    uint32_t readRefresh(Addr addr);
    uint32_t readControl(Addr addr);

    void writeRefresh(Addr addr, uint32_t data);
    void writeControl(Addr addr, uint32_t data);

  protected:
    /**
     * System Counter Listener: This object is being notified
     * any time there is a change in the SystemCounter.
     * (Like a change in the counter frequency)
     * This is needed since the Generic Watchdog doesn't have
     * a standalone counter and it is instead relying on the
     * global System Counter.
     */
    class Listener : public SystemCounterListener
    {
      public:
        explicit Listener(GenericWatchdog& _parent)
          : parent(_parent)
        {}

        void notify(void) override
        {
            panic_if(parent.enabled(),
                "The Generic Watchdog shall be disabled when "
                "the System Counter is being updated, or "
                "the results are unpredictable");
        }

      protected:
        GenericWatchdog &parent;
    };

    void explicitRefresh();
    void refresh();
    void timeout();
    EventFunctionWrapper timeoutEvent;

  private:
    enum class RefreshOffset : Addr
    {
        WRR = 0x000, // Watchdog Refresh Register
        W_IIDR = 0xfcc, // Watchdog Interface Identification Register
    };

    enum class ControlOffset : Addr
    {
        WCS = 0x000, // Watchdog Control and Status Register
        WOR = 0x008, // Watchdog Offset Register
        WCV_LO = 0x010, // Watchdog Compare Register [31:0]
        WCV_HI = 0x014, // Watchdog Compare Register [63:32]
        W_IIDR = 0xfcc, // Watchdog Interface Identification Register
    };

    BitUnion32(WCTRLS)
        Bitfield<2> ws1; // Watchdog Signal 1 Status
        Bitfield<1> ws0; // Watchdog Signal 0 Status
        Bitfield<0> enabled; // Watchdog Enable
    EndBitUnion(WCTRLS)

    /** Control and Status Register */
    WCTRLS controlStatus;

    /** Offset Register */
    uint32_t offset;

    /** Compare Register */
    uint64_t compare;

    /** Interface Identification Register */
    const uint32_t iidr;

    const AddrRange refreshFrame;
    const AddrRange controlFrame;

    const Tick pioLatency;

    SystemCounter &cnt;
    Listener cntListener;

    /** Watchdog Signals (IRQs) */
    ArmInterruptPin * const ws0;
    ArmInterruptPin * const ws1;
};

} // namespace gem5

#endif // __DEV_ARM_WATCHDOG_GENERIC_HH__
