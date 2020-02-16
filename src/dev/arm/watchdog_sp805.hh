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

#ifndef __DEV_ARM_WATCHDOG_SP805_HH__
#define __DEV_ARM_WATCHDOG_SP805_HH__

#include "dev/arm/amba_device.hh"

class Sp805Params;

/**
 * @file
 * Arm Watchdog Module (SP805)
 * Reference:
 *     Arm Watchdog Module (SP805) - Technical Reference Manual - rev. r1p0
 *     Doc. ID: ARM DDI 0270B
 */
class Sp805 : public AmbaIntDevice
{
  public:
    Sp805(Sp805Params const* params);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  private:
    enum Offset : Addr {
        WDOGLOAD      = 0x000,
        WDOGVALUE     = 0x004,
        WDOGCONTROL   = 0x008,
        WDOGINTCLR    = 0x00c,
        WDOGRIS       = 0x010,
        WDOGMIS       = 0x014,
        // 0x018 - 0xbfc -> Reserved
        WDOGLOCK      = 0xc00,
        // 0xc04 - 0xefc -> Reserved
        WDOGITCR      = 0xf00,
        WDOGITOP      = 0xf04,
        // 0xf08 - 0xfdc -> Reserved
        // 0xfe0 - 0xfff -> CoreSight / Peripheral ID (AMBA ID)
    };

    /** Timeout interval (in cycles) as specified in WdogLoad */
    uint32_t timeoutInterval;

    /** Timeout start tick to keep track of the counter value */
    Tick timeoutStartTick;

    /** Value as persisted when the watchdog is stopped */
    uint32_t persistedValue;

    /** Indicates if watchdog (counter and interrupt) is enabled */
    bool enabled;

    /** Indicates if reset behaviour is enabled when counter reaches 0 */
    bool resetEnabled;

    /** Indicates if an interrupt has been raised by the counter reaching 0 */
    bool intRaised;

    /** Indicates if write access to registers is enabled */
    bool writeAccessEnabled;

    /** Indicates if integration test harness is enabled */
    bool integrationTestEnabled;

    /** Timeout event, triggered when the counter value reaches 0 */
    EventFunctionWrapper timeoutEvent;

    /** Returns the current counter value */
    uint32_t value(void) const;

    /** Triggered when value reaches 0 */
    void timeoutExpired(void);

    /** Restarts the counter to the current timeout interval */
    void restartCounter(void);

    /** Stops the counter when watchdog becomes disabled */
    void stopCounter(void);

    /**
     * Raises an interrupt. If there is already a pending interrupt and
     * reset behaviour is enabled, asserts system reset
     */
    void sendInt(void);

    /** Clears any active interrupts */
    void clearInt(void);

    /** If written into WdogLock, registers are unlocked for writes */
    static constexpr uint32_t WDOGLOCK_MAGIC = 0x1acce551;
};

#endif // __DEV_ARM_WATCHDOG_SP805_HH__
