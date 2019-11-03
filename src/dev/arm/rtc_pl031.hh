/*
 * Copyright (c) 2010-2012 ARM Limited
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

#ifndef __DEV_ARM_RTC_PL310_HH__
#define __DEV_ARM_RTC_PL310_HH__

#include "dev/arm/amba_device.hh"
#include "params/PL031.hh"

/** @file
 * This implements the ARM Primecell 031 RTC
 */

class PL031 : public AmbaIntDevice
{
  protected:
    enum {
        DataReg    = 0x00,
        MatchReg   = 0x04,
        LoadReg    = 0x08,
        ControlReg = 0x0C,
        IntMask    = 0x10,
        RawISR     = 0x14,
        MaskedISR  = 0x18,
        IntClear   = 0x1C,
    };

    /* Seconds since epoch that correspond to time simulation was started at the
     * begining of simulation and is then updated if ever written. */
    uint32_t timeVal;

    /* Time when the timeVal register was written */
    Tick lastWrittenTick;

    /* Previous load value */
    uint32_t loadVal;

    /* RTC Match Value
     * Cause an interrupt when this value hits counter
     */
    uint32_t matchVal;

    /** If timer has caused an interrupt. This is irrespective of
     * interrupt enable */
    bool rawInt;

    /** If the timer interrupt mask that is anded with the raw interrupt to
     * generate a pending interrupt
     */
    bool maskInt;

    /** If an interrupt is currently pending. Logical and of CTRL.intEnable
     * and rawInt */
    bool pendingInt;

    /** Called when the counter reaches matches */
    void counterMatch();
    EventFunctionWrapper matchEvent;

    /** Called to update the matchEvent when the load Value or match value are
     * written.
     */
    void resyncMatch();

  public:
    typedef PL031Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    /**
      * The constructor for RealView just registers itself with the MMU.
      * @param p params structure
      */
    PL031(Params *p);

    /**
     * Handle a read to the device
     * @param pkt The memory request.
     * @param data Where to put the data.
     */
    Tick read(PacketPtr pkt) override;

    /**
     * Handle writes to the device
     * @param pkt The memory request.
     * @param data the data
     */
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};


#endif // __DEV_ARM_RTC_PL031_HH__

