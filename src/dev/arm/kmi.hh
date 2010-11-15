/*
 * Copyright (c) 2010 ARM Limited
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
 * Copyright (c) 2005 The Regents of The University of Michigan
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
 * Authors: William Wang
 */


/** @file
 * Implementiation of a PL050 KMI
 */

#ifndef __DEV_ARM_PL050_HH__
#define __DEV_ARM_PL050_HH__

#include "base/range.hh"
#include "dev/io_device.hh"
#include "params/Pl050.hh"

class Gic;

class Pl050 : public AmbaDevice
{
  protected:
    static const int kmiCr       = 0x000;
    static const int kmiStat     = 0x004;
    static const int kmiData     = 0x008;
    static const int kmiClkDiv   = 0x00C;
    static const int kmiISR      = 0x010;

    // control register
    uint8_t control;

    // status register
    uint8_t status;

    // received data (read) or data to be transmitted (write)
    uint8_t kmidata;

    // clock divisor register
    uint8_t clkdiv;

    BitUnion8(IntReg)
    Bitfield<0> txintr;
    Bitfield<1> rxintr;
    EndBitUnion(IntReg)

    /** interrupt mask register. */
    IntReg intreg;

    /** Interrupt number to generate */
    int intNum;

    /** Gic to use for interrupting */
    Gic *gic;

    /** Delay before interrupting */
    Tick intDelay;

    /** Function to generate interrupt */
    void generateInterrupt();

    /** Wrapper to create an event out of the thing */
    EventWrapper<Pl050, &Pl050::generateInterrupt> intEvent;

  public:
    typedef Pl050Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Pl050(const Params *p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    /**
     * Return if we have an interrupt pending
     * @return interrupt status
     * @todo fix me when implementation improves
     */
    virtual bool intStatus() { return false; }
};

#endif
