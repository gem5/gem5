/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
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
 */


/** @file
 * Implementiation of a PL050 KMI
 */

#ifndef __DEV_ARM_PL050_HH__
#define __DEV_ARM_PL050_HH__

#include <list>

#include "base/vnc/vncinput.hh"
#include "dev/arm/amba_device.hh"
#include "params/Pl050.hh"

namespace gem5
{

namespace ps2 {
class Device;
} // namespace ps2

class Pl050 : public AmbaIntDevice
{
  protected:
    static const int kmiCr       = 0x000;
    static const int kmiStat     = 0x004;
    static const int kmiData     = 0x008;
    static const int kmiClkDiv   = 0x00C;
    static const int kmiISR      = 0x010;

    BitUnion8(ControlReg)
        Bitfield<0> force_clock_low;
        Bitfield<1> force_data_low;
        Bitfield<2> enable;
        Bitfield<3> txint_enable;
        Bitfield<4> rxint_enable;
        Bitfield<5> type;
    EndBitUnion(ControlReg)

    /** control register
     */
    ControlReg control;

    /** KMI status register */
    BitUnion8(StatusReg)
        Bitfield<0> data_in;
        Bitfield<1> clk_in;
        Bitfield<2> rxparity;
        Bitfield<3> rxbusy;
        Bitfield<4> rxfull;
        Bitfield<5> txbusy;
        Bitfield<6> txempty;
    EndBitUnion(StatusReg)

    StatusReg status;

    /** clock divisor register
     * This register is just kept around to satisfy reads after driver does
     * writes. The divsor does nothing, as we're not actually signaling ps2
     * serial commands to anything.
     */
    uint8_t clkdiv;

    BitUnion8(InterruptReg)
        Bitfield<0> rx;
        Bitfield<1> tx;
    EndBitUnion(InterruptReg)

    /** raw interrupt register (unmasked) */
    InterruptReg rawInterrupts;

    /** Set or clear the TX interrupt */
    void setTxInt(bool value);

    /** Update the RX interrupt using PS/2 device state */
    void updateRxInt();

    /**
     * Update the status of the interrupt and control registers and
     * deliver an interrupt if required.
     */
    void updateIntCtrl(InterruptReg ints, ControlReg ctrl);

    void setInterrupts(InterruptReg ints) { updateIntCtrl(ints, control); }
    void setControl(ControlReg ctrl) { updateIntCtrl(rawInterrupts, ctrl); }

    /** Get current interrupt value */
    InterruptReg getInterrupt() const;

    /** PS2 device connected to this KMI interface */
    ps2::Device *ps2Device;

  public:
    Pl050(const Pl050Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_ARM_PL050_HH__
