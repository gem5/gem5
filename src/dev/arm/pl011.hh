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
 * Authors: Ali Saidi
 */


/** @file
 * Implementiation of a PL011 UART
 */

#ifndef __DEV_ARM_PL011_H__
#define __DEV_ARM_PL011_H__

#include "base/range.hh"
#include "dev/io_device.hh"
#include "dev/uart.hh"
#include "params/Pl011.hh"

class Gic;

class Pl011 : public Uart
{
  protected:
    static const uint64_t AMBA_ID = ULL(0xb105f00d00341011);
    static const int UART_DR = 0x000;
    static const int UART_FR = 0x018;
    static const int UART_FR_CTS  = 0x001;
    static const int UART_FR_TXFE = 0x080;
    static const int UART_FR_RXFE = 0x010;
    static const int UART_IBRD = 0x024;
    static const int UART_FBRD = 0x028;
    static const int UART_LCRH = 0x02C;
    static const int UART_CR   = 0x030;
    static const int UART_IFLS = 0x034;
    static const int UART_IMSC = 0x038;
    static const int UART_RIS  = 0x03C;
    static const int UART_MIS  = 0x040;
    static const int UART_ICR  = 0x044;

    uint16_t control;

    /** fractional baud rate divisor. Not used for anything but reporting
     * written value */
    uint16_t fbrd;

    /** integer baud rate divisor. Not used for anything but reporting
     * written value */
    uint16_t ibrd;

    /** Line control register. Not used for anything but reporting
     * written value */
    uint16_t lcrh;

    /** interrupt fifo level register. Not used for anything but reporting
     * written value */
    uint16_t ifls;

    BitUnion16(INTREG)
        Bitfield<0> rimim;
        Bitfield<1> ctsmim;
        Bitfield<2> dcdmim;
        Bitfield<3> dsrmim;
        Bitfield<4> rxim;
        Bitfield<5> txim;
        Bitfield<6> rtim;
        Bitfield<7> feim;
        Bitfield<8> peim;
        Bitfield<9> beim;
        Bitfield<10> oeim;
        Bitfield<15,11> rsvd;
    EndBitUnion(INTREG)

    /** interrupt mask register. */
    INTREG imsc;

    /** raw interrupt status register */
    INTREG rawInt;

    /** Masked interrupt status register */
    INTREG maskInt;

    /** Interrupt number to generate */
    int intNum;

    /** Gic to use for interrupting */
    Gic *gic;

    /** Should the simulation end on an EOT */
    bool endOnEOT;

    /** Delay before interrupting */
    Tick intDelay;

    /** Function to generate interrupt */
    void generateInterrupt();

    /** Wrapper to create an event out of the thing */
    EventWrapper<Pl011, &Pl011::generateInterrupt> intEvent;

  public:
   typedef Pl011Params Params;
   const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    Pl011(const Params *p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    /**
     * Inform the uart that there is data available.
     */
    virtual void dataAvailable();


    /**
     * Return if we have an interrupt pending
     * @return interrupt status
     * @todo fix me when implementation improves
     */
    virtual bool intStatus() { return false; }

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif //__DEV_ARM_PL011_H__
