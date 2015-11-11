/*
 * Copyright (c) 2010-2015 ARM Limited
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
 *          Andreas Sandberg
 */


/** @file
 * Implementiation of a PL011 UART
 */

#ifndef __DEV_ARM_PL011_H__
#define __DEV_ARM_PL011_H__

#include "dev/arm/amba_device.hh"
#include "dev/uart.hh"

class BaseGic;
struct Pl011Params;

class Pl011 : public Uart, public AmbaDevice
{
  public:
    Pl011(const Pl011Params *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // PioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  public: // Uart
    void dataAvailable() override;


  protected: // Interrupt handling
    /** Function to generate interrupt */
    void generateInterrupt();

    /**
     * Assign new interrupt values and update interrupt signals
     *
     * A new interrupt is scheduled signalled if the set of unmasked
     * interrupts goes empty to non-empty. Conversely, if the set of
     * unmasked interrupts goes from non-empty to empty, the interrupt
     * signal is cleared.
     *
     * @param ints New <i>raw</i> interrupt status
     * @param mask New interrupt mask
     */
    void setInterrupts(uint16_t ints, uint16_t mask);
    /**
     * Convenience function to update the interrupt mask
     *
     * @see setInterrupts
     * @param mask New interrupt mask
     */
    void setInterruptMask(uint16_t mask) { setInterrupts(rawInt, mask); }
    /**
     * Convenience function to raise a new interrupt
     *
     * @see setInterrupts
     * @param ints Set of interrupts to raise
     */
    void raiseInterrupts(uint16_t ints) { setInterrupts(rawInt | ints, imsc); }
    /**
     * Convenience function to clear interrupts
     *
     * @see setInterrupts
     * @param ints Set of interrupts to clear
     */
    void clearInterrupts(uint16_t ints) { setInterrupts(rawInt & ~ints, imsc); }

    /** Masked interrupt status register */
    const inline uint16_t maskInt() const { return rawInt & imsc; }

    /** Wrapper to create an event out of the thing */
    EventWrapper<Pl011, &Pl011::generateInterrupt> intEvent;

  protected: // Registers
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

    static const uint16_t UART_RIINTR = 1 << 0;
    static const uint16_t UART_CTSINTR = 1 << 1;
    static const uint16_t UART_CDCINTR = 1 << 2;
    static const uint16_t UART_DSRINTR = 1 << 3;
    static const uint16_t UART_RXINTR = 1 << 4;
    static const uint16_t UART_TXINTR = 1 << 5;
    static const uint16_t UART_RTINTR = 1 << 6;
    static const uint16_t UART_FEINTR = 1 << 7;
    static const uint16_t UART_PEINTR = 1 << 8;
    static const uint16_t UART_BEINTR = 1 << 9;
    static const uint16_t UART_OEINTR = 1 << 10;

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

    /** interrupt mask register. */
    uint16_t imsc;

    /** raw interrupt status register */
    uint16_t rawInt;

  protected: // Configuration
    /** Gic to use for interrupting */
    BaseGic * const gic;

    /** Should the simulation end on an EOT */
    const bool endOnEOT;

    /** Interrupt number to generate */
    const int intNum;

    /** Delay before interrupting */
    const Tick intDelay;
};

#endif //__DEV_ARM_PL011_H__
