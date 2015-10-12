/*
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
 * Defines a 8250 UART
 */

#ifndef __DEV_UART8250_HH__
#define __DEV_UART8250_HH__

#include "dev/io_device.hh"
#include "dev/uart.hh"
#include "params/Uart8250.hh"

/* UART8250 Interrupt ID Register
 *  bit 0    Interrupt Pending 0 = true, 1 = false
 *  bit 2:1  ID of highest priority interrupt
 *  bit 7:3  zeroes
 */
const uint8_t IIR_NOPEND = 0x1;

// Interrupt IDs
const uint8_t IIR_MODEM = 0x00; /* Modem Status (lowest priority) */
const uint8_t IIR_TXID  = 0x02; /* Tx Data */
const uint8_t IIR_RXID  = 0x04; /* Rx Data */
const uint8_t IIR_LINE  = 0x06; /* Rx Line Status (highest priority)*/

const uint8_t UART_IER_RDI  = 0x01;
const uint8_t UART_IER_THRI = 0x02;
const uint8_t UART_IER_RLSI = 0x04;


const uint8_t UART_LSR_TEMT = 0x40;
const uint8_t UART_LSR_THRE = 0x20;
const uint8_t UART_LSR_DR   = 0x01;

const uint8_t UART_MCR_LOOP = 0x10;


class Terminal;
class Platform;

class Uart8250 : public Uart
{
  protected:
    uint8_t IER, DLAB, LCR, MCR;
    Tick lastTxInt;

    class IntrEvent : public Event
    {
        protected:
            Uart8250 *uart;
            int intrBit;
        public:
            IntrEvent(Uart8250 *u, int bit);
            virtual void process();
            virtual const char *description() const;
            void scheduleIntr();
    };

    IntrEvent txIntrEvent;
    IntrEvent rxIntrEvent;

  public:
    typedef Uart8250Params Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    Uart8250(const Params *p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

    /**
     * Inform the uart that there is data available.
     */
    void dataAvailable() override;


    /**
     * Return if we have an interrupt pending
     * @return interrupt status
     */
    virtual bool intStatus() { return status ? true : false; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __TSUNAMI_UART_HH__
