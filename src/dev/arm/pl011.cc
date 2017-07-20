/*
 * Copyright (c) 2010, 2015 ARM Limited
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

#include "dev/arm/pl011.hh"

#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/Uart.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/Pl011.hh"
#include "sim/sim_exit.hh"

Pl011::Pl011(const Pl011Params *p)
    : Uart(p, 0xfff),
      intEvent([this]{ generateInterrupt(); }, name()),
      control(0x300), fbrd(0), ibrd(0), lcrh(0), ifls(0x12),
      imsc(0), rawInt(0),
      gic(p->gic), endOnEOT(p->end_on_eot), intNum(p->int_num),
      intDelay(p->int_delay)
{
}

Tick
Pl011::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Uart, " read register %#x size=%d\n", daddr, pkt->getSize());

    // use a temporary data since the uart registers are read/written with
    // different size operations
    //
    uint32_t data = 0;

    switch(daddr) {
      case UART_DR:
        data = 0;
        if (device->dataAvailable()) {
            data = device->readData();
            // Since we don't simulate a FIFO for incoming data, we
            // assume it's empty and clear RXINTR and RTINTR.
            clearInterrupts(UART_RXINTR | UART_RTINTR);
            if (device->dataAvailable()) {
                DPRINTF(Uart, "Re-raising interrupt due to more data "
                        "after UART_DR read\n");
                dataAvailable();
            }
        }
        break;
      case UART_FR:
        data =
            UART_FR_CTS | // Clear To Send
            // Given we do not simulate a FIFO we are either empty or full.
            (!device->dataAvailable() ? UART_FR_RXFE : UART_FR_RXFF) |
            UART_FR_TXFE; // TX FIFO empty

        DPRINTF(Uart,
                "Reading FR register as %#x rawInt=0x%x "
                "imsc=0x%x maskInt=0x%x\n",
                data, rawInt, imsc, maskInt());
        break;
      case UART_CR:
        data = control;
        break;
      case UART_IBRD:
        data = ibrd;
        break;
      case UART_FBRD:
        data = fbrd;
        break;
      case UART_LCRH:
        data = lcrh;
        break;
      case UART_IFLS:
        data = ifls;
        break;
      case UART_IMSC:
        data = imsc;
        break;
      case UART_RIS:
        data = rawInt;
        DPRINTF(Uart, "Reading Raw Int status as 0x%x\n", rawInt);
        break;
      case UART_MIS:
        DPRINTF(Uart, "Reading Masked Int status as 0x%x\n", maskInt());
        data = maskInt();
        break;
      default:
        if (readId(pkt, AMBA_ID, pioAddr)) {
            // Hack for variable size accesses
            data = pkt->get<uint32_t>();
            break;
        }

        panic("Tried to read PL011 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    switch(pkt->getSize()) {
      case 1:
        pkt->set<uint8_t>(data);
        break;
      case 2:
        pkt->set<uint16_t>(data);
        break;
      case 4:
        pkt->set<uint32_t>(data);
        break;
      default:
        panic("Uart read size too big?\n");
        break;
    }


    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Pl011::write(PacketPtr pkt)
{

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Uart, " write register %#x value %#x size=%d\n", daddr,
            pkt->get<uint8_t>(), pkt->getSize());

    // use a temporary data since the uart registers are read/written with
    // different size operations
    //
    uint32_t data = 0;

    switch(pkt->getSize()) {
      case 1:
        data = pkt->get<uint8_t>();
        break;
      case 2:
        data = pkt->get<uint16_t>();
        break;
      case 4:
        data = pkt->get<uint32_t>();
        break;
      default:
        panic("Uart write size too big?\n");
        break;
    }


    switch (daddr) {
        case UART_DR:
          if ((data & 0xFF) == 0x04 && endOnEOT)
            exitSimLoop("UART received EOT", 0);

        device->writeData(data & 0xFF);
        // We're supposed to clear TXINTR when this register is
        // written to, however. since we're also infinitely fast, we
        // need to immediately raise it again.
        clearInterrupts(UART_TXINTR);
        raiseInterrupts(UART_TXINTR);
        break;
      case UART_CR:
        control = data;
        break;
      case UART_IBRD:
        ibrd = data;
        break;
      case UART_FBRD:
        fbrd = data;
        break;
      case UART_LCRH:
        lcrh = data;
        break;
      case UART_IFLS:
        ifls = data;
        break;
      case UART_IMSC:
        DPRINTF(Uart, "Setting interrupt mask 0x%x\n", data);
        setInterruptMask(data);
        break;

      case UART_ICR:
        DPRINTF(Uart, "Clearing interrupts 0x%x\n", data);
        clearInterrupts(data);
        if (device->dataAvailable()) {
            DPRINTF(Uart, "Re-raising interrupt due to more data after "
                    "UART_ICR write\n");
            dataAvailable();
        }
        break;
      default:
        panic("Tried to write PL011 at offset %#x that doesn't exist\n", daddr);
        break;
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Pl011::dataAvailable()
{
    /*@todo ignore the fifo, just say we have data now
     * We might want to fix this, or we might not care */
    DPRINTF(Uart, "Data available, scheduling interrupt\n");
    raiseInterrupts(UART_RXINTR | UART_RTINTR);
}

void
Pl011::generateInterrupt()
{
    DPRINTF(Uart, "Generate Interrupt: imsc=0x%x rawInt=0x%x maskInt=0x%x\n",
            imsc, rawInt, maskInt());

    if (maskInt()) {
        gic->sendInt(intNum);
        DPRINTF(Uart, " -- Generated\n");
    }
}

void
Pl011::setInterrupts(uint16_t ints, uint16_t mask)
{
    const bool old_ints(!!maskInt());

    imsc = mask;
    rawInt = ints;

    if (!old_ints && maskInt()) {
        if (!intEvent.scheduled())
            schedule(intEvent, curTick() + intDelay);
    } else if (old_ints && !maskInt()) {
        gic->clearInt(intNum);
    }
}



void
Pl011::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm PL011\n");
    SERIALIZE_SCALAR(control);
    SERIALIZE_SCALAR(fbrd);
    SERIALIZE_SCALAR(ibrd);
    SERIALIZE_SCALAR(lcrh);
    SERIALIZE_SCALAR(ifls);

    // Preserve backwards compatibility by giving these silly names.
    paramOut(cp, "imsc_serial", imsc);
    paramOut(cp, "rawInt_serial", rawInt);
}

void
Pl011::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm PL011\n");

    UNSERIALIZE_SCALAR(control);
    UNSERIALIZE_SCALAR(fbrd);
    UNSERIALIZE_SCALAR(ibrd);
    UNSERIALIZE_SCALAR(lcrh);
    UNSERIALIZE_SCALAR(ifls);

    // Preserve backwards compatibility by giving these silly names.
    paramIn(cp, "imsc_serial", imsc);
    paramIn(cp, "rawInt_serial", rawInt);
}

Pl011 *
Pl011Params::create()
{
    return new Pl011(this);
}
