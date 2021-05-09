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
 */

/** @file
 * Implements a 8250 UART
 */

#include "dev/serial/uart8250.hh"

#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/trace.hh"
#include "debug/Uart.hh"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/serialize.hh"

namespace gem5
{

void
Uart8250::processIntrEvent(int intrBit)
{
    if (intrBit & registers.ier.get()) {
       DPRINTF(Uart, "UART InterEvent, interrupting\n");
       platform->postConsoleInt();
       status |= intrBit;
       lastTxInt = curTick();
    } else {
       DPRINTF(Uart, "UART InterEvent, not interrupting\n");
    }

}

/* The linux serial driver (8250.c about line 1182) loops reading from
 * the device until the device reports it has no more data to
 * read. After a maximum of 255 iterations the code prints "serial8250
 * too much work for irq X," and breaks out of the loop. Since the
 * simulated system is so much slower than the actual system, if a
 * user is typing on the keyboard it is very easy for them to provide
 * input at a fast enough rate to not allow the loop to exit and thus
 * the error to be printed. This magic number provides a delay between
 * the time the UART receives a character to send to the simulated
 * system and the time it actually notifies the system it has a
 * character to send to alleviate this problem. --Ali
 */
void
Uart8250::scheduleIntr(Event *event)
{
    static const Tick interval = 225 * sim_clock::as_int::ns;
    DPRINTF(Uart, "Scheduling IER interrupt for %s, at cycle %lld\n",
            event->name(), curTick() + interval);
    if (!event->scheduled())
        schedule(event, curTick() + interval);
    else
        reschedule(event, curTick() + interval);
}


Uart8250::Uart8250(const Params &p)
    : Uart(p, p.pio_size), registers(this, name() + ".registers"),
      lastTxInt(0),
      txIntrEvent([this]{ processIntrEvent(TX_INT); }, "TX"),
      rxIntrEvent([this]{ processIntrEvent(RX_INT); }, "RX")
{
}

Uart8250::Registers::Registers(Uart8250 *uart, const std::string &new_name) :
    RegisterBankLE(new_name, 0), rbrThr(rbr, thr), rbrThrDll(rbrThr, dll),
    ierDlh(ier, dlh), iirFcr(iir, fcr)
{
    rbr.reader(uart, &Uart8250::readRbr);
    thr.writer(uart, &Uart8250::writeThr);
    ier.writer(uart, &Uart8250::writeIer);
    iir.reader(uart, &Uart8250::readIir);

    lcr.writer([this](auto &reg, const auto &value) {
            reg.update(value);
            rbrThrDll.select(value.dlab);
            ierDlh.select(value.dlab);
        });

    mcr.writer([](auto &reg, const auto &value) {
            if (value == (UART_MCR_LOOP | 0x0A))
                reg.update(0x9A);
        });

    lsr.readonly().
        reader([device = uart->device](auto &reg) {
            Lsr lsr = 0;
            if (device->dataAvailable())
                lsr.rdr = 1;
            lsr.tbe = 1;
            lsr.txEmpty = 1;
            return lsr;
        });

    msr.readonly();

    addRegisters({rbrThrDll, ierDlh, iirFcr, lcr, mcr, lsr, msr, sr});
}

uint8_t
Uart8250::readRbr(Register8 &reg)
{
    uint8_t data = 0;
    if (device->dataAvailable())
        data = device->readData();
    else
        DPRINTF(Uart, "empty read of RX register\n");

    status &= ~RX_INT;
    platform->clearConsoleInt();

    if (device->dataAvailable() && registers.ier.get().rdi)
        scheduleIntr(&rxIntrEvent);

    return data;
}

void
Uart8250::writeThr(Register8 &reg, const uint8_t &data)
{
    device->writeData(data);
    platform->clearConsoleInt();
    status &= ~TX_INT;
    if (registers.ier.get().thri)
        scheduleIntr(&txIntrEvent);
}

Uart8250::Iir
Uart8250::readIir(Register<Iir> &reg)
{
    DPRINTF(Uart, "IIR Read, status = %#x\n", (uint32_t)status);

    Iir iir = 0;
    if (status & RX_INT) {
        // Rx data interrupt has a higher priority.
        iir.id = (uint8_t)InterruptIds::Rx;
    } else if (status & TX_INT) {
        iir.id = (uint8_t)InterruptIds::Tx;
        // Tx interrupts are cleared on IIR reads.
        status &= ~TX_INT;
    } else {
        iir.pending = 1;
    }
    return iir;
}

void
Uart8250::writeIer(Register<Ier> &reg, const Ier &ier)
{
    reg.update(ier);

    if (ier.thri) {
        DPRINTF(Uart, "IER: IER_THRI set, scheduling TX intrrupt\n");
        if (curTick() - lastTxInt > 225 * sim_clock::as_int::ns) {
            DPRINTF(Uart, "-- Interrupting Immediately... %d,%d\n",
                    curTick(), lastTxInt);
            txIntrEvent.process();
        } else {
            DPRINTF(Uart, "-- Delaying interrupt... %d,%d\n",
                    curTick(), lastTxInt);
            scheduleIntr(&txIntrEvent);
        }
    } else {
        DPRINTF(Uart, "IER: IER_THRI cleared, descheduling TX intrrupt\n");
        if (txIntrEvent.scheduled())
            deschedule(txIntrEvent);
        if (status & TX_INT)
            platform->clearConsoleInt();
        status &= ~TX_INT;
    }

    if (ier.rdi && device->dataAvailable()) {
        DPRINTF(Uart, "IER: IER_RDI set, scheduling RX intrrupt\n");
        scheduleIntr(&rxIntrEvent);
    } else {
        DPRINTF(Uart, "IER: IER_RDI cleared, descheduling RX intrrupt\n");
        if (rxIntrEvent.scheduled())
            deschedule(rxIntrEvent);
        if (status & RX_INT)
            platform->clearConsoleInt();
        status &= ~RX_INT;
    }
}

Tick
Uart8250::read(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Uart, "Read register %#x\n", daddr);

    registers.read(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Uart8250::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(Uart, "Write register %#x value %#x\n", daddr,
            pkt->getRaw<uint8_t>());

    registers.write(daddr, pkt->getPtr<void>(), pkt->getSize());

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Uart8250::dataAvailable()
{
    // If the kernel wants an interrupt when we have data.
    if (registers.ier.get().rdi) {
        platform->postConsoleInt();
        status |= RX_INT;
    }

}

AddrRangeList
Uart8250::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

void
Uart8250::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(status);
    paramOut(cp, "IER", registers.ier);
    paramOut(cp, "LCR", registers.lcr);
    paramOut(cp, "MCR", registers.mcr);
    Tick rxintrwhen;
    if (rxIntrEvent.scheduled())
        rxintrwhen = rxIntrEvent.when();
    else
        rxintrwhen = 0;
    Tick txintrwhen;
    if (txIntrEvent.scheduled())
        txintrwhen = txIntrEvent.when();
    else
        txintrwhen = 0;
     SERIALIZE_SCALAR(rxintrwhen);
     SERIALIZE_SCALAR(txintrwhen);
}

void
Uart8250::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(status);
    paramIn(cp, "IER", registers.ier);
    paramIn(cp, "LCR", registers.lcr);
    paramIn(cp, "MCR", registers.mcr);
    Tick rxintrwhen;
    Tick txintrwhen;
    UNSERIALIZE_SCALAR(rxintrwhen);
    UNSERIALIZE_SCALAR(txintrwhen);
    if (rxintrwhen != 0)
        schedule(rxIntrEvent, rxintrwhen);
    if (txintrwhen != 0)
        schedule(txIntrEvent, txintrwhen);
}

} // namespace gem5
