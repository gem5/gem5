/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

/* @file
 * Implements a 8250 UART
 */

#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/str.hh"        // for to_number
#include "base/trace.hh"
#include "dev/simconsole.hh"
#include "dev/uart.hh"
#include "dev/platform.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"

using namespace std;

Uart::IntrEvent::IntrEvent(Uart *u, int bit)
    : Event(&mainEventQueue), uart(u)
{
    DPRINTF(Uart, "UART Interrupt Event Initilizing\n");
    intrBit = bit;
}

const char *
Uart::IntrEvent::description()
{
    return "uart interrupt delay event";
}

void
Uart::IntrEvent::process()
{
    if (intrBit & uart->IER) {
       DPRINTF(Uart, "UART InterEvent, interrupting\n");
       uart->platform->postConsoleInt();
       uart->status |= intrBit;
    }
    else
       DPRINTF(Uart, "UART InterEvent, not interrupting\n");

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
Uart::IntrEvent::scheduleIntr()
{
    static const Tick interval = (Tick)((Clock::Float::s / 2e9) * 450);
    DPRINTF(Uart, "Scheduling IER interrupt for %#x, at cycle %lld\n", intrBit,
            curTick + interval);
    if (!scheduled())
        schedule(curTick + interval);
    else
        reschedule(curTick + interval);
}

Uart::Uart(const string &name, SimConsole *c, MemoryController *mmu, Addr a,
           Addr s, HierParams *hier, Bus *bus, Tick pio_latency, Platform *p)
    : PioDevice(name, p), addr(a), size(s), cons(c),
      txIntrEvent(this, TX_INT), rxIntrEvent(this, RX_INT)
{
    mmu->add_child(this, RangeSize(addr, size));


    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &Uart::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * bus->clockRate;
    }

    readAddr = 0;
    IER = 0;
    DLAB = 0;
    LCR = 0;
    MCR = 0;
    status = 0;

    // set back pointers
    cons->uart = this;
    platform->uart = this;

}

Fault
Uart::read(MemReqPtr &req, uint8_t *data)
{
    Addr daddr = req->paddr - (addr & EV5::PAddrImplMask);
    DPRINTF(Uart, " read register %#x\n", daddr);



#ifdef ALPHA_TLASER

    switch (req->size) {
      case sizeof(uint64_t):
        *(uint64_t *)data = 0;
        break;
      case sizeof(uint32_t):
        *(uint32_t *)data = 0;
        break;
      case sizeof(uint16_t):
        *(uint16_t *)data = 0;
        break;
      case sizeof(uint8_t):
        *(uint8_t *)data = 0;
        break;
    }

    switch (daddr) {
      case 0x80: // Status Register
        if (readAddr == 3) {
            readAddr = 0;
            if (status & TX_INT)
                *data = (1 << 4);
             else if (status & RX_INT)
                *data = (1 << 5);
             else
                DPRINTF(Uart, "spurious read\n");

        } else {
            *data = (1 << 2);
            if (status & RX_INT)
                *data |= (1 << 0);
        }
        break;

      case 0xc0: // Data register (RX)
        if (!cons->dataAvailable())
            panic("No data to read");

        cons->in(*data);

        if (!cons->dataAvailable()) {
            platform->clearConsoleInt();
            status &= ~RX_INT;
        }

        DPRINTF(Uart, "read data register \'%c\' %2x\n",
                isprint(*data) ? *data : ' ', *data);
        break;
    }


#else


    assert(req->size == 1);

    switch (daddr) {
        case 0x0:
            if (!(LCR & 0x80)) { // read byte
                if (cons->dataAvailable())
                    cons->in(*data);
                else {
                    *(uint8_t*)data = 0;
                    // A limited amount of these are ok.
                    DPRINTF(Uart, "empty read of RX register\n");
                }
                status &= ~RX_INT;
                platform->clearConsoleInt();

                if (cons->dataAvailable() && (IER & UART_IER_RDI))
                    rxIntrEvent.scheduleIntr();
            } else { // dll divisor latch
               ;
            }
            break;
        case 0x1:
            if (!(LCR & 0x80)) { // Intr Enable Register(IER)
                *(uint8_t*)data = IER;
            } else { // DLM divisor latch MSB
                ;
            }
            break;
        case 0x2: // Intr Identification Register (IIR)
            DPRINTF(Uart, "IIR Read, status = %#x\n", (uint32_t)status);
            if (status)
                *(uint8_t*)data = 0;
            else
                *(uint8_t*)data = 1;
            break;
        case 0x3: // Line Control Register (LCR)
            *(uint8_t*)data = LCR;
            break;
        case 0x4: // Modem Control Register (MCR)
            break;
        case 0x5: // Line Status Register (LSR)
            uint8_t lsr;
            lsr = 0;
            // check if there are any bytes to be read
            if (cons->dataAvailable())
                lsr = UART_LSR_DR;
            lsr |= UART_LSR_TEMT | UART_LSR_THRE;
            *(uint8_t*)data = lsr;
            break;
        case 0x6: // Modem Status Register (MSR)
            *(uint8_t*)data = 0;
            break;
        case 0x7: // Scratch Register (SCR)
            *(uint8_t*)data = 0; // doesn't exist with at 8250.
            break;
        default:
            panic("Tried to access a UART port that doesn't exist\n");
            break;
    }

#endif
    return No_Fault;

}

Fault
Uart::write(MemReqPtr &req, const uint8_t *data)
{
    Addr daddr = req->paddr - (addr & EV5::PAddrImplMask);

    DPRINTF(Uart, " write register %#x value %#x\n", daddr, *(uint8_t*)data);

#ifdef ALPHA_TLASER

    switch (daddr) {
      case 0x80:
        readAddr = *data;
        switch (*data) {
          case 0x28: // Ack of TX
              if ((status & TX_INT) == 0)
                  panic("Ack of transmit, though there was no interrupt");

              status &= ~TX_INT;
              platform->clearConsoleInt();
              break;
          case 0x00:
          case 0x01:
          case 0x03: // going to read RR3
          case 0x12:
                break;
          default:
                DPRINTF(Uart, "writing status register %#x \n",
                        *(uint64_t *)data);
                break;
        }
        break;

      case 0xc0: // Data register (TX)
        cons->out(*(uint64_t *)data);
        platform->postConsoleInt();
        status |= TX_INT;
        break;
    }


#else
    switch (daddr) {
        case 0x0:
            if (!(LCR & 0x80)) { // write byte
                cons->out(*(uint8_t *)data);
                platform->clearConsoleInt();
                status &= ~TX_INT;
                if (UART_IER_THRI & IER)
                    txIntrEvent.scheduleIntr();
            } else { // dll divisor latch
               ;
            }
            break;
        case 0x1:
            if (!(LCR & 0x80)) { // Intr Enable Register(IER)
                IER = *(uint8_t*)data;
                if (UART_IER_THRI & IER)
                {
                    DPRINTF(Uart, "IER: IER_THRI set, scheduling TX intrrupt\n");
                    txIntrEvent.scheduleIntr();
                }
                else
                {
                    DPRINTF(Uart, "IER: IER_THRI cleared, descheduling TX intrrupt\n");
                    if (txIntrEvent.scheduled())
                        txIntrEvent.deschedule();
                    if (status & TX_INT)
                        platform->clearConsoleInt();
                    status &= ~TX_INT;
                }

                if ((UART_IER_RDI & IER) && cons->dataAvailable()) {
                    DPRINTF(Uart, "IER: IER_RDI set, scheduling RX intrrupt\n");
                    rxIntrEvent.scheduleIntr();
                } else {
                    DPRINTF(Uart, "IER: IER_RDI cleared, descheduling RX intrrupt\n");
                    if (rxIntrEvent.scheduled())
                        rxIntrEvent.deschedule();
                    if (status & RX_INT)
                        platform->clearConsoleInt();
                    status &= ~RX_INT;
                }
             } else { // DLM divisor latch MSB
                ;
            }
            break;
        case 0x2: // FIFO Control Register (FCR)
            break;
        case 0x3: // Line Control Register (LCR)
            LCR = *(uint8_t*)data;
            break;
        case 0x4: // Modem Control Register (MCR)
            if (*(uint8_t*)data == (UART_MCR_LOOP | 0x0A))
                    MCR = 0x9A;
            break;
        case 0x7: // Scratch Register (SCR)
            // We are emulating a 8250 so we don't have a scratch reg
            break;
        default:
            panic("Tried to access a UART port that doesn't exist\n");
            break;
    }
#endif

    return No_Fault;
}

void
Uart::dataAvailable()
{
#ifdef ALPHA_TLASER
        platform->postConsoleInt();
        status |= RX_INT;
#else

    // if the kernel wants an interrupt when we have data
    if (IER & UART_IER_RDI)
    {
        platform->postConsoleInt();
        status |= RX_INT;
    }

#endif
}

Tick
Uart::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}

void
Uart::serialize(ostream &os)
{
#ifdef ALPHA_TLASER
    SERIALIZE_SCALAR(readAddr);
    SERIALIZE_SCALAR(status);
#else
    SERIALIZE_SCALAR(status);
    SERIALIZE_SCALAR(IER);
    SERIALIZE_SCALAR(DLAB);
    SERIALIZE_SCALAR(LCR);
    SERIALIZE_SCALAR(MCR);
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
#endif
}

void
Uart::unserialize(Checkpoint *cp, const std::string &section)
{
#ifdef ALPHA_TLASER
    UNSERIALIZE_SCALAR(readAddr);
    UNSERIALIZE_SCALAR(status);
#else
    UNSERIALIZE_SCALAR(status);
    UNSERIALIZE_SCALAR(IER);
    UNSERIALIZE_SCALAR(DLAB);
    UNSERIALIZE_SCALAR(LCR);
    UNSERIALIZE_SCALAR(MCR);
    Tick rxintrwhen;
    Tick txintrwhen;
    UNSERIALIZE_SCALAR(rxintrwhen);
    UNSERIALIZE_SCALAR(txintrwhen);
    if (rxintrwhen != 0)
        rxIntrEvent.schedule(rxintrwhen);
    if (txintrwhen != 0)
        txIntrEvent.schedule(txintrwhen);
#endif

}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Uart)

    SimObjectParam<SimConsole *> console;
    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<Platform *> platform;
    Param<Addr> addr;
    Param<Addr> size;
    SimObjectParam<Bus*> io_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;


END_DECLARE_SIM_OBJECT_PARAMS(Uart)

BEGIN_INIT_SIM_OBJECT_PARAMS(Uart)

    INIT_PARAM(console, "The console"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(platform, "Pointer to platfrom"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(size, "Device size", 0x8),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(Uart)

CREATE_SIM_OBJECT(Uart)
{
    return new Uart(getInstanceName(), console, mmu, addr, size, hier, io_bus,
                    pio_latency, platform);
}

REGISTER_SIM_OBJECT("Uart", Uart)
