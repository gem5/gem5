/*
 * Copyright (c) 2021 The Regents of the University of California
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

#include "dev/lupio/lupio_tty.hh"

#include "base/trace.hh"
#include "debug/LupioTTY.hh"
#include "dev/platform.hh"
#include "mem/packet_access.hh"
#include "params/LupioTTY.hh"

#define LUPIO_TTY_INVAL 0x80000000

/* Same fields for CTRL and STAT registers */
#define LUPIO_TTY_WBIT (1 << 0)
#define LUPIO_TTY_RBIT (1 << 1)

namespace gem5
{

LupioTTY::LupioTTY(const Params &params)
    : BasicPioDevice(params, params.pio_size),
      writChar(-1),
      readChar(-1),
      writIntrEn(false),
      readIntrEn(false),
      terminal(params.terminal),
      platform(params.platform)
{
    // setup serial device callbacks
    terminal->regInterfaceCallback([this]() { dataAvailable(); });
}

void
LupioTTY::lupioTTYUpdateIRQ()
{
    unsigned int irq;

    irq = (writIntrEn && writChar != -1) || (readIntrEn && readChar != -1);

    if (irq) {
        DPRINTF(LupioTTY, "LupioTTY InterEvent, interrupting\n");
        platform->postConsoleInt();
    } else {
        DPRINTF(LupioTTY, "LupioTTY InterEvent, not interrupting\n");
        platform->clearConsoleInt();
    }
}

void
LupioTTY::dataAvailable()
{
    gem5_assert(terminal->dataAvailable());

    // prevent from overwritting an unread character
    if (readChar != -1) {
        fprintf(stderr, "Dropping characters\n");
    } else {
        // read data from the terminal
        readChar = terminal->readData();
        lupioTTYUpdateIRQ();
    }
}

uint64_t
LupioTTY::lupioTTYRead(uint8_t addr)
{
    uint32_t ret = LUPIO_TTY_INVAL;

    switch (addr >> 2) {
    case LUPIO_TTY_WRIT:
        DPRINTF(LupioTTY, "Accessing LUPIO_TTY_WRIT\n");
        if (writChar != -1) {
            ret = writChar;
            writChar = -1;
            lupioTTYUpdateIRQ();
        }
        break;
    case LUPIO_TTY_READ:
        DPRINTF(LupioTTY, "Accessing LUPIO_TTY_READ\n");
        if (readChar != -1) {
            // return new character
            ret = readChar;
            readChar = -1;
            lupioTTYUpdateIRQ();
        }
        break;
    case LUPIO_TTY_CTRL:
        DPRINTF(LupioTTY, "Accessing LUPIO_TTY_CTRL\n");
        ret = 0;
        if (writIntrEn) {
            ret |= LUPIO_TTY_WBIT;
        }
        if (readIntrEn) {
            ret |= LUPIO_TTY_RBIT;
        }
        break;
    case LUPIO_TTY_STAT:
        DPRINTF(LupioTTY, "Accessing LUPIO_TTY_STAT\n");
        // always ready to write
        ret = LUPIO_TTY_WBIT;
        // ready to read if unread character available
        if (readChar != -1) {
            ret |= LUPIO_TTY_RBIT;
        }
        break;
    default:
        panic("Unexpected read to the LupioTTY device at address %d!", addr);
        break;
    }

    return ret;
}

void
LupioTTY::lupioTTYWrite(uint8_t addr, uint64_t val64)
{
    uint32_t val = val64;
    uint8_t c = val;

    switch (addr >> 2) {
    case LUPIO_TTY_WRIT:
        DPRINTF(LupioTTY, "Accessing Write: LUPIO_TTY_WRIT: %d\n", c);
        // write data to terminal
        terminal->writeData(c);
        writChar = c;
        lupioTTYUpdateIRQ();
        break;
    case LUPIO_TTY_CTRL:
        // set interrupt enable bits
        DPRINTF(LupioTTY, "Accessing LUPIO_TTY_CTRL\n");
        writIntrEn = val & LUPIO_TTY_WBIT;
        readIntrEn = val & LUPIO_TTY_RBIT;
        lupioTTYUpdateIRQ();
        break;
    default:
        panic("Unexpected write to the LupioTTY device at address %d!", addr);
        break;
    }
}

Tick
LupioTTY::read(PacketPtr pkt)
{
    Addr tty_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioTTY, "Read request - addr: %#x, size: %#x\n", tty_addr,
            pkt->getSize());

    uint64_t val = lupioTTYRead(tty_addr);
    pkt->setUintX(val, byteOrder);

    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioTTY::write(PacketPtr pkt)
{
    Addr tty_addr = pkt->getAddr() - pioAddr;
    DPRINTF(LupioTTY, "Write request - addr: %#x pktAddr: %#x value: %c\n",
            tty_addr, pkt->getAddr(), pkt->getUintX(byteOrder));

    lupioTTYWrite(tty_addr, pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}

} // namespace gem5
