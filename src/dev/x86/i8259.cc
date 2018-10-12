/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "dev/x86/i8259.hh"

#include "base/bitfield.hh"
#include "debug/I8259.hh"
#include "dev/x86/i82094aa.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

X86ISA::I8259::I8259(Params * p)
    : BasicPioDevice(p, 2), IntDevice(this),
      latency(p->pio_latency), output(p->output),
      mode(p->mode), slave(p->slave),
      IRR(0), ISR(0), IMR(0),
      readIRR(true), initControlWord(0), autoEOI(false)
{
    for (int i = 0; i < NumLines; i++)
        pinStates[i] = false;
}

Tick
X86ISA::I8259::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - pioAddr)
    {
      case 0x0:
        if (readIRR) {
            DPRINTF(I8259, "Reading IRR as %#x.\n", IRR);
            pkt->setLE(IRR);
        } else {
            DPRINTF(I8259, "Reading ISR as %#x.\n", ISR);
            pkt->setLE(ISR);
        }
        break;
      case 0x1:
        DPRINTF(I8259, "Reading IMR as %#x.\n", IMR);
        pkt->setLE(IMR);
        break;
    }
    pkt->makeAtomicResponse();
    return latency;
}

Tick
X86ISA::I8259::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    uint8_t val = pkt->getLE<uint8_t>();
    switch (pkt->getAddr() - pioAddr) {
      case 0x0:
        if (bits(val, 4)) {
            DPRINTF(I8259, "Received initialization command word 1.\n");
            IMR = 0;
            edgeTriggered = bits(val, 3);
            DPRINTF(I8259, "%s triggered mode.\n",
                    edgeTriggered ? "Edge" : "Level");
            cascadeMode = !bits(val, 1);
            DPRINTF(I8259, "%s mode.\n",
                    cascadeMode ? "Cascade" : "Single");
            expectICW4 = bits(val, 0);
            if (!expectICW4) {
                autoEOI = false;
            }
            initControlWord = 1;
            DPRINTF(I8259, "Expecting %d more bytes.\n", expectICW4 ? 3 : 2);
        } else if (bits(val, 4, 3) == 0) {
            DPRINTF(I8259, "Received operation command word 2.\n");
            switch (bits(val, 7, 5)) {
              case 0x0:
                DPRINTF(I8259,
                        "Subcommand: Rotate in auto-EOI mode (clear).\n");
                break;
              case 0x1:
                {
                    int line = findMsbSet(ISR);
                    DPRINTF(I8259, "Subcommand: Nonspecific EOI on line %d.\n",
                            line);
                    handleEOI(line);
                }
                break;
              case 0x2:
                DPRINTF(I8259, "Subcommand: No operation.\n");
                break;
              case 0x3:
                {
                    int line = bits(val, 2, 0);
                    DPRINTF(I8259, "Subcommand: Specific EIO on line %d.\n",
                            line);
                    handleEOI(line);
                }
                break;
              case 0x4:
                DPRINTF(I8259, "Subcommand: Rotate in auto-EOI mode (set).\n");
                break;
              case 0x5:
                DPRINTF(I8259, "Subcommand: Rotate on nonspecific EOI.\n");
                break;
              case 0x6:
                DPRINTF(I8259, "Subcommand: Set priority command.\n");
                DPRINTF(I8259, "Lowest: IRQ%d   Highest IRQ%d.\n",
                        bits(val, 2, 0), (bits(val, 2, 0) + 1) % 8);
                break;
              case 0x7:
                DPRINTF(I8259, "Subcommand: Rotate on specific EOI.\n");
                DPRINTF(I8259, "Lowest: IRQ%d   Highest IRQ%d.\n",
                        bits(val, 2, 0), (bits(val, 2, 0) + 1) % 8);
                break;
            }
        } else if (bits(val, 4, 3) == 1) {
            DPRINTF(I8259, "Received operation command word 3.\n");
            if (bits(val, 7)) {
                DPRINTF(I8259, "%s special mask mode.\n",
                        bits(val, 6) ? "Set" : "Clear");
            }
            if (bits(val, 1)) {
                readIRR = bits(val, 0);
                DPRINTF(I8259, "Read %s.\n", readIRR ? "IRR" : "ISR");
            }
        }
        break;
      case 0x1:
        switch (initControlWord) {
          case 0x0:
            DPRINTF(I8259, "Received operation command word 1.\n");
            DPRINTF(I8259, "Wrote IMR value %#x.\n", val);
            IMR = val;
            break;
          case 0x1:
            DPRINTF(I8259, "Received initialization command word 2.\n");
            vectorOffset = val & ~mask(3);
            DPRINTF(I8259, "Responsible for vectors %#x-%#x.\n",
                    vectorOffset, vectorOffset | mask(3));
            if (cascadeMode) {
                initControlWord++;
            } else {
                cascadeBits = 0;
                initControlWord = 0;
            }
            break;
          case 0x2:
            DPRINTF(I8259, "Received initialization command word 3.\n");
            if (mode == Enums::I8259Master) {
                DPRINTF(I8259, "Slaves attached to IRQs:%s%s%s%s%s%s%s%s\n",
                        bits(val, 0) ? " 0" : "",
                        bits(val, 1) ? " 1" : "",
                        bits(val, 2) ? " 2" : "",
                        bits(val, 3) ? " 3" : "",
                        bits(val, 4) ? " 4" : "",
                        bits(val, 5) ? " 5" : "",
                        bits(val, 6) ? " 6" : "",
                        bits(val, 7) ? " 7" : "");
                cascadeBits = val;
            } else {
                DPRINTF(I8259, "Slave ID is %d.\n", val & mask(3));
                cascadeBits = val & mask(3);
            }
            if (expectICW4)
                initControlWord++;
            else
                initControlWord = 0;
            break;
          case 0x3:
            DPRINTF(I8259, "Received initialization command word 4.\n");
            if (bits(val, 4)) {
                DPRINTF(I8259, "Special fully nested mode.\n");
            } else {
                DPRINTF(I8259, "Not special fully nested mode.\n");
            }
            if (bits(val, 3) == 0) {
                DPRINTF(I8259, "Nonbuffered.\n");
            } else if (bits(val, 2) == 0) {
                DPRINTF(I8259, "Buffered.\n");
            } else {
                DPRINTF(I8259, "Unrecognized buffer mode.\n");
            }
            autoEOI = bits(val, 1);
            DPRINTF(I8259, "%s End Of Interrupt.\n",
                    autoEOI ? "Automatic" : "Normal");

            DPRINTF(I8259, "%s mode.\n", bits(val, 0) ? "80x86" : "MCX-80/85");
            initControlWord = 0;
            break;
        }
        break;
    }
    pkt->makeAtomicResponse();
    return latency;
}

void
X86ISA::I8259::handleEOI(int line)
{
    ISR &= ~(1 << line);
    // There may be an interrupt that was waiting which can
    // now be sent.
    if (IRR)
        requestInterrupt(findMsbSet(IRR));
}

void
X86ISA::I8259::requestInterrupt(int line)
{
    if (bits(ISR, 7, line) == 0) {
        if (output) {
            DPRINTF(I8259, "Propogating interrupt.\n");
            output->raise();
            //XXX This is a hack.
            output->lower();
        } else {
            warn("Received interrupt but didn't have "
                    "anyone to tell about it.\n");
        }
    }
}

void
X86ISA::I8259::signalInterrupt(int line)
{
    DPRINTF(I8259, "Interrupt requested for line %d.\n", line);
    if (line >= NumLines)
        fatal("Line number %d doesn't exist. The max is %d.\n",
                line, NumLines - 1);
    if (bits(IMR, line)) {
        DPRINTF(I8259, "Interrupt %d was masked.\n", line);
    } else {
        IRR |= 1 << line;
        requestInterrupt(line);
    }
}

void
X86ISA::I8259::raiseInterruptPin(int number)
{
    DPRINTF(I8259, "Interrupt signal raised for pin %d.\n", number);
    if (number >= NumLines)
        fatal("Line number %d doesn't exist. The max is %d.\n",
                number, NumLines - 1);
    if (!pinStates[number])
        signalInterrupt(number);
    pinStates[number] = true;
}

void
X86ISA::I8259::lowerInterruptPin(int number)
{
    DPRINTF(I8259, "Interrupt signal lowered for pin %d.\n", number);
    if (number >= NumLines)
        fatal("Line number %d doesn't exist. The max is %d.\n",
                number, NumLines - 1);
    pinStates[number] = false;
}

int
X86ISA::I8259::getVector()
{
    /*
     * This code only handles one slave. Since that's how the PC platform
     * always uses the 8259 PIC, there shouldn't be any need for more. If
     * there -is- a need for more for some reason, "slave" can become a
     * vector of slaves.
     */
    int line = findMsbSet(IRR);
    IRR &= ~(1 << line);
    DPRINTF(I8259, "Interrupt %d was accepted.\n", line);
    if (autoEOI) {
        handleEOI(line);
    } else {
        ISR |= 1 << line;
    }
    if (slave && bits(cascadeBits, line)) {
        DPRINTF(I8259, "Interrupt was from slave who will "
                "provide the vector.\n");
        return slave->getVector();
    }
    return line | vectorOffset;
}

void
X86ISA::I8259::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(pinStates, NumLines);
    SERIALIZE_ENUM(mode);
    SERIALIZE_SCALAR(IRR);
    SERIALIZE_SCALAR(ISR);
    SERIALIZE_SCALAR(IMR);
    SERIALIZE_SCALAR(vectorOffset);
    SERIALIZE_SCALAR(cascadeMode);
    SERIALIZE_SCALAR(cascadeBits);
    SERIALIZE_SCALAR(edgeTriggered);
    SERIALIZE_SCALAR(readIRR);
    SERIALIZE_SCALAR(expectICW4);
    SERIALIZE_SCALAR(initControlWord);
    SERIALIZE_SCALAR(autoEOI);
}

void
X86ISA::I8259::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(pinStates, NumLines);
    UNSERIALIZE_ENUM(mode);
    UNSERIALIZE_SCALAR(IRR);
    UNSERIALIZE_SCALAR(ISR);
    UNSERIALIZE_SCALAR(IMR);
    UNSERIALIZE_SCALAR(vectorOffset);
    UNSERIALIZE_SCALAR(cascadeMode);
    UNSERIALIZE_SCALAR(cascadeBits);
    UNSERIALIZE_SCALAR(edgeTriggered);
    UNSERIALIZE_SCALAR(readIRR);
    UNSERIALIZE_SCALAR(expectICW4);
    UNSERIALIZE_SCALAR(initControlWord);
    UNSERIALIZE_SCALAR(autoEOI);
}

X86ISA::I8259 *
I8259Params::create()
{
    return new X86ISA::I8259(this);
}
