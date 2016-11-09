/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#include "dev/x86/i8237.hh"

#include "mem/packet.hh"
#include "mem/packet_access.hh"

Tick
X86ISA::I8237::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr offset = pkt->getAddr() - pioAddr;
    switch (offset) {
      case 0x0:
        panic("Read from i8237 channel 0 current address unimplemented.\n");
      case 0x1:
        panic("Read from i8237 channel 0 remaining "
                "word count unimplemented.\n");
      case 0x2:
        panic("Read from i8237 channel 1 current address unimplemented.\n");
      case 0x3:
        panic("Read from i8237 channel 1 remaining "
                "word count unimplemented.\n");
      case 0x4:
        panic("Read from i8237 channel 2 current address unimplemented.\n");
      case 0x5:
        panic("Read from i8237 channel 2 remaining "
                "word count unimplemented.\n");
      case 0x6:
        panic("Read from i8237 channel 3 current address unimplemented.\n");
      case 0x7:
        panic("Read from i8237 channel 3 remaining "
                "word count unimplemented.\n");
      case 0x8:
        panic("Read from i8237 status register unimplemented.\n");
      default:
        panic("Read from undefined i8237 register %d.\n", offset);
    }
    pkt->makeAtomicResponse();
    return latency;
}

Tick
X86ISA::I8237::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr offset = pkt->getAddr() - pioAddr;
    switch (offset) {
      case 0x0:
        panic("Write to i8237 channel 0 starting address unimplemented.\n");
      case 0x1:
        panic("Write to i8237 channel 0 starting "
                "word count unimplemented.\n");
      case 0x2:
        panic("Write to i8237 channel 1 starting address unimplemented.\n");
      case 0x3:
        panic("Write to i8237 channel 1 starting "
                "word count unimplemented.\n");
      case 0x4:
        panic("Write to i8237 channel 2 starting address unimplemented.\n");
      case 0x5:
        panic("Write to i8237 channel 2 starting "
                "word count unimplemented.\n");
      case 0x6:
        panic("Write to i8237 channel 3 starting address unimplemented.\n");
      case 0x7:
        panic("Write to i8237 channel 3 starting "
                "word count unimplemented.\n");
      case 0x8:
        panic("Write to i8237 command register unimplemented.\n");
      case 0x9:
        panic("Write to i8237 request register unimplemented.\n");
      case 0xa:
        {
            uint8_t command = pkt->get<uint8_t>();
            uint8_t select = bits(command, 1, 0);
            uint8_t bitVal = bits(command, 2);
            if (!bitVal)
                panic("Turning on i8237 channels unimplemented.\n");
            replaceBits(maskReg, select, bitVal);
        }
        break;
      case 0xb:
        panic("Write to i8237 mode register unimplemented.\n");
      case 0xc:
        panic("Write to i8237 clear LSB/MSB flip-flop "
                "register unimplemented.\n");
      case 0xd:
        panic("Write to i8237 master clear/reset register unimplemented.\n");
      case 0xe:
        panic("Write to i8237 clear mask register unimplemented.\n");
      case 0xf:
        panic("Write to i8237 write all mask register bits unimplemented.\n");
      default:
        panic("Write to undefined i8237 register.\n");
    }
    pkt->makeAtomicResponse();
    return latency;
}

void
X86ISA::I8237::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(maskReg);
}

void
X86ISA::I8237::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(maskReg);
}

X86ISA::I8237 *
I8237Params::create()
{
    return new X86ISA::I8237(this);
}
