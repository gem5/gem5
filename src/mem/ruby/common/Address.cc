/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#include "mem/ruby/common/Address.hh"

#include "mem/ruby/system/RubySystem.hh"

Addr
bitSelect(Addr addr, unsigned int small, unsigned int big)
{
    assert(big >= small);

    if (big >= ADDRESS_WIDTH - 1) {
        return (addr >> small);
    } else {
        Addr mask = ~((Addr)~0 << (big + 1));
        // FIXME - this is slow to manipulate a 64-bit number using 32-bits
        Addr partial = (addr & mask);
        return (partial >> small);
    }
}

Addr
bitRemove(Addr addr, unsigned int small, unsigned int big)
{
    assert(big >= small);

    if (small >= ADDRESS_WIDTH - 1) {
        return addr;
    } else if (big >= ADDRESS_WIDTH - 1) {
        Addr mask = (Addr)~0 >> small;
        return (addr & mask);
    } else if (small == 0) {
        Addr mask = (Addr)~0 << big;
        return (addr & mask);
    } else {
        Addr mask = ~((Addr)~0 << small);
        Addr lower_bits = addr & mask;
        mask = (Addr)~0 << (big + 1);
        Addr higher_bits = addr & mask;

        // Shift the valid high bits over the removed section
        higher_bits = higher_bits >> (big - small + 1);
        return (higher_bits | lower_bits);
    }
}

Addr
maskLowOrderBits(Addr addr, unsigned int number)
{
  Addr mask;

  if (number >= ADDRESS_WIDTH - 1) {
      mask = ~0;
  } else {
      mask = (Addr)~0 << number;
  }
  return (addr & mask);
}

Addr
maskHighOrderBits(Addr addr, unsigned int number)
{
    Addr mask;

    if (number >= ADDRESS_WIDTH - 1) {
        mask = ~0;
    } else {
        mask = (Addr)~0 >> number;
    }
    return (addr & mask);
}

Addr
shiftLowOrderBits(Addr addr, unsigned int number)
{
    return (addr >> number);
}

Addr
getOffset(Addr addr)
{
    return bitSelect(addr, 0, RubySystem::getBlockSizeBits() - 1);
}

Addr
makeLineAddress(Addr addr)
{
    return maskLowOrderBits(addr, RubySystem::getBlockSizeBits());
}

// returns the next stride address based on line address
Addr
makeNextStrideAddress(Addr addr, int stride)
{
    return maskLowOrderBits(addr, RubySystem::getBlockSizeBits())
        + RubySystem::getBlockSizeBytes() * stride;
}

std::string
printAddress(Addr addr)
{
    std::stringstream out;
    out << "[" << std::hex << "0x" << addr << "," << " line 0x"
       << maskLowOrderBits(addr, RubySystem::getBlockSizeBits())
       << std::dec << "]";
    return out.str();
}
