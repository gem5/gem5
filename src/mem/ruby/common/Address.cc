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

#include "base/bitfield.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

Addr
bitSelect(Addr addr, unsigned int small, unsigned int big)
{
    assert(big >= small);
    return bits<Addr>(addr, big, small);
}

Addr
maskLowOrderBits(Addr addr, unsigned int number)
{
    return mbits<Addr>(addr, 63, number);
}

Addr
getOffset(Addr addr)
{
    return bitSelect(addr, 0, RubySystem::getBlockSizeBits() - 1);
}

Addr
makeLineAddress(Addr addr)
{
    return mbits<Addr>(addr, 63, RubySystem::getBlockSizeBits());
}

Addr
makeLineAddress(Addr addr, int cacheLineBits)
{
    return maskLowOrderBits(addr, cacheLineBits);
}

// returns the next stride address based on line address
Addr
makeNextStrideAddress(Addr addr, int stride)
{
    return makeLineAddress(addr) +
           static_cast<int>(RubySystem::getBlockSizeBytes()) * stride;
}

std::string
printAddress(Addr addr)
{
    std::stringstream out;
    out << "[" << std::hex << "0x" << addr << ","
        << " line 0x" << makeLineAddress(addr) << std::dec << "]";
    return out.str();
}

} // namespace ruby
} // namespace gem5
