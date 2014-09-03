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

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/System.hh"

physical_address_t
Address::getLineAddress() const
{
    return bitSelect(RubySystem::getBlockSizeBits(), ADDRESS_WIDTH);
}

physical_address_t
Address::getOffset() const
{
    return bitSelect(0, RubySystem::getBlockSizeBits() - 1);
}

void
Address::makeLineAddress()
{
    m_address = maskLowOrderBits(RubySystem::getBlockSizeBits());
}

// returns the next stride address based on line address
void
Address::makeNextStrideAddress(int stride)
{
    m_address = maskLowOrderBits(RubySystem::getBlockSizeBits())
        + RubySystem::getBlockSizeBytes()*stride;
}

int64
Address::memoryModuleIndex() const
{
    int64 index =
        bitSelect(RubySystem::getBlockSizeBits() +
                  RubySystem::getMemorySizeBits(), ADDRESS_WIDTH);
    assert (index >= 0);
    return index;

    // int64 indexHighPortion =
    //     address.bitSelect(MEMORY_SIZE_BITS - 1,
    //                       PAGE_SIZE_BITS + NUMBER_OF_MEMORY_MODULE_BITS);
    // int64 indexLowPortion =
    //     address.bitSelect(DATA_BLOCK_BITS, PAGE_SIZE_BITS - 1);
    //
    // int64 index = indexLowPortion |
    //     (indexHighPortion << (PAGE_SIZE_BITS - DATA_BLOCK_BITS));

    /*
      Round-robin mapping of addresses, at page size granularity

ADDRESS_WIDTH    MEMORY_SIZE_BITS        PAGE_SIZE_BITS  DATA_BLOCK_BITS
  |                    |                       |               |
 \ /                  \ /                     \ /             \ /       0
  -----------------------------------------------------------------------
  |       unused        |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  |                     |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  -----------------------------------------------------------------------
                        indexHighPortion         indexLowPortion
                                        <------->
                               NUMBER_OF_MEMORY_MODULE_BITS
    */
}

void
Address::print(std::ostream& out) const
{
    using namespace std;
    out << "[" << hex << "0x" << m_address << "," << " line 0x"
        << maskLowOrderBits(RubySystem::getBlockSizeBits()) << dec << "]"
        << flush;
}

void
Address::output(std::ostream& out) const
{
    // Note: this outputs addresses in the form "ffff", not "0xffff".
    // This code should always be able to write out addresses in a
    // format that can be read in by the below input() method.  Please
    // don't change this without talking to Milo first.
    out << std::hex << m_address << std::dec;
}

void
Address::input(std::istream& in)
{
    // Note: this only works with addresses in the form "ffff", not
    // "0xffff".  This code should always be able to read in addresses
    // written out by the above output() method.  Please don't change
    // this without talking to Milo first.
    in >> std::hex >> m_address >> std::dec;
}

Address::Address(const Address& obj)
{
    m_address = obj.m_address;
}

Address&
Address::operator=(const Address& obj)
{
    if (this == &obj) {
        // assert(false);
    } else {
        m_address = obj.m_address;
    }
    return *this;
}

void
Address::makePageAddress()
{
    m_address = maskLowOrderBits(TheISA::PageShift);
}

Address
page_address(const Address& addr)
{
    Address temp = addr;
    temp.makePageAddress();
    return temp;
}

Address
next_stride_address(const Address& addr, int stride)
{
    Address temp = addr;
    temp.makeNextStrideAddress(stride);
    return temp;
}
