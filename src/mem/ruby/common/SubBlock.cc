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

#include "mem/ruby/common/SubBlock.hh"

#include "base/stl_helpers.hh"

namespace gem5
{

namespace ruby
{

using stl_helpers::operator<<;

SubBlock::SubBlock(Addr addr, int size, int cl_bits)
{
    m_address = addr;
    resize(size);
    for (int i = 0; i < size; i++) {
        setByte(i, 0);
    }
    m_cache_line_bits = cl_bits;
}

void
SubBlock::internalMergeFrom(const DataBlock& data)
{
    int size = getSize();
    assert(size > 0);
    int offset = getOffset(m_address, m_cache_line_bits);
    for (int i = 0; i < size; i++) {
        this->setByte(i, data.getByte(offset + i));
    }
}

void
SubBlock::internalMergeTo(DataBlock& data) const
{
    int size = getSize();
    assert(size > 0);
    int offset = getOffset(m_address, m_cache_line_bits);
    for (int i = 0; i < size; i++) {
        // This will detect crossing a cache line boundary
        data.setByte(offset + i, this->getByte(i));
    }
}

void
SubBlock::print(std::ostream& out) const
{
    out << "[" << m_address << ", " << getSize() << ", " << m_data << "]";
}

} // namespace ruby
} // namespace gem5
