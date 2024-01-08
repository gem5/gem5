/*
 * Copyright (c) 2021 ARM Limited
 * All rights reserved.
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

#include "mem/ruby/common/DataBlock.hh"

#include "mem/ruby/common/WriteMask.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

DataBlock::DataBlock(const DataBlock &cp)
{
    m_data = new uint8_t[RubySystem::getBlockSizeBytes()];
    memcpy(m_data, cp.m_data, RubySystem::getBlockSizeBytes());
    m_alloc = true;
}

void
DataBlock::alloc()
{
    m_data = new uint8_t[RubySystem::getBlockSizeBytes()];
    m_alloc = true;
    clear();
}

void
DataBlock::clear()
{
    memset(m_data, 0, RubySystem::getBlockSizeBytes());
}

bool
DataBlock::equal(const DataBlock& obj) const
{
    return !memcmp(m_data, obj.m_data, RubySystem::getBlockSizeBytes());
}

void
DataBlock::copyPartial(const DataBlock &dblk, const WriteMask &mask)
{
    for (int i = 0; i < RubySystem::getBlockSizeBytes(); i++) {
        if (mask.getMask(i, 1)) {
            m_data[i] = dblk.m_data[i];
        }
    }
}

void
DataBlock::atomicPartial(const DataBlock &dblk, const WriteMask &mask)
{
    for (int i = 0; i < RubySystem::getBlockSizeBytes(); i++) {
        m_data[i] = dblk.m_data[i];
    }
    mask.performAtomic(m_data);
}

void
DataBlock::print(std::ostream& out) const
{
    int size = RubySystem::getBlockSizeBytes();
    out << "[ ";
    for (int i = 0; i < size; i++) {
        out << std::setw(2) << std::setfill('0') << std::hex
            << "0x" << (int)m_data[i] << " " << std::setfill(' ');
    }
    out << std::dec << "]" << std::flush;
}

const uint8_t*
DataBlock::getData(int offset, int len) const
{
    assert(offset + len <= RubySystem::getBlockSizeBytes());
    return &m_data[offset];
}

uint8_t*
DataBlock::getDataMod(int offset)
{
    return &m_data[offset];
}

void
DataBlock::setData(const uint8_t *data, int offset, int len)
{
    memcpy(&m_data[offset], data, len);
}

void
DataBlock::setData(PacketPtr pkt)
{
    int offset = getOffset(pkt->getAddr());
    assert(offset + pkt->getSize() <= RubySystem::getBlockSizeBytes());
    pkt->writeData(&m_data[offset]);
}

DataBlock &
DataBlock::operator=(const DataBlock & obj)
{
    memcpy(m_data, obj.m_data, RubySystem::getBlockSizeBytes());
    return *this;
}

} // namespace ruby
} // namespace gem5
