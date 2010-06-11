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

#ifndef __MEM_RUBY_COMMON_DATABLOCK_HH__
#define __MEM_RUBY_COMMON_DATABLOCK_HH__

#include <iomanip>
#include <iostream>

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/System.hh"

class DataBlock
{
  public:
    DataBlock()
    {
        alloc();
    }

    DataBlock(const DataBlock &cp)
    {
        m_data = new uint8[RubySystem::getBlockSizeBytes()];
        memcpy(m_data, cp.m_data, RubySystem::getBlockSizeBytes());
        m_alloc = true;
    }

    ~DataBlock()
    {
        if (m_alloc)
            delete [] m_data;
    }

    DataBlock& operator=(const DataBlock& obj);

    void assign(uint8* data);

    void clear();
    uint8 getByte(int whichByte) const;
    const uint8* getData(int offset, int len) const;
    void setByte(int whichByte, uint8 data);
    void setData(uint8* data, int offset, int len);
    void copyPartial(const DataBlock & dblk, int offset, int len);
    bool equal(const DataBlock& obj) const;
    void print(std::ostream& out) const;

  private:
    void alloc();
    uint8* m_data;
    bool m_alloc;
};

inline void
DataBlock::assign(uint8* data)
{
    if (m_alloc) {
        delete [] m_data;
    }
    m_data = data;
    m_alloc = false;
}

inline void
DataBlock::alloc()
{
    m_data = new uint8[RubySystem::getBlockSizeBytes()];
    m_alloc = true;
    clear();
}

inline void
DataBlock::clear()
{
    memset(m_data, 0, RubySystem::getBlockSizeBytes());
}

inline bool
DataBlock::equal(const DataBlock& obj) const
{
    return !memcmp(m_data, obj.m_data, RubySystem::getBlockSizeBytes());
}

inline void
DataBlock::print(std::ostream& out) const
{
    using namespace std;

    int size = RubySystem::getBlockSizeBytes();
    out << "[ ";
    for (int i = 0; i < size; i++) {
        out << setw(2) << setfill('0') << hex << "0x" << (int)m_data[i] << " ";
        out << setfill(' ');
    }
    out << dec << "]" << flush;
}

inline uint8
DataBlock::getByte(int whichByte) const
{
    return m_data[whichByte];
}

inline const uint8*
DataBlock::getData(int offset, int len) const
{
    assert(offset + len <= RubySystem::getBlockSizeBytes());
    return &m_data[offset];
}

inline void
DataBlock::setByte(int whichByte, uint8 data)
{
    m_data[whichByte] = data;
}

inline void
DataBlock::setData(uint8* data, int offset, int len)
{
    assert(offset + len <= RubySystem::getBlockSizeBytes());
    memcpy(&m_data[offset], data, len);
}

inline void
DataBlock::copyPartial(const DataBlock & dblk, int offset, int len)
{
    setData(&dblk.m_data[offset], offset, len);
}

inline std::ostream&
operator<<(std::ostream& out, const DataBlock& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

inline bool
operator==(const DataBlock& obj1,const DataBlock& obj2)
{
    return obj1.equal(obj2);
}

#endif // __MEM_RUBY_COMMON_DATABLOCK_HH__
