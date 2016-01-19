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

#include <inttypes.h>

#include <cassert>
#include <iomanip>
#include <iostream>

class WriteMask;

class DataBlock
{
  public:
    DataBlock()
    {
        alloc();
    }

    DataBlock(const DataBlock &cp);

    ~DataBlock()
    {
        if (m_alloc)
            delete [] m_data;
    }

    DataBlock& operator=(const DataBlock& obj);

    void assign(uint8_t *data);

    void clear();
    uint8_t getByte(int whichByte) const;
    const uint8_t *getData(int offset, int len) const;
    uint8_t *getDataMod(int offset);
    void setByte(int whichByte, uint8_t data);
    void setData(const uint8_t *data, int offset, int len);
    void copyPartial(const DataBlock &dblk, int offset, int len);
    void copyPartial(const DataBlock &dblk, const WriteMask &mask);
    void atomicPartial(const DataBlock & dblk, const WriteMask & mask);
    bool equal(const DataBlock& obj) const;
    void print(std::ostream& out) const;

  private:
    void alloc();
    uint8_t *m_data;
    bool m_alloc;
};

inline void
DataBlock::assign(uint8_t *data)
{
    assert(data != NULL);
    if (m_alloc) {
        delete [] m_data;
    }
    m_data = data;
    m_alloc = false;
}

inline uint8_t
DataBlock::getByte(int whichByte) const
{
    return m_data[whichByte];
}

inline void
DataBlock::setByte(int whichByte, uint8_t data)
{
    m_data[whichByte] = data;
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
