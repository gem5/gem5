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

#ifndef __MEM_RUBY_COMMON_SUBBLOCK_HH__
#define __MEM_RUBY_COMMON_SUBBLOCK_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"

namespace gem5
{

namespace ruby
{

class SubBlock
{
  public:
    SubBlock() { }
    SubBlock(Addr addr, int size);
    ~SubBlock() { }

    Addr getAddress() const { return m_address; }
    void setAddress(Addr addr) { m_address = addr; }

    int getSize() const { return m_data.size(); }
    void resize(int size) {  m_data.resize(size); }
    uint8_t getByte(int offset) const { return m_data[offset]; }
    void setByte(int offset, uint8_t data) { m_data[offset] = data; }

    // Shorthands
    uint8_t readByte() const { return getByte(0); }
    void writeByte(uint8_t data) { setByte(0, data); }

    // Merging to and from DataBlocks - We only need to worry about
    // updates when we are using DataBlocks
    void mergeTo(DataBlock& data) const { internalMergeTo(data); }
    void mergeFrom(const DataBlock& data) { internalMergeFrom(data); }

    void print(std::ostream& out) const;

  private:
    void internalMergeTo(DataBlock& data) const;
    void internalMergeFrom(const DataBlock& data);

    // Data Members (m_ prefix)
    Addr m_address;
    std::vector<uint8_t> m_data;
};

inline std::ostream&
operator<<(std::ostream& out, const SubBlock& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_COMMON_SUBBLOCK_HH__
