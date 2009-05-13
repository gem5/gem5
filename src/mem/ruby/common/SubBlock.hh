
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

/*
 * $Id$
 *
 */

#ifndef SubBlock_H
#define SubBlock_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/gems_common/Vector.hh"

class SubBlock {
public:
  // Constructors
  SubBlock() { }
  SubBlock(const Address& addr, int size);
  SubBlock(const Address& addr, const Address& logicalAddress, int size);

  // Destructor
  ~SubBlock() { }

  // Public Methods
  const Address& getAddress() const { return m_address; }
  const Address& getLogicalAddress() const { return m_logicalAddress; }
  void setAddress(const Address& addr) { m_address = addr; }
  void setLogicalAddress(const Address& addr) { m_logicalAddress = addr; }

  int getSize() const { return m_data.size(); }
  void setSize(int size) {  m_data.setSize(size); }
  uint8 getByte(int offset) const { return m_data[offset]; }
  void setByte(int offset, uint8 data) { m_data[offset] = data; }

  // Shorthands
  uint8 readByte() const { return getByte(0); }
  void writeByte(uint8 data) { setByte(0, data); }

  // Merging to and from DataBlocks - We only need to worry about
  // updates when we are using DataBlocks
  void mergeTo(DataBlock& data) const { if (DATA_BLOCK) { internalMergeTo(data); } }
  void mergeFrom(const DataBlock& data) { if (DATA_BLOCK) { internalMergeFrom(data); } }

  void print(ostream& out) const;
private:
  // Private Methods
  //  SubBlock(const SubBlock& obj);
  //  SubBlock& operator=(const SubBlock& obj);
  //  bool bytePresent(const Address& addr) { return ((addr.getAddress() >= m_address.getAddress()) && (addr.getAddress() < (m_address.getAddress()+getSize()))); }
  //  uint8 getByte(const Address& addr) { return m_data[addr.getAddress() - m_address.getAddress()]; }

  void internalMergeTo(DataBlock& data) const;
  void internalMergeFrom(const DataBlock& data);

  // Data Members (m_ prefix)
  Address m_address;
  Address m_logicalAddress;
  Vector<unsigned> m_data;
};

// Output operator declaration
ostream& operator<<(ostream& out, const SubBlock& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const SubBlock& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SubBlock_H
