
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
 */

#include "DataBlock.hh"

DataBlock::DataBlock()
{
  if (DATA_BLOCK || XACT_MEMORY) {
    m_data.setSize(RubyConfig::dataBlockBytes());
  }
  clear();
}

DataBlock::~DataBlock()
{

}

void DataBlock::clear()
{
  int size = m_data.size();
  for (int i = 0; i < size; i++) {
    m_data[i] = 0;
  }
}

bool DataBlock::equal(const DataBlock& obj) const
{
  bool value = true;
  int size = m_data.size();
  for (int i = 0; i < size; i++) {
    value = value && (m_data[i] == obj.m_data[i]);
  }
  return value;
}

void DataBlock::print(ostream& out) const
{
  int size = m_data.size();
  for (int i = 0; i < size; i+=4) {
    out << hex << *((uint32*)(&(m_data[i]))) << " ";
  }
  out << dec << "]" << flush;
}

uint8 DataBlock::getByte(int whichByte) const
{
  if (DATA_BLOCK || XACT_MEMORY) {
  return m_data[whichByte];
  } else {
    return 0;
  }
}

void DataBlock::setByte(int whichByte, uint8 data)
{
  if (DATA_BLOCK || XACT_MEMORY) {
    m_data[whichByte] = data;
  }
}

