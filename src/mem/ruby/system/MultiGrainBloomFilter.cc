
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
 * MultiGrainBloomFilter.C
 *
 * Description:
 *
 *
 */

#include "MultiGrainBloomFilter.hh"
#include "Map.hh"
#include "Address.hh"

MultiGrainBloomFilter::MultiGrainBloomFilter(string str)
{
  string tail(str);

  // split into the 2 filter sizes
  string head = string_split(tail, '_');

  // head contains size of 1st bloom filter, tail contains size of 2nd bloom filter

  m_filter_size = atoi(head.c_str());
  m_filter_size_bits = log_int(m_filter_size);

  m_page_filter_size = atoi(tail.c_str());
  m_page_filter_size_bits = log_int(m_page_filter_size);

  m_filter.setSize(m_filter_size);
  m_page_filter.setSize(m_page_filter_size);
  clear();
}

MultiGrainBloomFilter::~MultiGrainBloomFilter(){
}

void MultiGrainBloomFilter::clear()
{
  for (int i = 0; i < m_filter_size; i++) {
    m_filter[i] = 0;
  }
  for(int i=0; i < m_page_filter_size; ++i){
    m_page_filter[i] = 0;
  }
}

void MultiGrainBloomFilter::increment(const Address& addr)
{
  // Not used
}


void MultiGrainBloomFilter::decrement(const Address& addr)
{
  // Not used
}

void MultiGrainBloomFilter::merge(AbstractBloomFilter * other_filter)
{
  // TODO
}

void MultiGrainBloomFilter::set(const Address& addr)
{
  int i = get_block_index(addr);
  int j = get_page_index(addr);
  assert(i < m_filter_size);
  assert(j < m_page_filter_size);
  m_filter[i] = 1;
  m_page_filter[i] = 1;

}

void MultiGrainBloomFilter::unset(const Address& addr)
{
  // not used
}

bool MultiGrainBloomFilter::isSet(const Address& addr)
{
  int i = get_block_index(addr);
  int j = get_page_index(addr);
  assert(i < m_filter_size);
  assert(j < m_page_filter_size);
  // we have to have both indices set
  return (m_filter[i] && m_page_filter[i]);
}

int MultiGrainBloomFilter::getCount(const Address& addr)
{
  // not used
  return 0;
}

int MultiGrainBloomFilter::getTotalCount()
{
  int count = 0;

  for (int i = 0; i < m_filter_size; i++) {
    count += m_filter[i];
  }

  for(int i=0; i < m_page_filter_size; ++i){
    count += m_page_filter[i] = 0;
  }

  return count;
}

int MultiGrainBloomFilter::getIndex(const Address& addr)
{
  return 0;
  // TODO
}

int MultiGrainBloomFilter::readBit(const int index) {
  return 0;
  // TODO
}

void MultiGrainBloomFilter::writeBit(const int index, const int value) {
  // TODO
}

void MultiGrainBloomFilter::print(ostream& out) const
{
}

int MultiGrainBloomFilter::get_block_index(const Address& addr)
{
  // grap a chunk of bits after byte offset
  return addr.bitSelect( RubyConfig::dataBlockBits(), RubyConfig::dataBlockBits() + m_filter_size_bits - 1);
}

int MultiGrainBloomFilter::get_page_index(const Address & addr)
{
  // grap a chunk of bits after first chunk
  return addr.bitSelect( RubyConfig::dataBlockBits() + m_filter_size_bits - 1,
                         RubyConfig::dataBlockBits() + m_filter_size_bits - 1 + m_page_filter_size_bits  - 1);
}




