
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
 * NonCountingBloomFilter.cc
 *
 * Description:
 *
 *
 */

#include "mem/ruby/filters/H3BloomFilter.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/common/Address.hh"

H3BloomFilter::H3BloomFilter(string str)
{
  //TODO: change this ugly init code...
  primes_list[0] = 9323;
  primes_list[1] = 11279;
  primes_list[2] = 10247;
  primes_list[3] = 30637;
  primes_list[4] = 25717;
  primes_list[5] = 43711;

  mults_list[0] = 255;
  mults_list[1] = 29;
  mults_list[2] = 51;
  mults_list[3] = 3;
  mults_list[4] = 77;
  mults_list[5] = 43;

  adds_list[0] = 841;
  adds_list[1] = 627;
  adds_list[2] = 1555;
  adds_list[3] = 241;
  adds_list[4] = 7777;
  adds_list[5] = 65931;



  string tail(str);
  string head = string_split(tail, '_');

  // head contains filter size, tail contains bit offset from block number
  m_filter_size = atoi(head.c_str());

  head = string_split(tail, '_');
  m_num_hashes = atoi(head.c_str());

  if(tail == "Regular") {
    isParallel = false;
  } else if (tail == "Parallel") {
    isParallel = true;
  } else {
    cout << "ERROR: Incorrect config string for MultiHash Bloom! :" << str << endl;
    assert(0);
  }

  m_filter_size_bits = log_int(m_filter_size);

  m_par_filter_size = m_filter_size/m_num_hashes;
  m_par_filter_size_bits = log_int(m_par_filter_size);

  m_filter.setSize(m_filter_size);
  clear();
}

H3BloomFilter::~H3BloomFilter(){
}

void H3BloomFilter::clear()
{
  for (int i = 0; i < m_filter_size; i++) {
    m_filter[i] = 0;
  }
}

void H3BloomFilter::increment(const Address& addr)
{
  // Not used
}


void H3BloomFilter::decrement(const Address& addr)
{
  // Not used
}

void H3BloomFilter::merge(AbstractBloomFilter * other_filter){
  // assumes both filters are the same size!
  H3BloomFilter * temp = (H3BloomFilter*) other_filter;
  for(int i=0; i < m_filter_size; ++i){
    m_filter[i] |= (*temp)[i];
  }

}

void H3BloomFilter::set(const Address& addr)
{
  for (int i = 0; i < m_num_hashes; i++) {
    int idx = get_index(addr, i);
    m_filter[idx] = 1;

    //Profile hash value distribution
    //g_system_ptr->getProfiler()->getXactProfiler()->profileHashValue(i, idx); // gem5:Arka decomissiong of log_tm
  }
}

void H3BloomFilter::unset(const Address& addr)
{
  cout << "ERROR: Unset should never be called in a Bloom filter";
  assert(0);
}

bool H3BloomFilter::isSet(const Address& addr)
{
  bool res = true;

  for (int i=0; i < m_num_hashes; i++) {
    int idx = get_index(addr, i);
    res = res && m_filter[idx];
  }
  return res;
}


int H3BloomFilter::getCount(const Address& addr)
{
  return isSet(addr)? 1: 0;
}

int H3BloomFilter::getIndex(const Address& addr)
{
  return 0;
}

int H3BloomFilter::readBit(const int index) {
  return 0;
}

void H3BloomFilter::writeBit(const int index, const int value) {

}

int H3BloomFilter::getTotalCount()
{
  int count = 0;

  for (int i = 0; i < m_filter_size; i++) {
    count += m_filter[i];
  }
  return count;
}

void H3BloomFilter::print(ostream& out) const
{
}

int H3BloomFilter::get_index(const Address& addr, int i)
{
  uint64 x = addr.getLineAddress();
  //uint64 y = (x*mults_list[i] + adds_list[i]) % primes_list[i];
  int y = hash_H3(x,i);

  if(isParallel) {
    return (y % m_par_filter_size) + i*m_par_filter_size;
  } else {
    return y % m_filter_size;
  }
}

int H3BloomFilter::hash_H3(uint64 value, int index) {
  uint64 mask = 1;
  uint64 val = value;
  int result = 0;

  for(int i = 0; i < 64; i++) {
    if(val&mask) result ^= H3[i][index];
    val = val >> 1;
  }
  return result;
  }

