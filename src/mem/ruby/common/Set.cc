
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
 * Set.C
 *
 * Description: See Set.h
 *
 * $Id$
 *
 */

#include "Set.hh"
#include "RubyConfig.hh"

#ifdef OPTBIGSET
#include "OptBigSet.cc"
#else

#ifdef BIGSET
#include "BigSet.cc" // code to supports sets larger than 32
#else

Set::Set()
{
  setSize(RubyConfig::numberOfChips());
}

Set::Set(int size)
{
  setSize(size);
}

bool Set::isEqual(const Set& set)
{
  return (m_bits == set.m_bits);
}

void Set::add(NodeID index)
{
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
  assert(index < m_size);
  m_bits |= (1 << index);
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::addSet(const Set& set)
{
  assert(m_size == set.m_size);
  m_bits |= set.m_bits;
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::addRandom()
{
  m_bits |= random();
  m_bits &= m_mask;
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::remove(NodeID index)
{
  assert(index < m_size);
  m_bits &= ~(1 << index);
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::removeSet(const Set& set)
{
  assert(m_size == set.m_size);
  m_bits &= ~(set.m_bits);
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::clear()
{
  m_bits = 0;
}

void Set::broadcast()
{
  m_bits = m_mask;
}

int Set::count() const
{
  int counter = 0;
  for (int i=0; i<m_size; i++) {
    if ((m_bits & (1 << i)) != 0) {
      counter++;
    }
  }
  return counter;
}

NodeID Set::elementAt(int index) {
  // count from right to left, index starts from 0
  for (int i=0; i<m_size; i++) {
    if ((m_bits & (1 << i)) != 0) {
      if (index == 0) return i;
      index --;
    }
  }
  assert(0); // index out of range
  return 0;
}

NodeID Set::smallestElement() const
{
  assert(count() > 0);
  int counter = 0;
  for (int i=0; i<m_size; i++) {
    if (isElement(i)) {
      return i;
    }
  }
  ERROR_MSG("No smallest element of an empty set.");
}

// Returns true iff all bits are set
bool Set::isBroadcast() const
{
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
  return (m_mask == m_bits);
}

// Returns true iff no bits are set
bool Set::isEmpty() const
{
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
  return (m_bits == 0);
}

// returns the logical OR of "this" set and orSet
Set Set::OR(const Set& orSet) const
{
  assert(m_size == orSet.m_size);
  Set result(m_size);
  result.m_bits = (m_bits | orSet.m_bits);
  assert((result.m_bits & result.m_mask) == result.m_bits);  // check for any bits outside the range
  return result;
}

// returns the logical AND of "this" set and andSet
Set Set::AND(const Set& andSet) const
{
  assert(m_size == andSet.m_size);
  Set result(m_size);
  result.m_bits = (m_bits & andSet.m_bits);
  assert((result.m_bits & result.m_mask) == result.m_bits);  // check for any bits outside the range
  return result;
}

// Returns true if the intersection of the two sets is non-empty
bool Set::intersectionIsNotEmpty(const Set& other_set) const
{
  assert(m_size == other_set.m_size);
  return ((m_bits & other_set.m_bits) != 0);
}

// Returns true if the intersection of the two sets is empty
bool Set::intersectionIsEmpty(const Set& other_set) const
{
  assert(m_size == other_set.m_size);
  return ((m_bits & other_set.m_bits) == 0);
}

bool Set::isSuperset(const Set& test) const
{
  assert(m_size == test.m_size);
  uint32 temp = (test.m_bits & (~m_bits));
  return (temp == 0);
}

bool Set::isElement(NodeID element) const
{
  return ((m_bits & (1 << element)) != 0);
}

void Set::setSize(int size)
{
  // We're using 32 bit ints, and the 32nd bit acts strangely due to
  // signed/unsigned, so restrict the set size to 31 bits.
  assert(size < 32);
  m_size = size;
  m_bits = 0;
  m_mask = ~((~0) << m_size);
  assert(m_mask != 0);
  assert((m_bits & m_mask) == m_bits);  // check for any bits outside the range
}

void Set::print(ostream& out) const
{
  out << "[Set (" << m_size << ") ";

  for (int i=0; i<m_size; i++) {
    out << (bool) isElement(i) << " ";
  }
  out << "]";
}

#endif // BIGSET

#endif // OPTBIGSET

