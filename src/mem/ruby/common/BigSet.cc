
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

#include "Set.hh"
#include "RubyConfig.hh"

Set::Set()
{
  setSize(RubyConfig::numberOfProcessors());
}

Set::Set(int size)
{
  setSize(size);
}

void Set::add(NodeID index)
{
  m_bits[index] = Present;
}

void Set::addSet(const Set& set)
{
  assert(m_bits.size() == set.getSize());
  for (int i=0; i<m_bits.size(); i++) {
    if(set.isElement(i)){
      add(i);
    }
  }
}

void Set::addRandom()
{
  int rand = random();
  for (int i=0; i<m_bits.size(); i++) {
    if(rand & 0x1 == 0) {  // Look at the low order bit
      add(i);
    }
    rand = (rand >> 1);  // Shift the random number to look at the next bit
  }
}

void Set::remove(NodeID index)
{
  m_bits[index] = NotPresent;
}

void Set::removeSet(const Set& set)
{
  assert(m_bits.size() == set.getSize());
  for (int i=0; i<m_bits.size(); i++) {
    if(set.isElement(i)){
      remove(i);
    }
  }
}

void Set::clear()
{
  for (int i=0; i<m_bits.size(); i++) {
    m_bits[i] = NotPresent;
  }
}

void Set::broadcast()
{
  for (int i=0; i<m_bits.size(); i++) {
    m_bits[i] = Present;
  }
}

int Set::count() const
{
  int counter = 0;
  for (int i=0; i<m_bits.size(); i++) {
    if (m_bits[i] == Present) {
      counter++;
    }
  }
  return counter;
}

bool Set::isEqual(const Set& set) const
{
  assert(m_bits.size() == set.getSize());
  for (int i=0; i<m_bits.size(); i++) {
    if (m_bits[i] != set.isElement(i)) {
      return false;
    }
  }
  return true;
}

NodeID Set::smallestElement() const
{
  assert(count() > 0);
  for (int i=0; i<m_bits.size(); i++) {
    if (isElement(i)) {
      return i;
    }
  }
  ERROR_MSG("No smallest element of an empty set.");
}

// Returns true iff all bits are set
bool Set::isBroadcast() const
{
  for (int i=0; i<m_bits.size(); i++) {
    if (m_bits[i] == NotPresent) {
      return false;
    }
  }
  return true;
}

// Returns true iff no bits are set
bool Set::isEmpty() const
{
  for (int i=0; i<m_bits.size(); i++) {
    if (m_bits[i] == Present) {
      return false;
    }
  }
  return true;
}

// returns the logical OR of "this" set and orSet
Set Set::OR(const Set& orSet) const
{
  Set result;
  assert(m_bits.size() == orSet.getSize());
  result.setSize(m_bits.size());
  for (int i=0; i<m_bits.size(); i++) {
    if(m_bits[i] == Present || orSet.isElement(i)){
      result.add(i);
    }else{
      result.remove(i);
    }
  }

  return result;

}

// returns the logical AND of "this" set and andSet
Set Set::AND(const Set& andSet) const
{
  Set result;
  assert(m_bits.size() == andSet.getSize());
  result.setSize(m_bits.size());
  for (int i=0; i<m_bits.size(); i++) {
    if(m_bits[i] == Present && andSet.isElement(i)){
      result.add(i);
    }else{
      result.remove(i);
    }
  }
  return result;
}

// Returns true if the intersection of the two sets is non-empty
bool Set::intersectionIsNotEmpty(const Set& other_set) const
{
  assert(m_bits.size() == other_set.getSize());
  for(int index=0; index < m_bits.size(); index++){
    if(other_set.isElement(index) && isElement(index)) {
      return true;
    }
  }
  return false;
}

// Returns true if the intersection of the two sets is non-empty
bool Set::intersectionIsEmpty(const Set& other_set) const
{
  assert(m_bits.size() == other_set.getSize());
  for(int index=0; index < m_bits.size(); index++){
    if(other_set.isElement(index) && isElement(index)) {
      return false;
    }
  }
  return true;
}

bool Set::isSuperset(const Set& test) const
{
  assert(m_bits.size() == test.getSize());
  for(int index=0; index < m_bits.size(); index++){
    if(test.isElement(index) && !isElement(index)) {
      return false;
    }
  }
  return true;
}

bool Set::isElement(NodeID element) const
{
  return (m_bits[element] == Present);
}

NodeID Set::elementAt(int index) const
{
  if (m_bits[index] == Present) {
    return m_bits[index] == Present;
  } else {
    return 0;
  }
}

void Set::setSize(int size)
{
  m_bits.setSize(size);
  clear();
}

void Set::print(ostream& out) const
{
  out << "[Set ";
  for (int i=0; i<m_bits.size(); i++) {
    out << (bool)m_bits[i] << " ";
  }
  out << "]";
}
