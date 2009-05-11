
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
 * Set.h
 *
 * Description:
 *
 * $Id: BigSet.h 1.6 05/01/19 13:12:25-06:00 mikem@maya.cs.wisc.edu $
 *
 */

// modified by Dan Gibson on 05/20/05 to accomidate FASTER
// >32 set lengths, using an array of ints w/ 32 bits/int

// NOTE: Never include this file directly, this should only be
// included from Set.h

#ifndef SET_H
#define SET_H

#include "Global.hh"
#include "Vector.hh"
#include "NodeID.hh"
#include "RubyConfig.hh"

// gibson 05/20/05
// enum PresenceBit {NotPresent, Present};

class Set {
public:
  // Constructors
  // creates and empty set
  Set();
  Set (int size);

  // used during the replay mechanism
  //  Set(const char *str);

  Set(const Set& obj);
  Set& operator=(const Set& obj);

  // Destructor
  ~Set();

  // Public Methods

  inline void add(NodeID index)
    {
#ifdef __32BITS__
      m_p_nArray[index>>5] |= (1 << (index & 0x01F));
#else
      m_p_nArray[index>>6] |= (((unsigned long) 1) << (index & 0x03F));
#endif // __32BITS__
    }

  void addSet(const Set& set);
  void addRandom();

  inline void remove(NodeID index)
    {
#ifdef __32BITS__
      m_p_nArray[index>>5] &= ~(0x00000001         << (index & 0x01F));
#else
      m_p_nArray[index>>6] &= ~(((unsigned long) 0x0000000000000001) << (index & 0x03F));
#endif // __32BITS__
    }


  void removeSet(const Set& set);

  inline void clear() { for(int i=0; i<m_nArrayLen; i++) m_p_nArray[i] = 0; }

  void broadcast();
  int count() const;
  bool isEqual(const Set& set) const;

  Set OR(const Set& orSet) const;  // return the logical OR of this set and orSet
  Set AND(const Set& andSet) const;  // return the logical AND of this set and andSet

  // Returns true if the intersection of the two sets is non-empty
  inline bool intersectionIsNotEmpty(const Set& other_set) const
    {
      for(int i=0; i< m_nArrayLen; i++) {
        if(m_p_nArray[i] & other_set.m_p_nArray[i]) {
          return true;
        }
      }
      return false;
    }

  // Returns true if the intersection of the two sets is empty
  inline bool intersectionIsEmpty(const Set& other_set) const
    {
      for(int i=0; i< m_nArrayLen; i++) {
        if(m_p_nArray[i] & other_set.m_p_nArray[i]) {
          return false;
        }
      }
      return true;
    }

  bool isSuperset(const Set& test) const;
  bool isSubset(const Set& test) const { return test.isSuperset(*this); }

  inline bool isElement(NodeID element) const
    {
#ifdef __32BITS__
      return ((m_p_nArray[element>>5] & (0x00000001         << (element & 0x01F)))!=0);
#else
      return ((m_p_nArray[element>>6] & (((unsigned long) 0x0000000000000001) << (element & 0x03F)))!=0);
#endif // __32BITS__

    }

  bool isBroadcast() const;
  bool isEmpty() const;

  NodeID smallestElement() const;

  //  int size() const;
  void setSize (int size);

  // get element for a index
  inline NodeID elementAt(int index) const
    {
      if(isElement(index)) return (NodeID) true;
      else return 0;
    }

  // gibson 05/20/05
  int getSize() const { return m_nSize; }

  // DEPRECATED METHODS
  void addToSet(NodeID newElement) { add(newElement); }  // Deprecated
  void removeFromSet(NodeID newElement) { remove(newElement); }  // Deprecated
  void clearSet() { clear(); }   // Deprecated
  void setBroadcast() { broadcast(); }   // Deprecated
  bool presentInSet(NodeID element) const { return isElement(element); }  // Deprecated

  void print(ostream& out) const;
private:
  // Private Methods

  // Data Members (m_ prefix)
  // gibson 05/20/05
  // Vector<uint8> m_bits;  // This is an vector of uint8 to reduce the size of the set

  int m_nSize;              // the number of bits in this set
  int m_nArrayLen;          // the number of 32-bit words that are held in the array

  // Changed 5/24/05 for static allocation of array
  // note that "long" corresponds to 32 bits on a 32-bit machine,
  // 64 bits if the -m64 parameter is passed to g++, which it is
  // for an AMD opteron under our configuration

  long * m_p_nArray;      // an word array to hold the bits in the set
  long m_p_nArray_Static[NUMBER_WORDS_PER_SET];
};

// Output operator declaration
ostream& operator<<(ostream& out, const Set& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Set& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SET_H

