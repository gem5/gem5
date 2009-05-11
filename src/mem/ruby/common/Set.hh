
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
 * $Id$
 *
 */

// Define this to use the BigSet class which is slower, but supports
// sets of size larger than 32.

// #define BIGSET

#define OPTBIGSET

#ifdef OPTBIGSET
#include "mem/ruby/common/OptBigSet.hh"
#else

#ifdef BIGSET
#include "mem/ruby/common/BigSet.hh" // code to supports sets larger than 32
#else

#ifndef SET_H
#define SET_H

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/config/RubyConfig.hh"

class Set {
public:
  // Constructors
  // creates and empty set
  Set();
  Set(int size);

  // used during the replay mechanism
  //  Set(const char *str);

  //  Set(const Set& obj);
  //  Set& operator=(const Set& obj);

  // Destructor
  // ~Set();

  // Public Methods

  void add(NodeID newElement);
  void addSet(const Set& set);
  void addRandom();
  void remove(NodeID newElement);
  void removeSet(const Set& set);
  void clear();
  void broadcast();
  int count() const;
  bool isEqual(const Set& set);

  Set OR(const Set& orSet) const;  // return the logical OR of this set and orSet
  Set AND(const Set& andSet) const;  // return the logical AND of this set and andSet

  // Returns true if the intersection of the two sets is non-empty
  bool intersectionIsNotEmpty(const Set& other_set) const;

  // Returns true if the intersection of the two sets is empty
  bool intersectionIsEmpty(const Set& other_set) const;

  bool isSuperset(const Set& test) const;
  bool isSubset(const Set& test) const { return test.isSuperset(*this); }
  bool isElement(NodeID element) const;
  bool isBroadcast() const;
  bool isEmpty() const;

  NodeID smallestElement() const;

  //  int size() const;
  void setSize (int size);

  // get element for a index
  NodeID elementAt(int index);
  int getSize() const { return m_size; }

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
  int m_size;
  uint32 m_bits;  // Set as a bit vector
  uint32 m_mask;  // a 000001111 mask where the number of 1s is equal to m_size

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
#endif //BIGSET
#endif //OPTBIGSET

