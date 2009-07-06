
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
 * Set.cc
 *
 * Description: See Set.hh
 *
 * $Id: BigSet.cc 1.9 05/01/19 13:12:25-06:00 mikem@maya.cs.wisc.edu $
 *
 */

// modified (rewritten) 05/20/05 by Dan Gibson to accomimdate FASTER >32 bit
// set sizes

#include "mem/ruby/common/Set.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/config/RubyConfig.hh"

#if __amd64__ || __LP64__
#define __64BITS__
#else
#define __32BITS__
#endif

Set::Set()
{
  m_p_nArray = NULL;
  setSize(RubySystem::getNumberOfSequencers());
}

// copy constructor
Set::Set(const Set& obj) {
  m_p_nArray = NULL;
  setSize(obj.m_nSize);

  // copy from the host to this array
  for(int i=0; i<m_nArrayLen; i++) {
    m_p_nArray[i] = obj.m_p_nArray[i];
  }

}

Set::Set(int size)
{
  m_p_nArray = NULL;
  assert(size>0);
  setSize(size);
}

Set::~Set() {
  if( (m_p_nArray != (&m_p_nArray_Static[0])) && (m_p_nArray != NULL))
    delete [] m_p_nArray;
  m_p_nArray = NULL;
}


// /*
//  * This function should set the bit corresponding to index
//  * to 1.
//  */

// void Set::add(NodeID index)
// {
//   assert(index<m_nSize && index >= 0);

// #ifdef __32BITS__
//   m_p_nArray[index>>5] |= (1 << (index & 0x01F));
// #else
//   m_p_nArray[index>>6] |= (((unsigned long) 1) << (index & 0x03F));
// #endif // __32BITS__

// }

/*
 * This function should set all the bits in the current set
 * that are already set in the parameter set
 */
void Set::addSet(const Set& set)
{
  assert(getSize()==set.getSize());
  for(int i=0; i<m_nArrayLen; i++) {
    m_p_nArray[i] |= set.m_p_nArray[i];
  }

}

/*
 * This function should randomly assign 1 to the bits in the set--
 * it should not clear the bits bits first, though?
 */
void Set::addRandom()
{

  for(int i=0; i<m_nArrayLen; i++) {
    m_p_nArray[i] |= random() ^ (random() << 4); // this ensures that all 32 bits are subject to random effects,
                                                 // as RAND_MAX typically = 0x7FFFFFFF
  }

  // now just ensure that no bits over the maximum size were set
#ifdef __32BITS__
  long mask = 0x7FFFFFFF;

  // the number of populated spaces in the higest-order array slot is:
  // m_nSize % 32, so the uppermost 32 - m_nSize%32 bits should be
  // cleared

  if((m_nSize % 32) != 0) {
    for(int j=0; j<32-(m_nSize&0x01F); j++) {
      m_p_nArray[m_nArrayLen-1] &= mask;
      mask = mask >> 1;
    }
  }
#else
  long mask = 0x7FFFFFFFFFFFFFFF;

  // the number of populated spaces in the higest-order array slot is:
  // m_nSize % 64, so the uppermost 64 - m_nSize%64 bits should be
  // cleared

  if((m_nSize % 64) != 0) {
    for(int j=0; j<64-(m_nSize&0x03F); j++) {
      m_p_nArray[m_nArrayLen-1] &= mask;
      mask = mask >> 1;
    }
  }
#endif // __32BITS__

}

// /*
//  * This function unsets the bit associated with index
//  */
// void Set::remove(NodeID index)
// {
//   assert(index<m_nSize && index>=0);

// #ifdef __32BITS__
//   m_p_nArray[index>>5] &= ~(0x00000001         << (index & 0x01F));
// #else
//   m_p_nArray[index>>6] &= ~(((unsigned long) 0x0000000000000001) << (index & 0x03F));
// #endif // __32BITS__

// }


/*
 * This function clears bits that are =1 in the parameter set
 */
void Set::removeSet(const Set& set)
{

  assert(m_nSize==set.m_nSize);
  for(int i=0; i<m_nArrayLen; i++) {
    m_p_nArray[i] &= ~(set.m_p_nArray[i]);
  }

}

// /*
//  * This function clears all bits in the set
//  */
// void Set::clear()
// {
//   for(int i=0; i<m_nArrayLen; i++) {
//     m_p_nArray[i] = 0;
//   }
// }

/*
 * this function sets all bits in the set
 */
void Set::broadcast()
{

  for(int i=0; i<m_nArrayLen; i++) {
    m_p_nArray[i] = -1; // note that -1 corresponds to all 1's in 2's comp.
  }

  // now just ensure that no bits over the maximum size were set
#ifdef __32BITS__
  long mask = 0x7FFFFFFF;

  // the number of populated spaces in the higest-order array slot is:
  // m_nSize % 32, so the uppermost 32 - m_nSize%32 bits should be
  // cleared

  if((m_nSize % 32) != 0) {
    for(int j=0; j<32-(m_nSize&0x01F); j++) {
      m_p_nArray[m_nArrayLen-1] &= mask;
      mask = mask >> 1;
    }
  }
#else
  long mask = 0x7FFFFFFFFFFFFFFF;

  // the number of populated spaces in the higest-order array slot is:
  // m_nSize % 64, so the uppermost 64 - m_nSize%64 bits should be
  // cleared

  if((m_nSize % 64) != 0) {
    for(int j=0; j<64-(m_nSize&0x03F); j++) {
      m_p_nArray[m_nArrayLen-1] &= mask;
      mask = mask >> 1;
    }
  }
#endif // __32BITS__

}

/*
 * This function returns the population count of 1's in the set
 */
int Set::count() const
{
  int counter = 0;
  long mask;
  for( int i=0; i<m_nArrayLen; i++) {
    mask = (long) 0x01;

#ifdef __32BITS__
    for( int j=0; j<32; j++) {
      if(m_p_nArray[i] & mask) counter++;
      mask = mask << 1;
    }

#else

    for( int j=0; j<64; j++) {  // FIXME - significant performance loss when array population << 64
      if((m_p_nArray[i] & mask) != 0) {
              counter++;
      }
      mask = mask << 1;
    }

#endif // __32BITS__

  }

  return counter;
}

/*
 * This function checks for set equality
 */

bool Set::isEqual(const Set& set) const
{
  assert(m_nSize==set.m_nSize);

  for(int i=0;i<m_nArrayLen;i++) {
    if(m_p_nArray[i] != set.m_p_nArray[i]) {
      return false;
    }
  }

  return true;
}

/*
 * This function returns the NodeID (int) of the
 * least set bit
 */
NodeID Set::smallestElement() const
{
  assert(count() > 0);
  long x;
  for( int i=0; i<m_nArrayLen; i++) {
    if(m_p_nArray[i]!=0) {
      // the least-set bit must be in here
      x = m_p_nArray[i];

#ifdef __32BITS__
      for( int j=0; j<32; j++) {
        if(x & 0x00000001) {
          return 32*i+j;
        }

        x = x >> 1;
      }
#else
      for( int j=0; j<64; j++) {
        if(x & 0x0000000000000001) {
          return 64*i+j;
        }

        x = x >> 1;
      }
#endif // __32BITS__

      ERROR_MSG("No smallest element of an empty set.");
    }
  }

  ERROR_MSG("No smallest element of an empty set.");

  return 0;
}

/*
 * this function returns true iff all bits are set
 */
bool Set::isBroadcast() const
{
  // check the fully-loaded words by equal to 0xffffffff
  // only the last word may not be fully loaded, it is not
  // fully loaded iff m_nSize % 32 or 64 !=0 => fully loaded iff
  // m_nSize % 32 or 64 == 0

#ifdef __32BITS__
  for(int i=0; i< (((m_nSize % 32)==0) ? m_nArrayLen : m_nArrayLen-1); i++) {
    if(m_p_nArray[i]!=-1) {
      return false;
    }
  }

  // now check the last word, which may not be fully loaded
  long mask = 1;
  for(int j=0; j< (m_nSize % 32); j++) {
    if((mask & m_p_nArray[m_nArrayLen-1])==0) {
      return false;
    }
    mask = mask << 1;
  }
#else
  for(int i=0; i< (((m_nSize % 64)==0) ? m_nArrayLen : m_nArrayLen-1); i++) {
    if(m_p_nArray[i]!=-1) {
      return false;
    }
  }

  // now check the last word, which may not be fully loaded
  long mask = 1;
  for(int j=0; j< (m_nSize % 64); j++) {
    if((mask & m_p_nArray[m_nArrayLen-1])==0) {
      return false;
    }
    mask = mask << 1;
  }

#endif // __32BITS__

  return true;
}

/*
 * this function returns true iff no bits are set
 */
bool Set::isEmpty() const
{

  // here we can simply check if all = 0, since we ensure
  // that "extra slots" are all zero
  for(int i=0; i< m_nArrayLen ; i++) {
    if(m_p_nArray[i]!=0) {
            return false;
    }
  }

  return true;
}

// returns the logical OR of "this" set and orSet
Set Set::OR(const Set& orSet) const
{
  Set result(m_nSize);
  assert(m_nSize == orSet.m_nSize);
  for(int i=0; i< m_nArrayLen; i++) {
    result.m_p_nArray[i] = m_p_nArray[i] | orSet.m_p_nArray[i];
  }

  return result;

}

// returns the logical AND of "this" set and andSet
Set Set::AND(const Set& andSet) const
{
  Set result(m_nSize);
  assert(m_nSize == andSet.m_nSize);

  for(int i=0; i< m_nArrayLen; i++) {
    result.m_p_nArray[i] = m_p_nArray[i] & andSet.m_p_nArray[i];
  }

  return result;
}

// // Returns true if the intersection of the two sets is non-empty
// bool Set::intersectionIsNotEmpty(const Set& other_set) const
// {
//   assert(m_nSize == other_set.m_nSize);
//   for(int i=0; i< m_nArrayLen; i++) {
//     if(m_p_nArray[i] & other_set.m_p_nArray[i]) {
//       return true;
//     }
//   }

//   return false;

// }

// // Returns true if the intersection of the two sets is empty
// bool Set::intersectionIsEmpty(const Set& other_set) const
// {
//   assert(m_nSize == other_set.m_nSize);
//   for(int i=0; i< m_nArrayLen; i++) {
//     if(m_p_nArray[i] & other_set.m_p_nArray[i]) {
//       return false;
//     }
//   }

//   return true;

// }

/*
 * Returns false if a bit is set in the parameter set that is
 * NOT set in this set
 */
bool Set::isSuperset(const Set& test) const
{
  assert(m_nSize == test.m_nSize);

  for(int i=0;i<m_nArrayLen;i++) {
    if(((test.m_p_nArray[i] & m_p_nArray[i]) | ~test.m_p_nArray[i]) != -1) {
      return false;
    }
  }

  return true;
}

// /*
//  * Returns true iff this bit is set
//  */
// bool Set::isElement(NodeID element) const
// {
//   bool result;

// #ifdef __32BITS__
//   result = ((m_p_nArray[element>>5] & (0x00000001         << (element & 0x01F)))!=0);
// #else
//   result = ((m_p_nArray[element>>6] & (((unsigned long) 0x0000000000000001) << (element & 0x03F)))!=0);
// #endif // __32BITS__

//   return result;
// }

/*
 * "Supposed" to return the node id of the (n+1)th set
 * bit, IE n=0 => returns nodeid of first set bit, BUT
 * since BigSet.cc behaves strangely, this implementation
 * will behave strangely just for reverse compatability.
 *
 * Was originally implemented for the flight data recorder
 * FDR
 */

// NodeID Set::elementAt(int n) const
// {
//   if(isElement(n)) return (NodeID) true;
//   else return 0;

// /*
//   int match = -1;
//   for(int i=0;i<m_nSize;i++) {
//     if(isElement(i)) match++;
//     if(match==n) {
//       return i;
//     }
//   }

//   return -1;
//   */
// }

void Set::setSize(int size)
{
  m_nSize = size;

#ifdef __32BITS__
  m_nArrayLen = m_nSize/32 + ((m_nSize%32==0) ? 0 : 1 );
#else
  m_nArrayLen = m_nSize/64 + ((m_nSize%64==0) ? 0 : 1 );
#endif // __32BITS__

  // decide whether to use dynamic or static alloction
  if(m_nArrayLen<=NUMBER_WORDS_PER_SET) { // constant defined in RubyConfig.hh
    // its OK to use the static allocation, and it will
    // probably be faster (as m_nArrayLen is already in the
    // cache and they will probably share the same cache line)

    // if switching from dyanamic to static allocation (which
    // is probably rare, but why not be complete?), must delete
    // the dynamically allocated space
    if((m_p_nArray != NULL) && (m_p_nArray != &m_p_nArray_Static[0]))
      delete [] m_p_nArray;

    m_p_nArray = & m_p_nArray_Static[0];
  } else {

    // can't use static allocation...simply not enough room
    // so dynamically allocate some space
    if((m_p_nArray != NULL) && (m_p_nArray != &m_p_nArray_Static[0]))
      delete [] m_p_nArray;

    m_p_nArray = new long[m_nArrayLen];
  }

  clear();
}

Set& Set::operator=(const Set& obj) {
  if(this == &obj) {
    // do nothing
  } else {

    // resize this item
    setSize(obj.getSize());

    // copy the elements from obj to this
    for(int i=0; i<m_nArrayLen; i++) {
      m_p_nArray[i] = obj.m_p_nArray[i];
    }
  }

  return *this;
}

void Set::print(ostream& out) const
{
  if(m_p_nArray==NULL) {
    out << "[Set {Empty}]";
    return;
  }
  char buff[24];
  out << "[Set (" << m_nSize << ") 0x ";
  for (int i=m_nArrayLen-1; i>=0; i--) {
#ifdef __32BITS__
    sprintf(buff,"%08X ",m_p_nArray[i]);
#else
    sprintf(buff,"0x %016llX ", (long long)m_p_nArray[i]);
#endif // __32BITS__
    out << buff;
  }
  out << " ]";

}


