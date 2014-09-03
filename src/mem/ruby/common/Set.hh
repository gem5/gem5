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

// modified by Dan Gibson on 05/20/05 to accomidate FASTER
// >32 set lengths, using an array of ints w/ 32 bits/int

#ifndef __MEM_RUBY_COMMON_SET_HH__
#define __MEM_RUBY_COMMON_SET_HH__

#include <iostream>
#include <limits>

#include "mem/ruby/common/TypeDefines.hh"

/*
 * This defines the number of longs (32-bits on 32 bit machines,
 * 64-bit on 64-bit AMD machines) to use to hold the set...
 * the default is 4, allowing 128 or 256 different members
 * of the set.
 *
 * This should never need to be changed for correctness reasons,
 * though increasing it will increase performance for larger
 * set sizes at the cost of a (much) larger memory footprint
 *
 */
const int NUMBER_WORDS_PER_SET = 1;

class Set
{
  private:
    int m_nSize;              // the number of bits in this set
    int m_nArrayLen;          // the number of 32-bit words that are
                              // held in the array

    // Changed 5/24/05 for static allocation of array
    // note that "long" corresponds to 32 bits on a 32-bit machine,
    // 64 bits if the -m64 parameter is passed to g++, which it is
    // for an AMD opteron under our configuration

    long *m_p_nArray;      // an word array to hold the bits in the set
    long m_p_nArray_Static[NUMBER_WORDS_PER_SET];

    static const int LONG_BITS = std::numeric_limits<long>::digits + 1;
    static const int INDEX_SHIFT = LONG_BITS == 64 ? 6 : 5;
    static const int INDEX_MASK = (1 << INDEX_SHIFT) - 1;

    void clearExcess();

  public:
    Set();
    Set(int size);
    Set(const Set& obj);
    ~Set();

    Set& operator=(const Set& obj);

    void
    add(NodeID index)
    {
        m_p_nArray[index >> INDEX_SHIFT] |=
            (((unsigned long) 1) << (index & INDEX_MASK));
    }

    void addSet(const Set& set);

    void
    remove(NodeID index)
    {
        m_p_nArray[index >> INDEX_SHIFT] &=
            ~(((unsigned long)1) << (index & INDEX_MASK));
    }

    void removeSet(const Set& set);

    void
    clear()
    {
        for (int i = 0; i < m_nArrayLen; i++)
            m_p_nArray[i] = 0;
    }

    void broadcast();
    int count() const;
    bool isEqual(const Set& set) const;

    // return the logical OR of this set and orSet
    Set OR(const Set& orSet) const;

    // return the logical AND of this set and andSet
    Set AND(const Set& andSet) const;

    // Returns true if the intersection of the two sets is empty
    bool
    intersectionIsEmpty(const Set& other_set) const
    {
        for (int i = 0; i < m_nArrayLen; i++)
            if (m_p_nArray[i] & other_set.m_p_nArray[i])
                return false;
        return true;
    }

    bool isSuperset(const Set& test) const;
    bool isSubset(const Set& test) const { return test.isSuperset(*this); }

    bool
    isElement(NodeID element) const
    {
        return (m_p_nArray[element>>INDEX_SHIFT] &
            (((unsigned long)1) << (element & INDEX_MASK))) != 0;
    }

    bool isBroadcast() const;
    bool isEmpty() const;

    NodeID smallestElement() const;

    void setSize(int size);

    NodeID
    elementAt(int index) const
    {
        if (isElement(index))
            return (NodeID)true;
        else
            return 0;
    }

    int getSize() const { return m_nSize; }

    void print(std::ostream& out) const;
};

inline std::ostream&
operator<<(std::ostream& out, const Set& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_COMMON_SET_HH__
