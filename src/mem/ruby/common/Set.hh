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

#include <bitset>
#include <cassert>
#include <iostream>

#include "base/misc.hh"
#include "mem/ruby/common/TypeDefines.hh"

// Change for systems with more than 64 controllers of a particular type.
const int NUMBER_BITS_PER_SET = 64;

class Set
{
  private:
    // Number of bits in use in this set.
    int m_nSize;
    std::bitset<NUMBER_BITS_PER_SET> bits;

  public:
    Set() : m_nSize(0) {}

    Set(int size) : m_nSize(size)
    {
        if (size > NUMBER_BITS_PER_SET)
            fatal("Number of bits(%d) < size specified(%d). "
                  "Increase the number of bits and recompile.\n",
                  NUMBER_BITS_PER_SET, size);
    }

    Set(const Set& obj) : m_nSize(obj.m_nSize), bits(obj.bits) {}
    ~Set() {}

    Set& operator=(const Set& obj)
    {
        m_nSize = obj.m_nSize;
        bits = obj.bits;
        return *this;
    }

    void
    add(NodeID index)
    {
        bits.set(index);
    }

    /*
     * This function should set all the bits in the current set that are
     * already set in the parameter set
     */
    void
    addSet(const Set& obj)
    {
        assert(m_nSize == obj.m_nSize);
        bits |= obj.bits;
    }

    /*
     * This function clears bits that are =1 in the parameter set
     */
    void
    remove(NodeID index)
    {
        bits.reset(index);
    }

    /*
     * This function clears bits that are =1 in the parameter set
     */
    void
    removeSet(const Set& obj)
    {
        assert(m_nSize == obj.m_nSize);
        bits &= (~obj.bits);
    }

    void clear() { bits.reset(); }

    /*
     * this function sets all bits in the set
     */
    void broadcast()
    {
        bits.set();
        for (int j = m_nSize; j < NUMBER_BITS_PER_SET; ++j) {
            bits.reset(j);
        }
    }

    /*
     * This function returns the population count of 1's in the set
     */
    int count() const { return bits.count(); }

    /*
     * This function checks for set equality
     */
    bool
    isEqual(const Set& obj) const
    {
        assert(m_nSize == obj.m_nSize);
        return bits == obj.bits;
    }

    // return the logical OR of this set and orSet
    Set
    OR(const Set& obj) const
    {
        assert(m_nSize == obj.m_nSize);
        Set r(m_nSize);
        r.bits = bits | obj.bits;
        return r;
    };

    // return the logical AND of this set and andSet
    Set
    AND(const Set& obj) const
    {
        assert(m_nSize == obj.m_nSize);
        Set r(m_nSize);
        r.bits = bits & obj.bits;
        return r;
    }

    // Returns true if the intersection of the two sets is empty
    bool
    intersectionIsEmpty(const Set& obj) const
    {
        std::bitset<NUMBER_BITS_PER_SET> r = bits & obj.bits;
        return r.none();
    }

    /*
     * Returns false if a bit is set in the parameter set that is NOT set
     * in this set
     */
    bool
    isSuperset(const Set& test) const
    {
        assert(m_nSize == test.m_nSize);
        std::bitset<NUMBER_BITS_PER_SET> r = bits | test.bits;
        return (r == bits);
    }

    bool isSubset(const Set& test) const { return test.isSuperset(*this); }

    bool isElement(NodeID element) const { return bits.test(element); }

    /*
     * this function returns true iff all bits in use are set
     */
    bool
    isBroadcast() const
    {
        return (bits.count() == m_nSize);
    }

    bool isEmpty() const { return bits.none(); }

    NodeID smallestElement() const
    {
        for (int i = 0; i < m_nSize; ++i) {
            if (bits.test(i)) {
                return i;
            }
        }
        panic("No smallest element of an empty set.");
    }

    bool elementAt(int index) const { return bits[index]; }

    int getSize() const { return m_nSize; }

    void
    setSize(int size)
    {
        if (size > NUMBER_BITS_PER_SET)
            fatal("Number of bits(%d) < size specified(%d). "
                  "Increase the number of bits and recompile.\n",
                  NUMBER_BITS_PER_SET, size);
        m_nSize = size;
        bits.reset();
    }

    void print(std::ostream& out) const
    {
        out << "[Set (" << m_nSize << "): " << bits << "]";
    }
};

inline std::ostream&
operator<<(std::ostream& out, const Set& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_COMMON_SET_HH__
