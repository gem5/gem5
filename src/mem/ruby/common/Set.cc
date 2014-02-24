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

// modified (rewritten) 05/20/05 by Dan Gibson to accomimdate FASTER
// >32 bit set sizes

#include <cassert>
#include <cstdio>

#include "base/misc.hh"
#include "mem/ruby/common/Set.hh"

Set::Set()
{
    m_p_nArray = NULL;
    m_nArrayLen = 0;
    m_nSize = 0;
}

Set::Set(const Set& obj)
{
    m_p_nArray = NULL;
    setSize(obj.m_nSize);

    // copy from the host to this array
    for (int i = 0; i < m_nArrayLen; i++)
        m_p_nArray[i] = obj.m_p_nArray[i];
}

Set::Set(int size)
{
    m_p_nArray = NULL;
    m_nArrayLen = 0;
    m_nSize = 0;
    if (size > 0)
        setSize(size);
}

Set::~Set()
{
    if (m_p_nArray && m_p_nArray != &m_p_nArray_Static[0])
        delete [] m_p_nArray;
    m_p_nArray = NULL;
}

void
Set::clearExcess()
{
    // now just ensure that no bits over the maximum size were set
#ifdef _LP64
    long mask = 0x7FFFFFFFFFFFFFFF;
#else
    long mask = 0x7FFFFFFF;
#endif

    // the number of populated spaces in the higest-order array slot
    // is: m_nSize % LONG_BITS, so the uppermost LONG_BITS -
    // m_nSize%64 bits should be cleared
    if ((m_nSize % LONG_BITS) != 0) {
        for (int j = 0; j < 64 - (m_nSize & INDEX_MASK); j++) {
            m_p_nArray[m_nArrayLen - 1] &= mask;
            mask = mask >> 1;
        }
    }
}


/*
 * This function should set all the bits in the current set that are
 * already set in the parameter set
 */
void
Set::addSet(const Set& set)
{
    assert(getSize()==set.getSize());
    for (int i = 0; i < m_nArrayLen; i++)
        m_p_nArray[i] |= set.m_p_nArray[i];
}

/*
 * This function should randomly assign 1 to the bits in the set--it
 * should not clear the bits bits first, though?
 */
void
Set::addRandom()
{

    for (int i = 0; i < m_nArrayLen; i++) {
        // this ensures that all 32 bits are subject to random effects,
        // as RAND_MAX typically = 0x7FFFFFFF
        m_p_nArray[i] |= random() ^ (random() << 4);
    }
    clearExcess();
}

/*
 * This function clears bits that are =1 in the parameter set
 */
void
Set::removeSet(const Set& set)
{
    assert(m_nSize == set.m_nSize);
    for (int i = 0; i < m_nArrayLen; i++)
        m_p_nArray[i] &= ~set.m_p_nArray[i];
}

/*
 * this function sets all bits in the set
 */
void
Set::broadcast()
{
    for (int i = 0; i < m_nArrayLen; i++)
        m_p_nArray[i] = -1; // note that -1 corresponds to all 1's in 2's comp.

    clearExcess();
}

/*
 * This function returns the population count of 1's in the set
 */
int
Set::count() const
{
    int counter = 0;
    long mask;

    for (int i = 0; i < m_nArrayLen; i++) {
        mask = (long)0x01;

        for (int j = 0; j < LONG_BITS; j++) {
            // FIXME - significant performance loss when array
            // population << LONG_BITS
            if ((m_p_nArray[i] & mask) != 0) {
                counter++;
            }
            mask = mask << 1;
        }
    }

    return counter;
}

/*
 * This function checks for set equality
 */
bool
Set::isEqual(const Set& set) const
{
    assert(m_nSize == set.m_nSize);

    for (int i = 0; i < m_nArrayLen; i++)
        if (m_p_nArray[i] != set.m_p_nArray[i])
            return false;

    return true;
}

/*
 * This function returns the NodeID (int) of the least set bit
 */
NodeID
Set::smallestElement() const
{
    assert(count() > 0);
    long x;
    for (int i = 0; i < m_nArrayLen; i++) {
        if (m_p_nArray[i] != 0) {
            // the least-set bit must be in here
            x = m_p_nArray[i];

            for (int j = 0; j < LONG_BITS; j++) {
                if (x & (unsigned long)1) {
                    return LONG_BITS * i + j;
                }

                x = x >> 1;
            }

            panic("No smallest element of an empty set.");
        }
    }

    panic("No smallest element of an empty set.");
}

/*
 * this function returns true iff all bits are set
 */
bool
Set::isBroadcast() const
{
    // check the fully-loaded words by equal to 0xffffffff
    // only the last word may not be fully loaded, it is not
    // fully loaded iff m_nSize % 32 or 64 !=0 => fully loaded iff
    // m_nSize % 32 or 64 == 0

    int max = (m_nSize % LONG_BITS) == 0 ? m_nArrayLen : m_nArrayLen - 1;
    for (int i = 0; i < max; i++) {
        if (m_p_nArray[i] != -1) {
            return false;
        }
    }

    // now check the last word, which may not be fully loaded
    long mask = 1;
    for (int j = 0; j < (m_nSize % LONG_BITS); j++) {
        if ((mask & m_p_nArray[m_nArrayLen-1]) == 0) {
            return false;
        }
        mask = mask << 1;
    }

    return true;
}

/*
 * this function returns true iff no bits are set
 */
bool
Set::isEmpty() const
{
    // here we can simply check if all = 0, since we ensure
    // that "extra slots" are all zero
    for (int i = 0; i < m_nArrayLen ; i++)
        if (m_p_nArray[i])
            return false;

    return true;
}

// returns the logical OR of "this" set and orSet
Set
Set::OR(const Set& orSet) const
{
    Set result(m_nSize);
    assert(m_nSize == orSet.m_nSize);
    for (int i = 0; i < m_nArrayLen; i++)
        result.m_p_nArray[i] = m_p_nArray[i] | orSet.m_p_nArray[i];

    return result;
}

// returns the logical AND of "this" set and andSet
Set
Set::AND(const Set& andSet) const
{
    Set result(m_nSize);
    assert(m_nSize == andSet.m_nSize);

    for (int i = 0; i < m_nArrayLen; i++) {
        result.m_p_nArray[i] = m_p_nArray[i] & andSet.m_p_nArray[i];
    }

    return result;
}

/*
 * Returns false if a bit is set in the parameter set that is NOT set
 * in this set
 */
bool
Set::isSuperset(const Set& test) const
{
    assert(m_nSize == test.m_nSize);

    for (int i = 0; i < m_nArrayLen; i++)
        if (((test.m_p_nArray[i] & m_p_nArray[i]) | ~test.m_p_nArray[i]) != -1)
            return false;

    return true;
}

void
Set::setSize(int size)
{
    m_nSize = size;
    m_nArrayLen = (m_nSize + LONG_BITS - 1) / LONG_BITS;

    // decide whether to use dynamic or static alloction
    if (m_nArrayLen <= NUMBER_WORDS_PER_SET) {
        // constant defined in RubySystem.hh
        // its OK to use the static allocation, and it will
        // probably be faster (as m_nArrayLen is already in the
        // cache and they will probably share the same cache line)

        // if switching from dyanamic to static allocation (which
        // is probably rare, but why not be complete?), must delete
        // the dynamically allocated space
        if (m_p_nArray && m_p_nArray != &m_p_nArray_Static[0])
            delete [] m_p_nArray;

        m_p_nArray = &m_p_nArray_Static[0];
    } else {
        // can't use static allocation...simply not enough room
        // so dynamically allocate some space
        if (m_p_nArray && m_p_nArray != &m_p_nArray_Static[0])
            delete [] m_p_nArray;

        m_p_nArray = new long[m_nArrayLen];
    }

    clear();
}

Set&
Set::operator=(const Set& obj)
{
    if (this != &obj) {
        // resize this item
        setSize(obj.getSize());

        // copy the elements from obj to this
        for (int i = 0; i < m_nArrayLen; i++)
            m_p_nArray[i] = obj.m_p_nArray[i];
    }

    return *this;
}

void
Set::print(std::ostream& out) const
{
    if (!m_p_nArray) {
        out << "[Set {Empty}]";
        return;
    }

    out << "[Set (" << m_nSize << ")";
    for (int i = m_nArrayLen - 1; i >= 0; i--) {
        out << csprintf(" 0x%08X", m_p_nArray[i]);
    }
    out << " ]";
}
