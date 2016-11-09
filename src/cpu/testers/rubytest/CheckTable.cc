/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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

#include "cpu/testers/rubytest/CheckTable.hh"

#include "base/intmath.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "cpu/testers/rubytest/Check.hh"
#include "debug/RubyTest.hh"

CheckTable::CheckTable(int _num_writers, int _num_readers, RubyTester* _tester)
    : m_num_writers(_num_writers), m_num_readers(_num_readers),
      m_tester_ptr(_tester)
{
    Addr physical = 0;

    const int size1 = 32;
    const int size2 = 100;

    DPRINTF(RubyTest, "Adding false sharing checks\n");
    // The first set is to get some false sharing
    physical = 1000;
    for (int i = 0; i < size1; i++) {
        // Setup linear addresses
        addCheck(physical);
        physical += CHECK_SIZE;
    }

    DPRINTF(RubyTest, "Adding cache conflict checks\n");
    // The next two sets are to get some limited false sharing and
    // cache conflicts
    physical = 1000;
    for (int i = 0; i < size2; i++) {
        // Setup linear addresses
        addCheck(physical);
        physical += 256;
    }

    DPRINTF(RubyTest, "Adding cache conflict checks2\n");
    physical = 1000 + CHECK_SIZE;
    for (int i = 0; i < size2; i++) {
        // Setup linear addresses
        addCheck(physical);
        physical += 256;
    }
}

CheckTable::~CheckTable()
{
    int size = m_check_vector.size();
    for (int i = 0; i < size; i++)
        delete m_check_vector[i];
}

void
CheckTable::addCheck(Addr address)
{
    if (floorLog2(CHECK_SIZE) != 0) {
        if (bitSelect(address, 0, CHECK_SIZE_BITS - 1) != 0) {
            panic("Check not aligned");
        }
    }

    for (int i = 0; i < CHECK_SIZE; i++) {
        if (m_lookup_map.count(address+i)) {
            // A mapping for this byte already existed, discard the
            // entire check
            return;
        }
    }

    DPRINTF(RubyTest, "Adding check for address: %s\n", address);

    Check* check_ptr = new Check(address, 100 + m_check_vector.size(),
                                 m_num_writers, m_num_readers, m_tester_ptr);
    for (int i = 0; i < CHECK_SIZE; i++) {
        // Insert it once per byte
        m_lookup_map[address + i] = check_ptr;
    }
    m_check_vector.push_back(check_ptr);
}

Check*
CheckTable::getRandomCheck()
{
    assert(m_check_vector.size() > 0);
    return m_check_vector[random_mt.random<unsigned>(0, m_check_vector.size() - 1)];
}

Check*
CheckTable::getCheck(const Addr address)
{
    DPRINTF(RubyTest, "Looking for check by address: %s\n", address);

    auto i = m_lookup_map.find(address);

    if (i == m_lookup_map.end())
        return NULL;

    Check* check = i->second;
    assert(check != NULL);
    return check;
}

void
CheckTable::print(std::ostream& out) const
{
}
