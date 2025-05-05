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

#include "mem/ruby/common/NetDest.hh"

#include <algorithm>

#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

NetDest::NetDest()
{
}

NetDest::NetDest(RubySystem *ruby_system)
    : m_ruby_system(ruby_system)
{
    resize();
}

void
NetDest::add(MachineID newElement)
{
    assert(m_bits.size() > 0);
    assert(bitIndex(newElement.num) < m_bits[vecIndex(newElement)].getSize());
    m_bits[vecIndex(newElement)].add(bitIndex(newElement.num));
}

void
NetDest::addNetDest(const NetDest& netDest)
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == netDest.getSize());
    for (int i = 0; i < m_bits.size(); i++) {
        m_bits[i].addSet(netDest.m_bits[i]);
    }
}

void
NetDest::setNetDest(MachineType machine, const Set& set)
{
    assert(m_ruby_system != nullptr);

    // assure that there is only one set of destinations for this machine
    assert(MachineType_base_level((MachineType)(machine + 1)) -
           MachineType_base_level(machine) == 1);
    m_bits[MachineType_base_level(machine)] = set;
}

void
NetDest::remove(MachineID oldElement)
{
    assert(m_bits.size() > 0);
    m_bits[vecIndex(oldElement)].remove(bitIndex(oldElement.num));
}

void
NetDest::removeNetDest(const NetDest& netDest)
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == netDest.getSize());
    for (int i = 0; i < m_bits.size(); i++) {
        m_bits[i].removeSet(netDest.m_bits[i]);
    }
}

void
NetDest::clear()
{
    assert(m_bits.size() > 0);
    for (int i = 0; i < m_bits.size(); i++) {
        m_bits[i].clear();
    }
}

void
NetDest::broadcast()
{
    for (MachineType machine = MachineType_FIRST;
         machine < MachineType_NUM; ++machine) {
        broadcast(machine);
    }
}

void
NetDest::broadcast(MachineType machineType)
{
    assert(m_ruby_system != nullptr);

    for (NodeID i = 0; i < MachineType_base_count(machineType); i++) {
        MachineID mach = {machineType, i};
        add(mach);
    }
}

//For Princeton Network
std::vector<NodeID>
NetDest::getAllDest()
{
    assert(m_ruby_system != nullptr);
    assert(m_bits.size() > 0);

    std::vector<NodeID> dest;
    dest.clear();
    for (int i = 0; i < m_bits.size(); i++) {
        for (int j = 0; j < m_bits[i].getSize(); j++) {
            if (m_bits[i].isElement(j)) {
                int id = MachineType_base_number((MachineType)i) + j;
                dest.push_back((NodeID)id);
            }
        }
    }
    return dest;
}

int
NetDest::count() const
{
    assert(m_bits.size() > 0);

    int counter = 0;
    for (int i = 0; i < m_bits.size(); i++) {
        counter += m_bits[i].count();
    }
    return counter;
}

NodeID
NetDest::elementAt(MachineID index)
{
    assert(m_bits.size() > 0);
    return m_bits[vecIndex(index)].elementAt(bitIndex(index.num));
}

MachineID
NetDest::smallestElement() const
{
    assert(m_bits.size() > 0);
    assert(count() > 0);
    for (int i = 0; i < m_bits.size(); i++) {
        for (NodeID j = 0; j < m_bits[i].getSize(); j++) {
            if (m_bits[i].isElement(j)) {
                MachineID mach = {MachineType_from_base_level(i), j};
                return mach;
            }
        }
    }
    panic("No smallest element of an empty set.");
}

MachineID
NetDest::smallestElement(MachineType machine) const
{
    assert(m_bits.size() > 0);
    assert(m_ruby_system != nullptr);

    int size = m_bits[MachineType_base_level(machine)].getSize();
    for (NodeID j = 0; j < size; j++) {
        if (m_bits[MachineType_base_level(machine)].isElement(j)) {
            MachineID mach = {machine, j};
            return mach;
        }
    }

    panic("No smallest element of given MachineType.");
}

// Returns true iff all bits are set
bool
NetDest::isBroadcast() const
{
    assert(m_bits.size() > 0);
    for (int i = 0; i < m_bits.size(); i++) {
        if (!m_bits[i].isBroadcast()) {
            return false;
        }
    }
    return true;
}

// Returns true iff no bits are set
bool
NetDest::isEmpty() const
{
    assert(m_bits.size() > 0);
    for (int i = 0; i < m_bits.size(); i++) {
        if (!m_bits[i].isEmpty()) {
            return false;
        }
    }
    return true;
}

// returns the logical OR of "this" set and orNetDest
NetDest
NetDest::OR(const NetDest& orNetDest) const
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == orNetDest.getSize());
    NetDest result(m_ruby_system);
    for (int i = 0; i < m_bits.size(); i++) {
        result.m_bits[i] = m_bits[i].OR(orNetDest.m_bits[i]);
    }
    return result;
}

// returns the logical AND of "this" set and andNetDest
NetDest
NetDest::AND(const NetDest& andNetDest) const
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == andNetDest.getSize());
    NetDest result(m_ruby_system);
    for (int i = 0; i < m_bits.size(); i++) {
        result.m_bits[i] = m_bits[i].AND(andNetDest.m_bits[i]);
    }
    return result;
}

// Returns true if the intersection of the two sets is non-empty
bool
NetDest::intersectionIsNotEmpty(const NetDest& other_netDest) const
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == other_netDest.getSize());
    for (int i = 0; i < m_bits.size(); i++) {
        if (!m_bits[i].intersectionIsEmpty(other_netDest.m_bits[i])) {
            return true;
        }
    }
    return false;
}

bool
NetDest::isSuperset(const NetDest& test) const
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == test.getSize());

    for (int i = 0; i < m_bits.size(); i++) {
        if (!m_bits[i].isSuperset(test.m_bits[i])) {
            return false;
        }
    }
    return true;
}

bool
NetDest::isElement(MachineID element) const
{
    assert(m_bits.size() > 0);
    return ((m_bits[vecIndex(element)])).isElement(bitIndex(element.num));
}

void
NetDest::resize()
{
    assert(m_ruby_system != nullptr);

    m_bits.resize(MachineType_base_level(MachineType_NUM));
    assert(m_bits.size() == MachineType_NUM);

    for (int i = 0; i < m_bits.size(); i++) {
        m_bits[i].setSize(MachineType_base_count((MachineType)i));
    }
}

void
NetDest::print(std::ostream& out) const
{
    assert(m_bits.size() > 0);
    out << "[NetDest (" << m_bits.size() << ") ";

    for (int i = 0; i < m_bits.size(); i++) {
        for (int j = 0; j < m_bits[i].getSize(); j++) {
            out << (bool) m_bits[i].isElement(j) << " ";
        }
        out << " - ";
    }
    out << "]";
}

bool
NetDest::isEqual(const NetDest& n) const
{
    assert(m_bits.size() > 0);
    assert(m_bits.size() == n.m_bits.size());
    for (unsigned int i = 0; i < m_bits.size(); ++i) {
        if (!m_bits[i].isEqual(n.m_bits[i]))
            return false;
    }
    return true;
}

int
NetDest::MachineType_base_count(const MachineType& obj)
{
    assert(m_ruby_system != nullptr);
    return m_ruby_system->MachineType_base_count(obj);
}

int
NetDest::MachineType_base_number(const MachineType& obj)
{
    assert(m_ruby_system != nullptr);
    return m_ruby_system->MachineType_base_number(obj);
}

} // namespace ruby
} // namespace gem5
