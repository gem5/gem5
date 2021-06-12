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

#ifndef __MEM_RUBY_COMMON_NETDEST_HH__
#define __MEM_RUBY_COMMON_NETDEST_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Set.hh"
#include "mem/ruby/common/MachineID.hh"

namespace gem5
{

namespace ruby
{

// NetDest specifies the network destination of a Message
class NetDest
{
  public:
    // Constructors
    // creates and empty set
    NetDest();
    explicit NetDest(int bit_size);

    NetDest& operator=(const Set& obj);

    ~NetDest()
    { }

    void add(MachineID newElement);
    void addNetDest(const NetDest& netDest);
    void setNetDest(MachineType machine, const Set& set);
    void remove(MachineID oldElement);
    void removeNetDest(const NetDest& netDest);
    void clear();
    void broadcast();
    void broadcast(MachineType machine);
    int count() const;
    bool isEqual(const NetDest& netDest) const;

    // return the logical OR of this netDest and orNetDest
    NetDest OR(const NetDest& orNetDest) const;

    // return the logical AND of this netDest and andNetDest
    NetDest AND(const NetDest& andNetDest) const;

    // Returns true if the intersection of the two netDests is non-empty
    bool intersectionIsNotEmpty(const NetDest& other_netDest) const;

    // Returns true if the intersection of the two netDests is empty
    bool intersectionIsEmpty(const NetDest& other_netDest) const;

    bool isSuperset(const NetDest& test) const;
    bool isSubset(const NetDest& test) const { return test.isSuperset(*this); }
    bool isElement(MachineID element) const;
    bool isBroadcast() const;
    bool isEmpty() const;

    // For Princeton Network
    std::vector<NodeID> getAllDest();

    MachineID smallestElement() const;
    MachineID smallestElement(MachineType machine) const;

    void resize();
    int getSize() const { return m_bits.size(); }

    // get element for a index
    NodeID elementAt(MachineID index);

    void print(std::ostream& out) const;

  private:
    // returns a value >= MachineType_base_level("this machine")
    // and < MachineType_base_level("next highest machine")
    int
    vecIndex(MachineID m) const
    {
        int vec_index = MachineType_base_level(m.type);
        assert(vec_index < m_bits.size());
        return vec_index;
    }

    NodeID bitIndex(NodeID index) const { return index; }

    std::vector<Set> m_bits;  // a vector of bit vectors - i.e. Sets
};

inline std::ostream&
operator<<(std::ostream& out, const NetDest& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_COMMON_NETDEST_HH__
