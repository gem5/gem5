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

#ifndef __MEM_RUBY_STRUCTURES_TIMERTABLE_HH__
#define __MEM_RUBY_STRUCTURES_TIMERTABLE_HH__

#include <cassert>
#include <iostream>
#include <map>
#include <string>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"

namespace gem5
{

namespace ruby
{

class TimerTable
{
  public:
    TimerTable();

    void
    setBlockSize(int block_size)
    {
        m_block_size_bits = floorLog2(block_size);
    }

    void
    setConsumer(Consumer* consumer_ptr)
    {
        assert(m_consumer_ptr == NULL);
        m_consumer_ptr = consumer_ptr;
    }

    void
    setDescription(const std::string& name)
    {
        m_name = name;
    }

    bool isReady(Tick curTime) const;
    Addr nextAddress() const;
    bool isSet(Addr address) const { return !!m_map.count(address); }
    void set(Addr address, Tick ready_time);
    void unset(Addr address);
    void print(std::ostream& out) const;

  private:
    void updateNext() const;

    // Private copy constructor and assignment operator
    TimerTable(const TimerTable& obj);
    TimerTable& operator=(const TimerTable& obj);

    // Data Members (m_prefix)

    // use a std::map for the address map as this container is sorted
    // and ensures a well-defined iteration order
    typedef std::map<Addr, Tick> AddressMap;
    AddressMap m_map;
    mutable bool m_next_valid;
    mutable Tick m_next_time; // Only valid if m_next_valid is true
    mutable Addr m_next_address;  // Only valid if m_next_valid is true

    //! Consumer to signal a wakeup()
    Consumer* m_consumer_ptr;

    int m_block_size_bits = 0;

    std::string m_name;
};

inline std::ostream&
operator<<(std::ostream& out, const TimerTable& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_TIMERTABLE_HH__
