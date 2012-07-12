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

#ifndef __MEM_RUBY_SYSTEM_PERFECTCACHEMEMORY_HH__
#define __MEM_RUBY_SYSTEM_PERFECTCACHEMEMORY_HH__

#include "base/hashmap.hh"
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/common/Address.hh"

template<class ENTRY>
struct PerfectCacheLineState
{
    PerfectCacheLineState() { m_permission = AccessPermission_NUM; }
    AccessPermission m_permission;
    ENTRY m_entry;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const PerfectCacheLineState<ENTRY>& obj)
{
    return out;
}

template<class ENTRY>
class PerfectCacheMemory
{
  public:
    PerfectCacheMemory();

    // tests to see if an address is present in the cache
    bool isTagPresent(const Address& address) const;

    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an Invalid line in the same cache "way"
    bool cacheAvail(const Address& address) const;

    // find an Invalid entry and sets the tag appropriate for the address
    void allocate(const Address& address);

    void deallocate(const Address& address);

    // Returns with the physical address of the conflicting cache line
    Address cacheProbe(const Address& newAddress) const;

    // looks an address up in the cache
    ENTRY& lookup(const Address& address);
    const ENTRY& lookup(const Address& address) const;

    // Get/Set permission of cache block
    AccessPermission getPermission(const Address& address) const;
    void changePermission(const Address& address, AccessPermission new_perm);

    // Print cache contents
    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    PerfectCacheMemory(const PerfectCacheMemory& obj);
    PerfectCacheMemory& operator=(const PerfectCacheMemory& obj);

    // Data Members (m_prefix)
    m5::hash_map<Address, PerfectCacheLineState<ENTRY> > m_map;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const PerfectCacheMemory<ENTRY>& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

template<class ENTRY>
inline
PerfectCacheMemory<ENTRY>::PerfectCacheMemory()
{
}

// tests to see if an address is present in the cache
template<class ENTRY>
inline bool
PerfectCacheMemory<ENTRY>::isTagPresent(const Address& address) const
{
    return m_map.count(line_address(address)) > 0;
}

template<class ENTRY>
inline bool
PerfectCacheMemory<ENTRY>::cacheAvail(const Address& address) const
{
    return true;
}

// find an Invalid or already allocated entry and sets the tag
// appropriate for the address
template<class ENTRY>
inline void
PerfectCacheMemory<ENTRY>::allocate(const Address& address)
{
    PerfectCacheLineState<ENTRY> line_state;
    line_state.m_permission = AccessPermission_Invalid;
    line_state.m_entry = ENTRY();
    m_map[line_address(address)] = line_state;
}

// deallocate entry
template<class ENTRY>
inline void
PerfectCacheMemory<ENTRY>::deallocate(const Address& address)
{
    m_map.erase(line_address(address));
}

// Returns with the physical address of the conflicting cache line
template<class ENTRY>
inline Address
PerfectCacheMemory<ENTRY>::cacheProbe(const Address& newAddress) const
{
    panic("cacheProbe called in perfect cache");
    return newAddress;
}

// looks an address up in the cache
template<class ENTRY>
inline ENTRY&
PerfectCacheMemory<ENTRY>::lookup(const Address& address)
{
    return m_map[line_address(address)].m_entry;
}

// looks an address up in the cache
template<class ENTRY>
inline const ENTRY&
PerfectCacheMemory<ENTRY>::lookup(const Address& address) const
{
    return m_map[line_address(address)].m_entry;
}

template<class ENTRY>
inline AccessPermission
PerfectCacheMemory<ENTRY>::getPermission(const Address& address) const
{
    return m_map[line_address(address)].m_permission;
}

template<class ENTRY>
inline void
PerfectCacheMemory<ENTRY>::changePermission(const Address& address,
                                            AccessPermission new_perm)
{
    Address line_address = address;
    line_address.makeLineAddress();
    PerfectCacheLineState<ENTRY>& line_state = m_map[line_address];
    line_state.m_permission = new_perm;
}

template<class ENTRY>
inline void
PerfectCacheMemory<ENTRY>::print(std::ostream& out) const
{
}

#endif // __MEM_RUBY_SYSTEM_PERFECTCACHEMEMORY_HH__
