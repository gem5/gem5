/*
 * Copyright (c) 2017 Advanced Micro Devices, Inc.
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
 *
 * Authors: Brandon Potter
 *          Steve Reinhardt
 *          Alexandru Dutu
 */

#ifndef __FUTEX_MAP_HH__
#define __FUTEX_MAP_HH__

#include <unordered_map>

#include <cpu/thread_context.hh>

/**
 * FutexKey class defines an unique identifier for a particular futex in the
 * system. The tgid and an address are the unique values needed as the key.
 */
class FutexKey {
  public:
    uint64_t addr;
    uint64_t tgid;

    FutexKey(uint64_t addr_in, uint64_t tgid_in)
        : addr(addr_in), tgid(tgid_in)
    {
    }

    bool
    operator==(const FutexKey &in) const
    {
        return addr == in.addr && tgid == in.tgid;
    }
};

namespace std {
    /**
     * The unordered_map structure needs the parenthesis operator defined for
     * std::hash if a user defined key is used. Our key is is user defined
     * so we need to provide the hash functor.
     */
    template <>
    struct hash<FutexKey>
    {
        size_t operator()(const FutexKey& in) const
        {
            size_t hash = 65521;
            for (int i = 0; i < sizeof(uint64_t) / sizeof(size_t); i++) {
                hash ^= (size_t)(in.addr >> sizeof(size_t) * i) ^
                        (size_t)(in.tgid >> sizeof(size_t) * i);
            }
            return hash;
        }
    };
}

typedef std::list<ThreadContext *> ThreadContextList;

/**
 * FutexMap class holds a map of all futexes used in the system
 */
class FutexMap : public std::unordered_map<FutexKey, ThreadContextList>
{
  public:
    /** Inserts a futex into the map with one waiting TC */
    void
    suspend(Addr addr, uint64_t tgid, ThreadContext *tc)
    {
        FutexKey key(addr, tgid);
        auto it = find(key);

        if (it == end()) {
            ThreadContextList tcList {tc};
            insert({key, tcList});
        } else {
            it->second.push_back(tc);
        }

        /** Suspend the thread context */
        tc->suspend();
    }

    /** Wakes up at most count waiting threads on a futex */
    int
    wakeup(Addr addr, uint64_t tgid, int count)
    {
        FutexKey key(addr, tgid);
        auto it = find(key);

        if (it == end())
            return 0;

        int woken_up = 0;
        auto &tcList = it->second;

        while (!tcList.empty() && woken_up < count) {
            tcList.front()->activate();
            tcList.pop_front();
            woken_up++;
        }

        if (tcList.empty())
            erase(it);

        return woken_up;
    }

};

#endif // __FUTEX_MAP_HH__
