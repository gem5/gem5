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

/**
 * WaiterState defines internal state of a waiter thread. The state
 * includes a pointer to the thread's context and its associated bitmask.
 */
class WaiterState {
  public:
    ThreadContext* tc;
    int bitmask;

    /**
     * this constructor is used if futex ops with bitset are used
     */
    WaiterState(ThreadContext* _tc, int _bitmask)
      : tc(_tc), bitmask(_bitmask)
    { }

    /**
     * if bitset is not defined, just set bitmask to 0xffffffff
     */
    WaiterState(ThreadContext* _tc)
      : tc(_tc), bitmask(0xffffffff)
    { }

    /**
     * return true if the bit-wise AND of the wakeup_bitmask given by
     * a waking thread and this thread's internal bitmask is non-zero
     */
    bool
    checkMask(int wakeup_bitmask) const
    {
        return bitmask & wakeup_bitmask;
    }
};

typedef std::list<WaiterState> WaiterList;

/**
 * FutexMap class holds a map of all futexes used in the system
 */
class FutexMap : public std::unordered_map<FutexKey, WaiterList>
{
  public:
    /** Inserts a futex into the map with one waiting TC */
    void
    suspend(Addr addr, uint64_t tgid, ThreadContext *tc)
    {
        FutexKey key(addr, tgid);
        auto it = find(key);

        if (it == end()) {
            WaiterList waiterList {WaiterState(tc)};
            insert({key, waiterList});
        } else {
            it->second.push_back(WaiterState(tc));
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
        auto &waiterList = it->second;

        while (!waiterList.empty() && woken_up < count) {
            // Threads may be woken up by access to locked
            // memory addresses outside of syscalls, so we
            // must only count threads that were actually
            // woken up by this syscall.
            auto& tc = waiterList.front().tc;
            if (tc->status() == ThreadContext::Suspended) {
                tc->activate();
                woken_up++;
            }
            waiterList.pop_front();
        }

        if (waiterList.empty())
            erase(it);

        return woken_up;
    }

    /**
     * inserts a futex into the map with one waiting TC
     * associates the waiter with a given bitmask
     */
    void
    suspend_bitset(Addr addr, uint64_t tgid, ThreadContext *tc,
                   int bitmask)
    {
        FutexKey key(addr, tgid);
        auto it = find(key);

        if (it == end()) {
            WaiterList waiterList {WaiterState(tc, bitmask)};
            insert({key, waiterList});
        } else {
            it->second.push_back(WaiterState(tc, bitmask));
        }

        /** Suspend the thread context */
        tc->suspend();
    }

    /**
     * Wakes up all waiters waiting on the addr and associated with the
     * given bitset
     */
    int
    wakeup_bitset(Addr addr, uint64_t tgid, int bitmask)
    {
        FutexKey key(addr, tgid);
        auto it = find(key);

        if (it == end())
            return 0;

        int woken_up = 0;

        auto &waiterList = it->second;
        auto iter = waiterList.begin();

        while (iter != waiterList.end()) {
            WaiterState& waiter = *iter;

            if (waiter.checkMask(bitmask)) {
                waiter.tc->activate();
                iter = waiterList.erase(iter);
                woken_up++;
            } else {
                ++iter;
            }
        }

        if (waiterList.empty())
            erase(it);

        return woken_up;
    }

    /**
     * This operation wakes a given number (val) of waiters. If there are
     * more threads waiting than woken, they are removed from the wait
     * queue of the futex pointed to by addr1 and added to the wait queue
     * of the futex pointed to by addr2. The number of waiter moved is
     * capped by count2 (misused timeout parameter).
     *
     * The return value is the number of waiters that are woken or
     * requeued.
     */
    int
    requeue(Addr addr1, uint64_t tgid, int count, int count2, Addr addr2)
    {
        FutexKey key1(addr1, tgid);
        auto it1 = find(key1);

        if (it1 == end())
            return 0;

        int woken_up = 0;
        auto &waiterList1 = it1->second;

        while (!waiterList1.empty() && woken_up < count) {
            waiterList1.front().tc->activate();
            waiterList1.pop_front();
            woken_up++;
        }

        WaiterList tmpList;
        int requeued = 0;

        while (!waiterList1.empty() && requeued < count2) {
          auto w = waiterList1.front();
          waiterList1.pop_front();
          tmpList.push_back(w);
          requeued++;
        }

        FutexKey key2(addr2, tgid);
        auto it2 = find(key2);

        if (it2 == end() && requeued > 0) {
            insert({key2, tmpList});
        } else {
            it2->second.insert(it2->second.end(),
                               tmpList.begin(), tmpList.end());
        }

        if (waiterList1.empty())
            erase(it1);

        return woken_up + requeued;
    }
};

#endif // __FUTEX_MAP_HH__
