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
#include <unordered_set>

#include <cpu/thread_context.hh>

/**
 * FutexKey class defines an unique identifier for a particular futex in the
 * system. The tgid and an address are the unique values needed as the key.
 */
class FutexKey {
  public:
    uint64_t addr;
    uint64_t tgid;

    FutexKey(uint64_t addr_in, uint64_t tgid_in);

    bool operator==(const FutexKey &in) const;
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
        size_t operator()(const FutexKey& in) const;
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
    WaiterState(ThreadContext* _tc, int _bitmask);

    /**
     * return true if the bit-wise AND of the wakeup_bitmask given by
     * a waking thread and this thread's internal bitmask is non-zero
     */
    bool checkMask(int wakeup_bitmask) const;
};

typedef std::list<WaiterState> WaiterList;

/**
 * FutexMap class holds a map of all futexes used in the system
 */
class FutexMap : public std::unordered_map<FutexKey, WaiterList>
{
  public:
    /** Inserts a futex into the map with one waiting TC */
    void suspend(Addr addr, uint64_t tgid, ThreadContext *tc);

    /** Wakes up at most count waiting threads on a futex */
    int wakeup(Addr addr, uint64_t tgid, int count);

    void suspend_bitset(Addr addr, uint64_t tgid, ThreadContext *tc,
                   int bitmask);

    int wakeup_bitset(Addr addr, uint64_t tgid, int bitmask);

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
    int requeue(Addr addr1, uint64_t tgid, int count, int count2, Addr addr2);

    /**
     * Determine if the given thread context is currently waiting on a
     * futex wait operation on any of the futexes tracked by this FutexMap.
     */
    bool is_waiting(ThreadContext *tc);

  private:

    std::unordered_set<ThreadContext *> waitingTcs;
};

#endif // __FUTEX_MAP_HH__
