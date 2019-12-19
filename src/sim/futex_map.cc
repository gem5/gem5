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

#include <sim/futex_map.hh>

FutexKey::FutexKey(uint64_t addr_in, uint64_t tgid_in)
        : addr(addr_in), tgid(tgid_in) {}

bool
FutexKey::operator==(const FutexKey &in) const
{
    return addr == in.addr && tgid == in.tgid;
}

namespace std {
    size_t hash<FutexKey>::operator()(const FutexKey& in) const
    {
        size_t hash = 65521;
        for (int i = 0; i < sizeof(uint64_t) / sizeof(size_t); i++) {
            hash ^= (size_t)(in.addr >> sizeof(size_t) * i) ^
                    (size_t)(in.tgid >> sizeof(size_t) * i);
        }
        return hash;
    }
}

WaiterState::WaiterState(ThreadContext* _tc, int _bitmask)
      : tc(_tc), bitmask(_bitmask) { }

bool
WaiterState::checkMask(int wakeup_bitmask) const
{
    return bitmask & wakeup_bitmask;
}

void
FutexMap::suspend(Addr addr, uint64_t tgid, ThreadContext *tc)
{
    suspend_bitset(addr, tgid, tc, 0xffffffff);
}

int
FutexMap::wakeup(Addr addr, uint64_t tgid, int count)
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
        tc->activate();
        woken_up++;
        waiterList.pop_front();
        waitingTcs.erase(tc);
    }

    if (waiterList.empty())
        erase(it);

    return woken_up;
}

void
FutexMap::suspend_bitset(Addr addr, uint64_t tgid, ThreadContext *tc,
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
    waitingTcs.emplace(tc);

    /** Suspend the thread context */
    tc->suspend();
}

int
FutexMap::wakeup_bitset(Addr addr, uint64_t tgid, int bitmask)
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
            waitingTcs.erase(waiter.tc);
            woken_up++;
        } else {
            ++iter;
        }
    }

    if (waiterList.empty())
        erase(it);

    return woken_up;
}

int
FutexMap::requeue(Addr addr1, uint64_t tgid, int count, int count2, Addr addr2)
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

bool
FutexMap::is_waiting(ThreadContext *tc)
{
    return waitingTcs.find(tc) != waitingTcs.end();
}
