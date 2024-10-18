/*
 * Copyright (c) 2024 Samsung Electronics
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

/**
 * @file
 * Describes a SMS prefetcher.
 */

#ifndef __MEM_CACHE_PREFETCH_SMS_HH__
#define __MEM_CACHE_PREFETCH_SMS_HH__

#include <set>

#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SmsPrefetcherParams;

namespace prefetch
{


class Sms : public Queued
{

  private:
    const int Max_Contexts; //= 64;
    const uint64_t MAX_PHTSize; //= 512;
    const Addr Region_Size; //= 4096;

    std::map< Addr, std::set<Addr> > AGT;
    std::map< Addr, std::pair<Addr,Addr> > AGTPC;
    std::map< Addr, std::pair<Addr,Addr> > FT;
    std::map< std::pair <Addr,Addr> , std::set<Addr> > PHT;
    std::deque<Addr> fifoFT;
    std::deque<Addr> lruAGT;
    std::deque< std::pair <Addr,Addr> > lruPHT;

    using EvictionInfo = CacheDataUpdateProbeArg;
    void notifyEvict(const EvictionInfo &info) override;

  public:
    Sms(const SmsPrefetcherParams &p);
    ~Sms() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_SMS_HH__
