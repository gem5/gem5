/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#ifndef __MEM_CACHE_PREFETCH_SLIM_AMPM_HH__
#define __MEM_CACHE_PREFETCH_SLIM_AMPM_HH__

#include "mem/cache/prefetch/access_map_pattern_matching.hh"
#include "mem/cache/prefetch/delta_correlating_prediction_tables.hh"
#include "mem/cache/prefetch/queued.hh"

/**
 * The SlimAMPM Prefetcher
 * Reference:
 *    Towards Bandwidth-Efficient Prefetching with Slim AMPM.
 *    Young, Vinson, and A. Krishna.
 *    The 2nd Data Prefetching Championship (2015).
 *
 * This prefetcher uses two other prefetchers, the AMPM and the DCPT.
 */

namespace gem5
{

struct SlimAMPMPrefetcherParams;

namespace prefetch
{

class SlimAMPM : public Queued
{
    /** AMPM prefetcher object */
    AccessMapPatternMatching &ampm;
    /** DCPT prefetcher object */
    DeltaCorrelatingPredictionTables &dcpt;

  public:
    SlimAMPM(const SlimAMPMPrefetcherParams &p);
    ~SlimAMPM() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif //__MEM_CACHE_PREFETCH_SLIM_AMPM_HH__
