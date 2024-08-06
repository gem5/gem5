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

 /**
  * Implementation of the Signature Path Prefetcher (v2)
  *
  * References:
  *     Path confidence based lookahead prefetching
  *     Jinchun Kim, Seth H. Pugsley, Paul V. Gratz, A. L. Narasimha Reddy,
  *     Chris Wilkerson, and Zeshan Chishti. 2016.
  *     In The 49th Annual IEEE/ACM International Symposium on
  *     Microarchitecture (MICRO-49). IEEE Press, Piscataway, NJ, USA,
  *     Article 60, 12 pages.
  */

#ifndef __MEM_CACHE_PREFETCH_SIGNATURE_PATH_V2_HH__
#define __MEM_CACHE_PREFETCH_SIGNATURE_PATH_V2_HH__

#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/signature_path.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SignaturePathPrefetcherV2Params;

namespace prefetch
{

class SignaturePathV2 : public SignaturePath
{
    /** Global History Register entry datatype */
    struct GlobalHistoryEntry : public TaggedEntry
    {
        signature_t signature;
        double confidence;
        stride_t lastBlock;
        stride_t delta;
        GlobalHistoryEntry() : signature(0), confidence(0.0), lastBlock(0),
                               delta(0) {}
    };
    /** Global History Register */
    AssociativeSet<GlobalHistoryEntry> globalHistoryRegister;

    double calculateLookaheadConfidence(PatternEntry const &sig,
            PatternStrideEntry const &lookahead) const override;

    double calculatePrefetchConfidence(PatternEntry const &sig,
            PatternStrideEntry const &lookahead) const override;

    void increasePatternEntryCounter(PatternEntry &pattern_entry,
            PatternStrideEntry &pstride_entry) override;

    void handleSignatureTableMiss(stride_t current_block,
            signature_t &new_signature, double &new_conf,
            stride_t &new_stride) override;

    /**
     * In this version of the Signature Path Prefetcher, there is no auxiliary
     * prefetcher, so this function does not perform any actions.
     */
    void auxiliaryPrefetcher(Addr ppn, stride_t current_block, bool is_secure,
            std::vector<AddrPriority> &addresses) override
    {}

    virtual void handlePageCrossingLookahead(signature_t signature,
            stride_t last_offset, stride_t delta, double path_confidence)
            override;

  public:
    SignaturePathV2(const SignaturePathPrefetcherV2Params &p);
    ~SignaturePathV2() = default;
};

} // namespace prefetch
} // namespace gem5

#endif//__MEM_CACHE_PREFETCH_SIGNATURE_PATH_V2_HH__
