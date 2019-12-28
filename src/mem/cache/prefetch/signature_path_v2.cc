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

#include "mem/cache/prefetch/signature_path_v2.hh"

#include <cassert>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SignaturePathPrefetcherV2.hh"

namespace Prefetcher {

SignaturePathV2::SignaturePathV2(const SignaturePathPrefetcherV2Params *p)
    : SignaturePath(p),
      globalHistoryRegister(p->global_history_register_entries,
                            p->global_history_register_entries,
                            p->global_history_register_indexing_policy,
                            p->global_history_register_replacement_policy,
                            GlobalHistoryEntry())
{
}

void
SignaturePathV2::handleSignatureTableMiss(stride_t current_block,
    signature_t &new_signature, double &new_conf, stride_t &new_stride)
{
    bool found = false;

    // This should return all entries of the GHR, since it is a fully
    // associative table
    std::vector<GlobalHistoryEntry *> all_ghr_entries =
             globalHistoryRegister.getPossibleEntries(0 /* any value works */);

    for (auto gh_entry : all_ghr_entries) {
        if (gh_entry->lastBlock + gh_entry->delta == current_block) {
            new_signature = gh_entry->signature;
            new_conf = gh_entry->confidence;
            new_stride = gh_entry->delta;
            found = true;
            globalHistoryRegister.accessEntry(gh_entry);
            break;
        }
    }
    if (!found) {
        new_signature = current_block;
        new_conf = 1.0;
        new_stride = current_block;
    }
}

double
SignaturePathV2::calculateLookaheadConfidence(
        PatternEntry const &sig, PatternStrideEntry const &lookahead) const
{
    if (sig.counter == 0) return 0.0;
    return (((double) usefulPrefetches) / issuedPrefetches) *
            (((double) lookahead.counter) / sig.counter);
}

double
SignaturePathV2::calculatePrefetchConfidence(PatternEntry const &sig,
        PatternStrideEntry const &entry) const
{
    if (sig.counter == 0) return 0.0;
    return ((double) entry.counter) / sig.counter;
}

void
SignaturePathV2::increasePatternEntryCounter(
        PatternEntry &pattern_entry, PatternStrideEntry &pstride_entry)
{
    if (pattern_entry.counter.isSaturated()) {
        pattern_entry.counter >>= 1;
        for (auto &entry : pattern_entry.strideEntries) {
            entry.counter >>= 1;
        }
    }
    if (pstride_entry.counter.isSaturated()) {
        pattern_entry.counter >>= 1;
        for (auto &entry : pattern_entry.strideEntries) {
            entry.counter >>= 1;
        }
    }
    pattern_entry.counter++;
    pstride_entry.counter++;
}

void
SignaturePathV2::handlePageCrossingLookahead(signature_t signature,
            stride_t last_offset, stride_t delta, double path_confidence)
{
    // Always use the replacement policy to assign new entries, as all
    // of them are unique, there are never "hits" in the GHR
    GlobalHistoryEntry *gh_entry = globalHistoryRegister.findVictim(0);
    assert(gh_entry != nullptr);
    // Any address value works, as it is never used
    globalHistoryRegister.insertEntry(0, false, gh_entry);

    gh_entry->signature = signature;
    gh_entry->lastBlock = last_offset;
    gh_entry->delta = delta;
    gh_entry->confidence = path_confidence;
}

} // namespace Prefetcher

Prefetcher::SignaturePathV2*
SignaturePathPrefetcherV2Params::create()
{
    return new Prefetcher::SignaturePathV2(this);
}
