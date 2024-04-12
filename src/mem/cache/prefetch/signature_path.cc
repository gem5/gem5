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

#include "mem/cache/prefetch/signature_path.hh"

#include <cassert>
#include <climits>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SignaturePathPrefetcher.hh"

namespace gem5
{

namespace prefetch
{

SignaturePath::SignaturePath(const SignaturePathPrefetcherParams &p)
    : Queued(p),
      stridesPerPatternEntry(p.strides_per_pattern_entry),
      signatureShift(p.signature_shift),
      signatureBits(p.signature_bits),
      prefetchConfidenceThreshold(p.prefetch_confidence_threshold),
      lookaheadConfidenceThreshold(p.lookahead_confidence_threshold),
      signatureTable((name() + ".SignatureTable").c_str(),
                     p.signature_table_entries,
                     p.signature_table_assoc,
                     p.signature_table_replacement_policy,
                     p.signature_table_indexing_policy,
                     SignatureEntry(p.signature_table_indexing_policy)),
      patternTable((name() + ".PatternTable").c_str(),
                   p.pattern_table_entries,
                   p.pattern_table_assoc,
                   p.pattern_table_replacement_policy,
                   p.pattern_table_indexing_policy,
                   PatternEntry(stridesPerPatternEntry, p.num_counter_bits,
                                p.pattern_table_indexing_policy))
{
    fatal_if(prefetchConfidenceThreshold < 0,
        "The prefetch confidence threshold must be greater than 0\n");
    fatal_if(prefetchConfidenceThreshold > 1,
        "The prefetch confidence threshold must be less than 1\n");
    fatal_if(lookaheadConfidenceThreshold < 0,
        "The lookahead confidence threshold must be greater than 0\n");
    fatal_if(lookaheadConfidenceThreshold > 1,
        "The lookahead confidence threshold must be less than 1\n");
}

SignaturePath::PatternStrideEntry &
SignaturePath::PatternEntry::getStrideEntry(stride_t stride)
{
    PatternStrideEntry *pstride_entry = findStride(stride);
    if (pstride_entry == nullptr) {
        // Specific replacement algorithm for this table,
        // pick the entry with the lowest counter value,
        // then decrease the counter of all entries

        // If all counters have the max value, this will be the pick
        PatternStrideEntry *victim_pstride_entry = &(strideEntries[0]);

        unsigned long current_counter = ULONG_MAX;
        for (auto &entry : strideEntries) {
            if (entry.counter < current_counter) {
                victim_pstride_entry = &entry;
                current_counter = entry.counter;
            }
            entry.counter--;
        }
        pstride_entry = victim_pstride_entry;
        pstride_entry->counter.reset();
        pstride_entry->stride = stride;
    }
    return *pstride_entry;
}

void
SignaturePath::addPrefetch(Addr ppn, stride_t last_block,
    stride_t delta, double path_confidence, signature_t signature,
    bool is_secure, std::vector<AddrPriority> &addresses)
{
    stride_t block = last_block + delta;

    Addr pf_ppn;
    stride_t pf_block;
    if (block < 0) {
        stride_t num_cross_pages = 1 + (-block) / (pageBytes/blkSize);
        if (num_cross_pages > ppn) {
            // target address smaller than page 0, ignore this request;
            return;
        }
        pf_ppn = ppn - num_cross_pages;
        pf_block = block + (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else if (block >= (pageBytes/blkSize)) {
        stride_t num_cross_pages = block / (pageBytes/blkSize);
        if (MaxAddr/pageBytes < (ppn + num_cross_pages)) {
            // target address goes beyond MaxAddr, ignore this request;
            return;
        }
        pf_ppn = ppn + num_cross_pages;
        pf_block = block - (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
    } else {
        pf_ppn = ppn;
        pf_block = block;
    }

    Addr new_addr = pf_ppn * pageBytes;
    new_addr += pf_block * (Addr)blkSize;

    DPRINTF(HWPrefetch, "Queuing prefetch to %#x.\n", new_addr);
    addresses.push_back(AddrPriority(new_addr, 0));
}

void
SignaturePath::handleSignatureTableMiss(stride_t current_block,
    signature_t &new_signature, double &new_conf, stride_t &new_stride)
{
    new_signature = current_block;
    new_conf = 1.0;
    new_stride = current_block;
}

void
SignaturePath::increasePatternEntryCounter(
        PatternEntry &pattern_entry, PatternStrideEntry &pstride_entry)
{
    pstride_entry.counter++;
}

void
SignaturePath::updatePatternTable(Addr signature, stride_t stride)
{
    assert(stride != 0);
    // The pattern table is indexed by signatures
    PatternEntry &p_entry = getPatternEntry(signature);
    PatternStrideEntry &ps_entry = p_entry.getStrideEntry(stride);
    increasePatternEntryCounter(p_entry, ps_entry);
}

SignaturePath::SignatureEntry &
SignaturePath::getSignatureEntry(Addr ppn, bool is_secure,
        stride_t block, bool &miss, stride_t &stride,
        double &initial_confidence)
{
    SignatureEntry* signature_entry = signatureTable.findEntry(ppn, is_secure);
    if (signature_entry != nullptr) {
        signatureTable.accessEntry(signature_entry);
        miss = false;
        stride = block - signature_entry->lastBlock;
    } else {
        signature_entry = signatureTable.findVictim(ppn);
        assert(signature_entry != nullptr);

        // Sets signature_entry->signature, initial_confidence, and stride
        handleSignatureTableMiss(block, signature_entry->signature,
            initial_confidence, stride);

        signatureTable.insertEntry(ppn, is_secure, signature_entry);
        miss = true;
    }
    signature_entry->lastBlock = block;
    return *signature_entry;
}

SignaturePath::PatternEntry &
SignaturePath::getPatternEntry(Addr signature)
{
    constexpr bool is_secure = false;
    PatternEntry* pattern_entry = patternTable.findEntry(signature, is_secure);
    if (pattern_entry != nullptr) {
        // Signature found
        patternTable.accessEntry(pattern_entry);
    } else {
        // Signature not found
        pattern_entry = patternTable.findVictim(signature);
        assert(pattern_entry != nullptr);

        patternTable.insertEntry(signature, is_secure, pattern_entry);
    }
    return *pattern_entry;
}

double
SignaturePath::calculatePrefetchConfidence(PatternEntry const &sig,
        PatternStrideEntry const &entry) const
{
    return entry.counter.calcSaturation();
}

double
SignaturePath::calculateLookaheadConfidence(PatternEntry const &sig,
        PatternStrideEntry const &lookahead) const
{
    double lookahead_confidence = lookahead.counter.calcSaturation();
    if (lookahead_confidence > 0.95) {
        /**
         * maximum confidence is 0.95, guaranteeing that
         * current confidence will eventually fall beyond
         * the threshold
         */
        lookahead_confidence = 0.95;
    }
    return lookahead_confidence;
}

void
SignaturePath::calculatePrefetch(const PrefetchInfo &pfi,
                                 std::vector<AddrPriority> &addresses,
                                 const CacheAccessor &cache)
{
    Addr request_addr = pfi.getAddr();
    Addr ppn = request_addr / pageBytes;
    stride_t current_block = (request_addr % pageBytes) / blkSize;
    stride_t stride;
    bool is_secure = pfi.isSecure();
    double initial_confidence = 1.0;

    // Get the SignatureEntry of this page to:
    // - compute the current stride
    // - obtain the current signature of accesses
    bool miss;
    SignatureEntry &signature_entry = getSignatureEntry(ppn, is_secure,
            current_block, miss, stride, initial_confidence);

    if (miss) {
        // No history for this page, can't continue
        return;
    }

    if (stride == 0) {
        // Can't continue with a stride 0
        return;
    }

    // Update the confidence of the current signature
    updatePatternTable(signature_entry.signature, stride);

    // Update the current SignatureEntry signature
    signature_entry.signature =
        updateSignature(signature_entry.signature, stride);

    signature_t current_signature = signature_entry.signature;
    double current_confidence = initial_confidence;
    stride_t current_stride = signature_entry.lastBlock;

    // Look for prefetch candidates while the current path confidence is
    // high enough
    while (current_confidence > lookaheadConfidenceThreshold) {
        // With the updated signature, attempt to generate prefetches
        // - search the PatternTable and select all entries with enough
        //   confidence, these are prefetch candidates
        // - select the entry with the highest counter as the "lookahead"
        PatternEntry *current_pattern_entry =
            patternTable.findEntry(current_signature, is_secure);
        PatternStrideEntry const *lookahead = nullptr;
        if (current_pattern_entry != nullptr) {
            unsigned long max_counter = 0;
            for (auto const &entry : current_pattern_entry->strideEntries) {
                //select the entry with the maximum counter value as lookahead
                if (max_counter < entry.counter) {
                    max_counter = entry.counter;
                    lookahead = &entry;
                }
                double prefetch_confidence =
                    calculatePrefetchConfidence(*current_pattern_entry, entry);

                if (prefetch_confidence >= prefetchConfidenceThreshold) {
                    assert(entry.stride != 0);
                    //prefetch candidate
                    addPrefetch(ppn, current_stride, entry.stride,
                                current_confidence, current_signature,
                                is_secure, addresses);
                }
            }
        }

        if (lookahead != nullptr) {
            current_confidence *= calculateLookaheadConfidence(
                    *current_pattern_entry, *lookahead);
            current_signature =
                updateSignature(current_signature, lookahead->stride);
            current_stride += lookahead->stride;
        } else {
            current_confidence = 0.0;
        }
    }

    auxiliaryPrefetcher(ppn, current_block, is_secure, addresses);
}

void
SignaturePath::auxiliaryPrefetcher(Addr ppn, stride_t current_block,
        bool is_secure, std::vector<AddrPriority> &addresses)
{
    if (addresses.empty()) {
        // Enable the next line prefetcher if no prefetch candidates are found
        addPrefetch(ppn, current_block, 1, 0.0 /* unused*/, 0 /* unused */,
                    is_secure, addresses);
    }
}

} // namespace prefetch
} // namespace gem5
