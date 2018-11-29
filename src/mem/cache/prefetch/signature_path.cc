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
 *
 * Authors: Javier Bueno
 */

#include "mem/cache/prefetch/signature_path.hh"

#include <cassert>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SignaturePathPrefetcher.hh"

SignaturePathPrefetcher::SignaturePathPrefetcher(
    const SignaturePathPrefetcherParams *p)
    : QueuedPrefetcher(p),
      stridesPerPatternEntry(p->strides_per_pattern_entry),
      signatureShift(p->signature_shift),
      signatureBits(p->signature_bits),
      maxCounterValue(p->max_counter_value),
      prefetchConfidenceThreshold(p->prefetch_confidence_threshold),
      lookaheadConfidenceThreshold(p->lookahead_confidence_threshold),
      signatureTable(p->signature_table_assoc, p->signature_table_entries,
                     p->signature_table_indexing_policy,
                     p->signature_table_replacement_policy),
      patternTable(p->pattern_table_assoc, p->pattern_table_entries,
                   p->pattern_table_indexing_policy,
                   p->pattern_table_replacement_policy,
                   PatternEntry(stridesPerPatternEntry))
{
}

SignaturePathPrefetcher::PatternStrideEntry &
SignaturePathPrefetcher::PatternEntry::getStrideEntry(stride_t stride,
                                                     uint8_t max_counter_value)
{
    PatternStrideEntry *pstride_entry = findStride(stride);
    if (pstride_entry == nullptr) {
        // Specific replacement algorithm for this table,
        // pick the entry with the lowest counter value,
        // then decrease the counter of all entries

        // If all counters have the max value, this will be the pick
        PatternStrideEntry *victim_pstride_entry = &(strideEntries[0]);

        uint8_t current_counter = max_counter_value;
        for (auto &entry : strideEntries) {
            if (entry.counter < current_counter) {
                victim_pstride_entry = &entry;
                current_counter = entry.counter;
            }
            if (entry.counter > 0) {
                entry.counter -= 1;
            }
        }
        pstride_entry = victim_pstride_entry;
        pstride_entry->counter = 0;
        pstride_entry->stride = stride;
    }
    return *pstride_entry;
}

void
SignaturePathPrefetcher::addPrefetch(Addr ppn, stride_t block,
    bool is_secure, std::vector<AddrPriority> &addresses)
{
    /**
     * block is relative to the provided ppn. Assuming a page size of 4kB and
     * a block size of 64B, the range of the stride of this prefetcher is
     * -63,63 (pageBytes/blkSize) as the last accessed block also ranges from
     * 0,63, the block value is expected to be between -63 and 126
     * Negative block means that we are accessing the lower contiguous page,
     * 64 or greater point to the next contiguous.
     */
    assert(block > -((stride_t)(pageBytes/blkSize)));
    assert(block < 2*((stride_t)(pageBytes/blkSize)));

    Addr pf_ppn;
    stride_t pf_block;
    if (block < 0) {
        pf_ppn = ppn - 1;
        pf_block = block + (pageBytes/blkSize);
    } else if (block >= (pageBytes/blkSize)) {
        pf_ppn = ppn + 1;
        pf_block = block - (pageBytes/blkSize);
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
SignaturePathPrefetcher::updatePatternTable(Addr signature, stride_t stride)
{
    assert(stride != 0);
    // The pattern table is indexed by signatures
    PatternEntry &p_entry = getPatternEntry(signature);
    PatternStrideEntry &ps_entry = p_entry.getStrideEntry(stride,
                                                          maxCounterValue);
    if (ps_entry.counter < maxCounterValue) {
        ps_entry.counter += 1;
    }
}

SignaturePathPrefetcher::SignatureEntry &
SignaturePathPrefetcher::getSignatureEntry(Addr ppn, bool is_secure,
                                           stride_t block, bool &miss)
{
    SignatureEntry* signature_entry = signatureTable.findEntry(ppn, is_secure);
    if (signature_entry != nullptr) {
        signatureTable.accessEntry(signature_entry);
        miss = false;
    } else {
        signature_entry = signatureTable.findVictim(ppn);
        assert(signature_entry != nullptr);

        signatureTable.insertEntry(ppn, is_secure, signature_entry);
        signature_entry->signature = block;
        signature_entry->lastBlock = block;
        miss = true;
    }
    return *signature_entry;
}

SignaturePathPrefetcher::PatternEntry &
SignaturePathPrefetcher::getPatternEntry(Addr signature)
{
    PatternEntry* pattern_entry = patternTable.findEntry(signature, false);
    if (pattern_entry != nullptr) {
        // Signature found
        patternTable.accessEntry(pattern_entry);
    } else {
        // Signature not found
        pattern_entry = patternTable.findVictim(signature);
        assert(pattern_entry != nullptr);

        patternTable.insertEntry(signature, false, pattern_entry);
    }
    return *pattern_entry;
}

void
SignaturePathPrefetcher::calculatePrefetch(const PrefetchInfo &pfi,
                                 std::vector<AddrPriority> &addresses)
{
    Addr request_addr = pfi.getAddr();
    Addr ppn = request_addr / pageBytes;
    stride_t current_block = (request_addr % pageBytes) / blkSize;
    stride_t stride;
    bool is_secure = pfi.isSecure();

    // Get the SignatureEntry of this page to:
    // - compute the current stride
    // - obtain the current signature of accesses
    bool miss;
    SignatureEntry &signature_entry = getSignatureEntry(ppn, is_secure,
                                                        current_block, miss);
    if (miss) {
        // No history for this page, can't continue
        return;
    }

    stride = current_block - signature_entry.lastBlock;
    if (stride == 0) {
        // Can't continue with a stride 0
        return;
    }

    // Update the confidence of the current signature
    updatePatternTable(signature_entry.signature, stride);

    // Update the current SignatureEntry signature and lastBlock
    signature_entry.signature =
        updateSignature(signature_entry.signature, stride);
    signature_entry.lastBlock = current_block;

    signature_t current_signature = signature_entry.signature;
    double current_confidence = 1.0;
    stride_t current_stride = signature_entry.lastBlock;

    do {
        // With the updated signature, attempt to generate prefetches
        // - search the PatternTable and select all entries with enough
        //   confidence, these are prefetch candidates
        // - select the entry with the highest counter as the "lookahead"
        PatternEntry *current_pattern_entry =
            patternTable.findEntry(current_signature, false);
        PatternStrideEntry const *lookahead = nullptr;
        if (current_pattern_entry != nullptr) {
            uint8_t max_counter = 0;
            for (auto const &entry : current_pattern_entry->strideEntries) {
                //select the entry with the maximum counter value as lookahead
                if (max_counter < entry.counter) {
                    max_counter = entry.counter;
                    lookahead = &entry;
                }
                double prefetch_confidence =
                    (double) entry.counter / maxCounterValue;

                if (prefetch_confidence >= prefetchConfidenceThreshold) {
                    assert(entry.stride != 0);
                    //prefetch candidate
                    addPrefetch(ppn, current_stride + entry.stride,
                                         is_secure, addresses);
                }
            }
        }
        if (lookahead != nullptr) {
            // If a lookahead was selected, compute its confidence using
            // the counter of its entry and the accumulated confidence
            // if the confidence is high enough, generate a new signature
            double lookahead_confidence;
            if (lookahead->counter == maxCounterValue) {
                // maximum confidence is 0.95, guaranteeing that
                // current confidence will eventually fall beyond
                // the threshold
                lookahead_confidence = 0.95;
            } else {
                lookahead_confidence =
                    ((double) lookahead->counter / maxCounterValue);
            }
            current_confidence *= lookahead_confidence;
            current_signature =
                updateSignature(current_signature, lookahead->stride);
            current_stride += lookahead->stride;
        } else {
            current_confidence = 0.0;
        }
        // If the accumulated confidence is high enough, keep repeating
        // this process with the updated signature
    }
    while (current_confidence > lookaheadConfidenceThreshold);

    if (addresses.empty()) {
        // Enable the next line prefetcher if no prefetch candidates are found
        addPrefetch(ppn, current_block + 1, is_secure, addresses);
    }
}

SignaturePathPrefetcher*
SignaturePathPrefetcherParams::create()
{
    return new SignaturePathPrefetcher(this);
}
