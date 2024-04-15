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
  * Implementation of the Signature Path Prefetcher
  *
  * References:
  *     Lookahead prefetching with signature path
  *     J Kim, PV Gratz, ALN Reddy
  *     The 2nd Data Prefetching Championship (DPC2)
  * The filter feature described in the paper is not implemented, as it
  * redundant prefetches are dropped by the cache.
  */

#ifndef __MEM_CACHE_PREFETCH_SIGNATURE_PATH_HH__
#define __MEM_CACHE_PREFETCH_SIGNATURE_PATH_HH__

#include "base/cache/associative_cache.hh"
#include "base/sat_counter.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/tags/tagged_entry.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SignaturePathPrefetcherParams;

namespace prefetch
{

class SignaturePath : public Queued
{
  protected:
    /** Signature type */
    typedef uint16_t signature_t;
    /** Stride type */
    typedef int16_t stride_t;

    /** Number of strides stored in each pattern entry */
    const unsigned stridesPerPatternEntry;
    /** Number of bits to shift when generating a new signature */
    const uint8_t signatureShift;
    /** Size of the signature, in bits */
    const signature_t signatureBits;
    /** Minimum confidence to issue a prefetch */
    const double prefetchConfidenceThreshold;
    /** Minimum confidence to keep navigating lookahead entries */
    const double lookaheadConfidenceThreshold;

    /** Signature entry data type */
    struct SignatureEntry : public TaggedEntry
    {
        /** Path signature */
        signature_t signature;
        /** Last accessed block within a page */
        stride_t lastBlock;
        SignatureEntry(TaggedIndexingPolicy *ip)
          : TaggedEntry(ip), signature(0), lastBlock(0)
        {}
    };
    /** Signature table */
    AssociativeCache<SignatureEntry> signatureTable;

    /** A stride entry with its counter */
    struct PatternStrideEntry
    {
        /** stride in a page in blkSize increments */
        stride_t stride;
        /** Saturating counter */
        SatCounter8 counter;
        PatternStrideEntry(unsigned bits) : stride(0), counter(bits)
        {}
    };
    /** Pattern entry data type, a set of stride and counter entries */
    struct PatternEntry : public TaggedEntry
    {
        /** group of stides */
        std::vector<PatternStrideEntry> strideEntries;
        /** use counter, used by SPPv2 */
        SatCounter8 counter;
        PatternEntry(size_t num_strides, unsigned counter_bits,
                     TaggedIndexingPolicy *ip)
          : TaggedEntry(ip), strideEntries(num_strides, counter_bits),
            counter(counter_bits)
        {
        }

        /** Reset the entries to their initial values */
        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            for (auto &entry : strideEntries) {
                entry.counter.reset();
                entry.stride = 0;
            }
            counter.reset();
        }

        /**
         * Returns the entry with the desired stride
         * @param stride the stride to find
         * @result a pointer to the entry, if the stride was found, or nullptr,
         *         if the stride was not found
         */
        PatternStrideEntry *findStride(stride_t stride)
        {
            PatternStrideEntry *found_entry = nullptr;
            for (auto &entry : strideEntries) {
                if (entry.stride == stride) {
                    found_entry = &entry;
                    break;
                }
            }
            return found_entry;
        }

        /**
         * Gets the entry with the provided stride, if there is no entry with
         * the associated stride, it replaces one of them.
         * @param stride the stride to find
         * @result reference to the selected entry
         */
        PatternStrideEntry &getStrideEntry(stride_t stride);
    };

    /** Pattern table */
    AssociativeCache<PatternEntry> patternTable;

    /**
     * Generates a new signature from an existing one and a new stride
     * @param sig current signature
     * @param str stride to add to the new signature
     * @result the new signature
     */
    inline signature_t updateSignature(signature_t sig, stride_t str) const {
        sig <<= signatureShift;
        sig ^= str;
        sig &= mask(signatureBits);
        return sig;
    }

    /**
     * Generates an address to be prefetched.
     * @param ppn page number to prefetch from
     * @param last_block last accessed block within the page ppn
     * @param delta difference, in number of blocks, from the last_block
     *        accessed to the block to prefetch. The block to prefetch is
     *        computed by this formula:
     *          ppn * pageBytes + (last_block + delta) * blkSize
     *        This value can be negative.
     * @param path_confidence the confidence factor of this prefetch
     * @param signature the current path signature
     * @param is_secure whether this page is inside the secure memory area
     * @param addresses addresses to prefetch will be added to this vector
     */
    void addPrefetch(Addr ppn, stride_t last_block, stride_t delta,
                          double path_confidence, signature_t signature,
                          bool is_secure,
                          std::vector<AddrPriority> &addresses);

    /**
     * Obtains the SignatureEntry of the given page, if the page is not found,
     * it allocates a new one, replacing an existing entry if needed
     * It also provides the stride of the current block and the initial
     * path confidence of the corresponding entry
     * @param ppn physical page number of the page
     * @param is_secure whether this page is inside the secure memory area
     * @param block accessed block within the page
     * @param miss if the entry is not found, this will be set to true
     * @param stride set to the computed stride
     * @param initial_confidence set to the initial confidence value
     * @result a reference to the SignatureEntry
     */
    SignatureEntry &getSignatureEntry(Addr ppn, bool is_secure, stride_t block,
            bool &miss, stride_t &stride, double &initial_confidence);
    /**
     * Obtains the PatternEntry of the given signature, if the signature is
     * not found, it allocates a new one, replacing an existing entry if needed
     * @param signature the signature of the desired entry
     * @result a reference to the PatternEntry
     */
    PatternEntry& getPatternEntry(Addr signature);

    /**
     * Updates the pattern table with the provided signature and stride
     * @param signature the signature to use to index the pattern table
     * @param stride the stride to use to index the set of strides of the
     *        pattern table entry
     */
    void updatePatternTable(Addr signature, stride_t stride);

    /**
     * Computes the lookahead path confidence of the provided pattern entry
     * @param sig the PatternEntry to use
     * @param lookahead PatternStrideEntry within the provided PatternEntry
     * @return the computed confidence factor
     */
    virtual double calculateLookaheadConfidence(PatternEntry const &sig,
            PatternStrideEntry const &lookahead) const;

    /**
     * Computes the prefetch confidence of the provided pattern entry
     * @param sig the PatternEntry to use
     * @param entry PatternStrideEntry within the provided PatternEntry
     * @return the computed confidence factor
     */
    virtual double calculatePrefetchConfidence(PatternEntry const &sig,
            PatternStrideEntry const &entry) const;

    /**
     * Increases the counter of a given PatternEntry/PatternStrideEntry
     * @param pattern_entry the corresponding PatternEntry
     * @param pstride_entry the PatternStrideEntry within the PatternEntry
     */
    virtual void increasePatternEntryCounter(PatternEntry &pattern_entry,
            PatternStrideEntry &pstride_entry);

    /**
     * Whenever a new SignatureEntry is allocated, it computes the new
     * signature to be used with the new entry, the resulting stride and the
     * initial path confidence of the new entry.
     * @param current_block accessed block within the page of the associated
              entry
     * @param new_signature new signature of the allocated entry
     * @param new_conf the initial path confidence of this entry
     * @param new_stride the resulting current stride
     */
    virtual void handleSignatureTableMiss(stride_t current_block,
            signature_t &new_signature, double &new_conf,
            stride_t &new_stride);

    /**
     * Auxiliar prefetch mechanism used at the end of calculatePrefetch.
     * This prefetcher uses this to activate the next line prefetcher if
     * no prefetch candidates have been found.
     * @param ppn physical page number of the current accessed page
     * @param current_block last accessed block within the page ppn
     * @param is_secure whether this page is inside the secure memory area
     * @param addresses the addresses to be prefetched are added to this vector
     * @param updated_filter_entries set of addresses containing these that
     *        their filter has been updated, if this call updates a new entry
     */
    virtual void auxiliaryPrefetcher(Addr ppn, stride_t current_block,
            bool is_secure, std::vector<AddrPriority> &addresses);

    /**
     * Handles the situation when the lookahead process has crossed the
     * boundaries of the current page. This is not fully described in the
     * paper that was used to implement this code, however, the article
     * describing the upgraded version of this prefetcher provides some
     * details. For this prefetcher, there are no specific actions to be
     * done.
     * @param signature the lookahead signature that crossed the page
     * @param delta the current stride that caused it
     * @param last_offset the last accessed block within the page
     * @param path_confidence the path confidence at the moment of crossing
     */
    virtual void handlePageCrossingLookahead(signature_t signature,
            stride_t last_offset, stride_t delta, double path_confidence) {
    }

  public:
    SignaturePath(const SignaturePathPrefetcherParams &p);
    ~SignaturePath() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif//__MEM_CACHE_PREFETCH_SIGNATURE_PATH_HH__
