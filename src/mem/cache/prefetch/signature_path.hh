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

#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

struct SignaturePathPrefetcherParams;

class SignaturePathPrefetcher : public QueuedPrefetcher
{
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
    /** Maximum pattern entries counter value */
    const uint8_t maxCounterValue;
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
        SignatureEntry() : signature(0), lastBlock(0)
        {}
    };
    /** Signature table */
    AssociativeSet<SignatureEntry> signatureTable;

    /** A stride entry with its counter */
    struct PatternStrideEntry
    {
        /** stride in a page in blkSize increments */
        stride_t stride;
        /** counter value (max value defined by maxCounterValue) */
        uint8_t counter;
        PatternStrideEntry() : stride(0), counter(0)
        {}
    };
    /** Pattern entry data type, a set of stride and counter entries */
    struct PatternEntry : public TaggedEntry
    {
        /** group of stides */
        std::vector<PatternStrideEntry> strideEntries;
        PatternEntry(size_t num_strides) : strideEntries(num_strides)
        {}

        /** Reset the entries to their initial values */
        void reset() override
        {
            for (auto &entry : strideEntries) {
                entry.counter = 0;
                entry.stride = 0;
            }
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
         * @param max_counter_value maximum value of the confidence counters,
         *        it is used when no strides are found and an entry needs to be
         *        replaced
         * @result reference to the selected entry
         */
        PatternStrideEntry &getStrideEntry(stride_t stride,
                                           uint8_t max_counter_value);
    };
    /** Pattern table */
    AssociativeSet<PatternEntry> patternTable;

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
     * @param block block number within the page, this value can be negative,
     *        which means that the block refered is actualy in the previous
     *        page (ppn-1), if the value is greater than (pageBytes/blkSize-1)
     *        then the block refers to a block within the next page (ppn+1)
     * @param is_secure whether this page is inside the secure memory area
     * @param addresses if allowed, the address will be added to this vector
     */
    void addPrefetch(Addr ppn, stride_t block, bool is_secure,
                              std::vector<AddrPriority> &addresses);

    /**
     * Obtains the SignatureEntry of the given page, if the page is not found,
     * it allocates a new one, replacing an existing entry if needed
     * @param ppn physical page number of the page
     * @param is_secure whether this page is inside the secure memory area
     * @param block accessed block within the page
     * @param miss output, if the entry is not found, this will be set to true
     * @result a reference to the SignatureEntry
     */
    SignatureEntry & getSignatureEntry(Addr ppn, bool is_secure,
                                       stride_t block, bool &miss);
    /**
     * Obtains the PatternEntry of the given signature, if the signature is
     * not found, it allocates a new one, replacing an existing entry if needed
     * @param signature the signature of the desired entry
     * @result a reference to the PatternEntry
     */
    PatternEntry & getPatternEntry(Addr signature);

    /**
     * Updates the pattern table with the provided signature and stride
     * @param signature the signature to use to index the pattern table
     * @param stride the stride to use to index the set of strides of the
     *        pattern table entry
     */
    void updatePatternTable(Addr signature, stride_t stride);

  public:
    SignaturePathPrefetcher(const SignaturePathPrefetcherParams* p);
    ~SignaturePathPrefetcher() {}
    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses) override;
};

#endif//__MEM_CACHE_PREFETCH_SIGNATURE_PATH_HH__
