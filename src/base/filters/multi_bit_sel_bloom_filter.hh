/*
 * Copyright (c) 2019 Inria
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __BASE_FILTERS_MULTI_BIT_SEL_BLOOM_FILTER_HH__
#define __BASE_FILTERS_MULTI_BIT_SEL_BLOOM_FILTER_HH__

#include "base/filters/base.hh"

namespace gem5
{

struct BloomFilterMultiBitSelParams;

GEM5_DEPRECATED_NAMESPACE(BloomFilter, bloom_filter);
namespace bloom_filter
{

/**
 * The MultiBitSel Bloom Filter associates an address to multiple entries
 * through the use of multiple hash functions.
 */
class MultiBitSel : public Base
{
  public:
    MultiBitSel(const BloomFilterMultiBitSelParams &p);
    ~MultiBitSel();

    void set(Addr addr) override;
    int getCount(Addr addr) const override;

  protected:
    /**
     * Apply the selected the hash functions to an address.
     *
     * @param addr The address to hash.
     * @param hash_number Index of the hash function to be used.
     */
    virtual int hash(Addr addr, int hash_number) const;

    /** Number of hashes. */
    const int numHashes;

    /** Size of the filter when doing parallel hashing. */
    const int parFilterSize;

    /** Whether hashing should be performed in parallel. */
    const bool isParallel;

  private:
    /**
     * Bit offset from block number. Used to simulate bit selection hashing
     * on larger than cache-line granularities, by skipping some bits.
     */
    const int skipBits;
};

} // namespace bloom_filter
} // namespace gem5

#endif // __BASE_FILTERS_MULTI_BIT_SEL_BLOOM_FILTER_HH__
