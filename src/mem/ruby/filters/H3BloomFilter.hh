/*
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

#ifndef __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__
#define __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__

#include "mem/ruby/filters/AbstractBloomFilter.hh"

struct H3BloomFilterParams;

/**
 * Implementation of the bloom filter as described in "Implementing Signatures
 * for Transactional Memory", by Sanchez, Daniel, et al.
 */
class H3BloomFilter : public AbstractBloomFilter
{
  public:
    H3BloomFilter(const H3BloomFilterParams* p);
    ~H3BloomFilter();

    void merge(const AbstractBloomFilter* other) override;
    void set(Addr addr) override;
    int getCount(Addr addr) const override;

  private:
    /**
     * Apply a hash functions to an address.
     *
     * @param addr The address to hash.
     * @param hash_number Index of the H3 hash function to be used.
     */
    int hash(Addr addr, int hash_number) const;

    /**
     * Apply one of the H3 hash functions to a value.
     *
     * @param value The value to hash.
     * @param hash_number Index of the hash function to be used.
     */
    int hashH3(uint64_t value, int hash_number) const;

    /** The number of hashes used in this filter. Can be at most 16. */
    const int numHashes;

    /** Whether hashing should be performed in parallel. */
    bool isParallel;

    /** Size of the filter when doing parallel hashing. */
    int parFilterSize;
};

#endif // __MEM_RUBY_FILTERS_H3BLOOMFILTER_HH__
