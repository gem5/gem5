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

#ifndef __BASE_FILTERS_BASE_HH__
#define __BASE_FILTERS_BASE_HH__

#include <vector>

#include "base/intmath.hh"
#include "base/sat_counter.hh"
#include "base/types.hh"
#include "params/BloomFilterBase.hh"
#include "sim/sim_object.hh"

namespace BloomFilter {

class Base : public SimObject
{
  protected:
    /** Number of LSB bits to ignore from the the addresses. */
    const unsigned offsetBits;

    /** The filter itself. */
    std::vector<SatCounter> filter;

    /** Number of bits needed to represent the size of the filter. */
    const int sizeBits;

    /** Threshold at which a filter entry starts being considered as set. */
    const int setThreshold;

  public:
    /**
     * Create and clear the filter.
     */
    Base(const BloomFilterBaseParams* p)
        : SimObject(p), offsetBits(p->offset_bits),
          filter(p->size, SatCounter(p->num_bits)),
          sizeBits(floorLog2(p->size)), setThreshold(p->threshold)
    {
        clear();
    }
    virtual ~Base() {};

    /**
     * Clear the filter by resetting all values.
     */
    virtual void clear()
    {
        for (auto& entry : filter) {
            entry.reset();
        }
    }

    /**
     * Merges the contents of both filters into this' (Bloom Filter union).
     * Both must have the same number of entries.
     *
     * @param other The other bloom filter to merge with.
     */
    virtual void
    merge(const Base* other)
    {
        assert(filter.size() == other->filter.size());
        for (int i = 0; i < filter.size(); ++i){
            filter[i] += other->filter[i];
        }
    }

    /**
     * Perform the filter specific function to set the corresponding
     * entries (can be multiple) of an address.
     *
     * @param addr The address being parsed.
     */
    virtual void set(Addr addr) = 0;

    /**
     * Perform the filter specific function to clear the corresponding
     * entries (can be multiple) of an address. By default a bloom
     * filter does not support element deletion.
     *
     * @param addr The address being parsed.
     */
    virtual void unset(Addr addr) {};

    /**
     * Check if the corresponding filter entries of an address should be
     * considered as set.
     *
     * @param addr The address being parsed.
     * @return Whether the respective filter entry is set.
     */
    virtual bool
    isSet(Addr addr) const
    {
        return getCount(addr) >= setThreshold;
    }

    /**
     * Get the value stored in the corresponding filter entry of an address.
     *
     * @param addr The address being parsed.
     * @param Get the value stored in the respective filter entry.
     */
    virtual int getCount(Addr addr) const { return 0; }

    /**
     * Get the total value stored in the filter entries.
     *
     * @return The sum of all filter entries.
     */
    virtual int getTotalCount() const
    {
        int count = 0;
        for (const auto& entry : filter) {
            count += entry;
        }
        return count;
    }
};

} // namespace BloomFilter

#endif // __BASE_FILTERS_BASE_HH__
