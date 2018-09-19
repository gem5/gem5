/*
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho
 */

/**
 * @file
 * Declaration of a skewed associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_SKEWED_ASSOC_HH__
#define __MEM_CACHE_TAGS_SKEWED_ASSOC_HH__

#include "mem/cache/blk.hh"
#include "mem/cache/tags/base_set_assoc.hh"
#include "params/SkewedAssoc.hh"

/**
 * A SkewedAssoc cache tag store.
 * @sa \ref gem5MemorySystem "gem5 Memory System"
 *
 * The SkewedAssoc placement policy divides the cache into s sets of w
 * cache lines (ways). A cache line has a different set mapping for each way,
 * according to a skewing function.
 *
 * If provided with an associativity higher than the number of skewing
 * functions, the skewing functions of the extra ways might be sub-optimal.
 */
class SkewedAssoc : public BaseSetAssoc
{
  private:
    /**
     * The number of skewing functions implemented. Should be updated if more
     * functions are added. If more than this number of skewing functions are
     * needed (i.e., assoc > this value), we programatically generate new ones,
     * which may be sub-optimal.
     */
    const int NUM_SKEWING_FUNCTIONS = 8;

    /**
     * The amount to shift a set index to get its MSB.
     */
    const int msbShift;

    /**
     * The hash function itself. Uses the hash function H, as described in
     * "Skewed-Associative Caches", from Seznec et al. (section 3.3): It
     * applies an XOR to the MSB and LSB, shifts all bits one bit to the right,
     * and set the result of the XOR as the new MSB.
     *
     * This function is not bijective if the address has only 1 bit, as the MSB
     * and LSB will be the same, and therefore the xor will always be 0.
     *
     * @param addr The address to be hashed.
     * @param The hashed address.
     */
    Addr hash(const Addr addr) const;

    /**
     * Inverse of the hash function.
     * @sa hash().
     *
     * @param addr The address to be dehashed.
     * @param The dehashed address.
     */
    Addr dehash(const Addr addr) const;

    /**
     * Address skewing function selection. It selects and applies one of the
     * skewing functions functions based on the way provided.
     *
     * @param addr Address to be skewed. Should contain the set and tag bits.
     * @param way The cache way, used to select a hash function.
     * @return The skewed address.
     */
    Addr skew(const Addr addr, const unsigned way) const;

    /**
     * Address deskewing function (inverse of the skew function) of the given
     * way.
     * @sa skew()
     *
     * @param addr Address to be deskewed. Should contain the set and tag bits.
     * @param way The cache way, used to select a hash function.
     * @return The deskewed address.
     */
    Addr deskew(const Addr addr, const unsigned way) const;

    /**
     * Apply a skewing function to calculate address' set given a way.
     *
     * @param addr The address to calculate the set for.
     * @param way The way to get the set from.
     * @return The set index for given combination of address and way.
     */
    unsigned extractSet(Addr addr, unsigned way) const;

  public:
    /** Convenience typedef. */
     typedef SkewedAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    SkewedAssoc(const Params *p);

    /**
     * Destructor
     */
    ~SkewedAssoc() {};

    /**
     * Find all possible block locations for insertion and replacement of
     * an address. Should be called immediately before ReplacementPolicy's
     * findVictim() not to break cache resizing.
     * Returns blocks in all ways belonging to the set of the address.
     *
     * @param addr The addr to a find possible locations for.
     * @return The possible locations.
     */
    const std::vector<CacheBlk*> getPossibleLocations(Addr addr) const
                                                             override;

    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     *
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    /**
     * Regenerate the block address from the tag, set and way. Uses the
     * inverse of the skewing function.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override;
};

#endif //__MEM_CACHE_TAGS_SKEWED_ASSOC_HH__
