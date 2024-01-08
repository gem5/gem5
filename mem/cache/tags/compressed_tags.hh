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
 */

/**
 * @file
 * Declaration of a compressed set associative tag store using superblocks.
 */

#ifndef __MEM_CACHE_TAGS_COMPRESSED_TAGS_HH__
#define __MEM_CACHE_TAGS_COMPRESSED_TAGS_HH__

#include <vector>

#include "mem/cache/tags/sector_tags.hh"
#include "mem/cache/tags/super_blk.hh"

namespace gem5
{

class BaseCache;
class CacheBlk;
struct CompressedTagsParams;

/**
 * A CompressedTags cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The Compression Ratio (CR) of a superblock is defined by
 *     CR = uncompressed_size / compressed_size.
 *
 * The CompressedTags placement policy divides the cache into s sets of w
 * superblocks (ways). Each superblock can then contain up to CR compressed
 * blocks.
 *
 * For each tag entry there can be multiple data blocks. We have the same
 * number of tags a conventional cache would have, but we instantiate the
 * maximum number of data blocks (according to the compression ratio) per
 * tag, to virtually implement compression without increasing the complexity
 * of the simulator.
 *
 * numBlocksPerSector holds the maximum number of blocks a superblock with
 * the best possible compression factor would hold. It is equivalent to CR
 * from the previous definition.
 */
class CompressedTags : public SectorTags
{
  private:
    /** The cache blocks. */
    std::vector<CompressionBlk> blks;
    /** The cache superblocks. */
    std::vector<SuperBlk> superBlks;

  public:
    /** Convenience typedef. */
     typedef CompressedTagsParams Params;

    /**
     * Construct and initialize this tag store.
     */
    CompressedTags(const Params &p);

    /**
     * Destructor.
     */
    virtual ~CompressedTags() {};

    /**
     * Initialize blocks as SuperBlk and CompressionBlk instances.
     */
    void tagsInit() override;

    /**
     * Find replacement victim based on address. Checks if data can be co-
     * allocated before choosing blocks to be evicted.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param compressed_size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         const std::size_t compressed_size,
                         std::vector<CacheBlk*>& evict_blks) override;

    /**
     * Visit each sub-block in the tags and apply a visitor.
     *
     * The visitor should be a std::function that takes a cache block.
     * reference as its parameter.
     *
     * @param visitor Visitor to call on each block.
     */
    void forEachBlk(std::function<void(CacheBlk &)> visitor) override;

    /**
     * Find if any of the sub-blocks satisfies a condition.
     *
     * The visitor should be a std::function that takes a cache block
     * reference as its parameter. The visitor will terminate the
     * traversal early if the condition is satisfied.
     *
     * @param visitor Visitor to call on each block.
     */
    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override;
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_COMPRESSED_TAGS_HH__
