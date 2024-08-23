/**
 * Copyright (c) 2018, 2020 Inria
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

/** @file
 * Definition of a simple sector block class. Each sector consists of a
 * sequence of cache blocks that may or may not be present in the cache.
 */

#ifndef __MEM_CACHE_TAGS_SECTOR_BLK_HH__
#define __MEM_CACHE_TAGS_SECTOR_BLK_HH__

#include <vector>

#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

class SectorBlk;

/**
 * A sector is composed of sub-blocks, and each sub-block has information
 * regarding its sector and a pointer to its sector tag.
 */
class SectorSubBlk : public CacheBlk
{
  protected:
    /**
     * Sector block associated to this block.
     */
    SectorBlk* _sectorBlk;

    /**
     * The offset of this sub-block in the sector.
     */
    int _sectorOffset;

  public:
    SectorSubBlk() : CacheBlk(), _sectorBlk(nullptr), _sectorOffset(0) {}
    SectorSubBlk(const SectorSubBlk&) = delete;
    using CacheBlk::operator=;
    SectorSubBlk& operator=(const SectorSubBlk&) = delete;
    SectorSubBlk(SectorSubBlk&&) = delete;
    /**
     * Move assignment operator.
     * This should only be used to move an existing valid entry into an
     * invalid one, not to create a new entry. In the end the valid entry
     * will become invalid, and the invalid, valid. All location related
     * variables will remain the same, that is, an entry cannot change
     * its sector block nor its offset.
     */
    SectorSubBlk& operator=(SectorSubBlk&& other) = default;
    ~SectorSubBlk() = default;

    /**
     * Set sector block associated to this block.
     *
     * @param sector_blk The sector block pointer.
     */
    void setSectorBlock(SectorBlk* sector_blk);

    /**
     * Get sector block associated to this block.
     *
     * @return The sector block pointer.
     */
    SectorBlk* getSectorBlock() const;

    /**
     * Set offset of this sub-block within the sector.
     *
     * @param sector_offset The block's offset.
     */
    void setSectorOffset(const int sector_offset);

    /**
     * Get offset of this sub-block within the sector.
     *
     * @return sector_offset The block's offset.
     */
    int getSectorOffset() const;

    Addr getTag() const override;

    /**
     * Set valid bit and inform sector block.
     */
    void setValid() override;

    void insert(const KeyType &tag) override;

    /**
     * Invalidate the block and inform sector block.
     */
    void invalidate() override;

    /**
     * Pretty-print sector offset and other CacheBlk information.
     *
     * @return string with basic state information
     */
    std::string print() const override;
};

/**
 * A Basic Sector block.
 * Contains the tag and a list of blocks associated to this sector.
 */
class SectorBlk : public TaggedEntry
{
  private:
    /**
     * Counter of the number of valid sub-blocks. The sector is valid if any
     * of its sub-blocks is valid.
     */
    uint8_t _validCounter;

  public:
    SectorBlk();
    SectorBlk(const SectorBlk&) = delete;
    SectorBlk& operator=(const SectorBlk&) = delete;
    ~SectorBlk() {};

    /** List of blocks associated to this sector. */
    std::vector<SectorSubBlk*> blks;

    /**
     * Checks that a sector block is valid.
     *
     * @return True if any of the blocks in the sector is valid.
     */
    bool isValid() const override;

    /**
     * Get the number of sub-blocks that have been validated.
     *
     * @return The number of valid sub-blocks.
     */
    uint8_t getNumValid() const;

    /**
     * Increase the number of valid sub-blocks.
     */
    void validateSubBlk();

    /**
     * Decrease the number of valid sub-blocks.
     */
    void invalidateSubBlk();

    /**
     * Sets the position of the sub-entries, besides its own.
     *
     * @param set The set of this entry and sub-entries.
     * @param way The way of this entry and sub-entries.
     */
    void setPosition(const uint32_t set, const uint32_t way) override;

    /**
     * Print relevant information for this sector block and its sub-blocks.
     *
     * @return A string with the contents of the sector block.
     */
    std::string print() const override;
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_SECTOR_BLK_HH__
