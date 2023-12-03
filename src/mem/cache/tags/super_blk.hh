/**
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

/** @file
 * Definition of a simple superblock class. Each superblock consists of a
 * number of compressed cache blocks limited by the maximum compression
 * factor that may or may not be present in the cache.
 */

#ifndef __MEM_CACHE_TAGS_SUPER_BLK_HH__
#define __MEM_CACHE_TAGS_SUPER_BLK_HH__

#include "mem/cache/tags/sector_blk.hh"

namespace gem5
{

class SuperBlk;

/**
 * A superblock is composed of sub-blocks, and each sub-block has information
 * regarding its superblock and a pointer to its superblock tag. A superblock
 * can be seen as a variation of a sector block, and therefore we use a sector
 * nomenclature.
 */
class CompressionBlk : public SectorSubBlk
{
  private:
    /**
     * Set size, in bits, of this compressed block's data.
     */
    std::size_t _size;

    /**
     * Number of cycles needed to decompress this block. We store it to avoid
     * doing decompressions.
     */
    Cycles _decompressionLatency;

    /** Compression bit. */
    bool _compressed;

  public:
    /**
     * When an overwrite happens, the data size may change an not fit in its
     * current container any longer. This enum declared which kind of size
     * change happened in this situation.
     */
    enum OverwriteType : int
    {
        /** New data contents are considered smaller than previous contents. */
        DATA_CONTRACTION = -1,
        /** New and old contents are considered of similar sizes. */
        UNCHANGED = 0,
        /** New data contents are considered larger than previous contents. */
        DATA_EXPANSION = 1,
    };

    CompressionBlk();
    CompressionBlk(const CompressionBlk &) = delete;
    CompressionBlk &operator=(const CompressionBlk &) = delete;
    CompressionBlk(CompressionBlk &&) = delete;
    /**
     * Move assignment operator.
     * This should only be used to move an existing valid entry into an
     * invalid one, not to create a new entry. In the end the valid entry
     * will become invalid, and the invalid, valid. All location related
     * variables will remain the same.
     */
    CompressionBlk &operator=(CompressionBlk &&other);
    CacheBlk &operator=(CacheBlk &&other) override;
    ~CompressionBlk() = default;

    /**
     * Check if this block holds compressed data.
     *
     * @return True if the block holds compressed data.
     */
    bool isCompressed() const;

    /**
     * Set compression bit.
     */
    void setCompressed();

    /**
     * Clear compression bit.
     */
    void setUncompressed();

    /*
     * Get size, in bits, of this compressed block's data.
     *
     * @return The compressed size.
     */
    std::size_t getSizeBits() const;

    /**
     * Set size, in bits, of this compressed block's data.
     *
     * @param The compressed size.
     */
    void setSizeBits(const std::size_t size);

    /**
     * Get number of cycles needed to decompress this block.
     *
     * @return Decompression latency.
     */
    Cycles getDecompressionLatency() const;

    /**
     * Set number of cycles needed to decompress this block.
     *
     * @param Decompression latency.
     */
    void setDecompressionLatency(const Cycles lat);

    void invalidate() override;

    /**
     * Determines if changing the size of the block will cause a data
     * expansion (new size is bigger) or contraction (new size is smaller).
     * Sizes are not necessarily compared at at bit granularities (e.g., 20
     * bits is considered equal to 23 bits when blocks use 32-bit spaces as
     * minimum allocation units).
     *
     * @param size The new compressed size.
     * @return Type of size change. @sa OverwriteType.
     */
    OverwriteType checkExpansionContraction(const std::size_t size) const;

    /**
     * Pretty-print sector offset and other CacheBlk information.
     *
     * @return string with basic state information
     */
    std::string print() const override;
};

/**
 * A basic compression superblock.
 * Contains the tag and a list of blocks associated to this superblock.
 */
class SuperBlk : public SectorBlk
{
  protected:
    /** Block size, in bytes. */
    std::size_t blkSize;

    /**
     * Superblock's compression factor. It is aligned to be a power of two,
     * limited by the maximum compression ratio, and calculated as:
     *   compressionFactor = uncompressedSize/compressedSize
     */
    uint8_t compressionFactor;

  public:
    SuperBlk();
    SuperBlk(const SuperBlk &) = delete;
    SuperBlk &operator=(const SuperBlk &) = delete;
    ~SuperBlk(){};

    /**
     * Returns whether the superblock contains compressed blocks or not. By
     * default, if not blocks are valid, the superblock is compressible.
     *
     * @param ignored_blk If provided don't consider the given block.
     * @return The compressibility state of the superblock.
     */
    bool isCompressed(const CompressionBlk *ignored_blk = nullptr) const;

    /**
     * Checks whether a superblock can co-allocate given compressed data block.
     *
     * @param compressed_size Size, in bits, of new block to allocate.
     * @return True if block can be co-allocated in superblock.
     */
    bool canCoAllocate(const std::size_t compressed_size) const;

    /**
     * Set block size. Should be called only once, when initializing blocks.
     *
     * @param blk_size The uncompressed block size.
     */
    void setBlkSize(const std::size_t blk_size);

    /**
     * Calculate the compression factor (cf) given a compressed size and the
     * maximum compression ratio. Therefore cf is:
     *  1 if comp_size > blk_size/2,
     *  2 if comp_size > blk_size/4,
     *  4 if comp_size > blk_size/8,
     *  8 if comp_size > blk_size/16,
     * and so on.
     *
     * @param size The compressed size.
     * @return Compression factor corresponding to the size.
     */
    uint8_t calculateCompressionFactor(const std::size_t size) const;

    /**
     * Get the compression factor of this superblock.
     *
     * @return The compression factor.
     */
    uint8_t getCompressionFactor() const;

    /**
     * Set the compression factor of this superblock.
     *
     * @param compression_factor The new compression factor.
     */
    void setCompressionFactor(const uint8_t compression_factor);

    void invalidate() override;

    std::string print() const override;
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_SUPER_BLK_HH__
