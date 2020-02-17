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

  public:
    CompressionBlk();
    CompressionBlk(const CompressionBlk&) = delete;
    CompressionBlk& operator=(const CompressionBlk&) = delete;
    ~CompressionBlk() {};

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

  public:
    SuperBlk() : SectorBlk(), blkSize(0) {}
    SuperBlk(const SuperBlk&) = delete;
    SuperBlk& operator=(const SuperBlk&) = delete;
    ~SuperBlk() {};

    /**
     * Returns whether the superblock contains compressed blocks or not. By
     * default, if not blocks are valid, the superblock is compressible.
     *
     * @param ignored_blk If provided don't consider the given block.
     * @return The compressibility state of the superblock.
     */
    bool isCompressed(const CompressionBlk* ignored_blk = nullptr) const;

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
};

#endif //__MEM_CACHE_TAGS_SUPER_BLK_HH__
