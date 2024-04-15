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
 * Implementation of a simple superblock class. Each superblock consists of a
 * number of compressed cache blocks limited by the maximum compression factor
 * that may or may not be present in the cache.
 */

#include "mem/cache/tags/super_blk.hh"

#include <climits>
#include <cmath>

#include "base/bitfield.hh"

namespace gem5
{

CompressionBlk::CompressionBlk()
    : SectorSubBlk(), _size(0), _decompressionLatency(0), _compressed(false)
{}

CacheBlk &
CompressionBlk::operator=(CacheBlk &&other)
{
    operator=(std::move(static_cast<CompressionBlk &&>(other)));
    return *this;
}

CompressionBlk &
CompressionBlk::operator=(CompressionBlk &&other)
{
    // Copy internal variables; if moving, that means we had an expansion or
    // contraction, and therefore the size is no longer valid, so it is not
    // moved
    setDecompressionLatency(other.getDecompressionLatency());
    if (other.isCompressed()) {
        setCompressed();
    } else {
        setUncompressed();
    }

    CacheBlk::operator=(std::move(other));
    return *this;
}

bool
CompressionBlk::isCompressed() const
{
    return _compressed;
}

void
CompressionBlk::setCompressed()
{
    _compressed = true;
}

void
CompressionBlk::setUncompressed()
{
    _compressed = false;
}

std::size_t
CompressionBlk::getSizeBits() const
{
    return _size;
}

void
CompressionBlk::setSizeBits(const std::size_t size)
{
    _size = size;

    SuperBlk *superblock = static_cast<SuperBlk *>(getSectorBlock());
    const uint8_t compression_factor =
        superblock->calculateCompressionFactor(size);
    superblock->setCompressionFactor(compression_factor);

    // Either this function is called after an insertion, or an update.
    // If somebody else is present in the block, keep the superblock's
    // compressibility. Otherwise, check if it can co-allocate
    const uint8_t num_valid = superblock->getNumValid();
    assert(num_valid >= 1);
    if (num_valid == 1) {
        if (compression_factor != 1) {
            setCompressed();
        } else {
            setUncompressed();
        }
    } else {
        if (superblock->isCompressed(this)) {
            setCompressed();
        } else {
            setUncompressed();
        }
    }
}

Cycles
CompressionBlk::getDecompressionLatency() const
{
    return _decompressionLatency;
}

void
CompressionBlk::setDecompressionLatency(const Cycles lat)
{
    _decompressionLatency = lat;
}

void
CompressionBlk::invalidate()
{
    SectorSubBlk::invalidate();
    setUncompressed();
}

CompressionBlk::OverwriteType
CompressionBlk::checkExpansionContraction(const std::size_t size) const
{
    // An expansion happens when a block passes from a compressible state
    // to a less compressible state (e.g., blkSize/4 to (blkSize/2 or blkSize),
    // or blkSize/2 to blkSize). A contraction happens when a block passes
    // from a less compressible state to a more compressible state (i.e., the
    // opposite of expansion)
    const SuperBlk *superblock =
        static_cast<const SuperBlk *>(getSectorBlock());
    const uint8_t prev_cf = superblock->getCompressionFactor();
    const uint8_t new_cf = superblock->calculateCompressionFactor(size);
    return (new_cf < prev_cf) ?
               DATA_EXPANSION :
               ((new_cf > prev_cf) ? DATA_CONTRACTION : UNCHANGED);
}

std::string
CompressionBlk::print() const
{
    return csprintf("%s compressed: %d size: %llu decompression latency: %d",
                    SectorSubBlk::print(), isCompressed(), getSizeBits(),
                    getDecompressionLatency());
}

SuperBlk::SuperBlk() : SectorBlk(), blkSize(0), compressionFactor(1) {}

void
SuperBlk::invalidate()
{
    SectorBlk::invalidate();
    compressionFactor = 1;
}

bool
SuperBlk::isCompressed(const CompressionBlk *ignored_blk) const
{
    for (const auto &blk : blks) {
        if (blk->isValid() && (blk != ignored_blk)) {
            return static_cast<CompressionBlk *>(blk)->isCompressed();
        }
    }

    // An invalid block is seen as compressed
    return true;
}

bool
SuperBlk::canCoAllocate(const std::size_t compressed_size) const
{
    // A YACC-like (Sardashti et al., 2016) co-allocation function: at most
    // numBlocksPerSector blocks that compress at least to fit in the space
    // allocated by its compression factor can share a superblock
    return (getNumValid() < getCompressionFactor()) &&
           (compressed_size <= (blkSize * CHAR_BIT) / getCompressionFactor());
}

void
SuperBlk::setBlkSize(const std::size_t blk_size)
{
    assert(blkSize == 0);
    blkSize = blk_size;
}

uint8_t
SuperBlk::calculateCompressionFactor(const std::size_t size) const
{
    // The number of blocks per sector determines the maximum comp factor.
    // If the compressed size is worse than the uncompressed size, we assume
    // the size is the uncompressed size, and thus the compression factor is 1
    const std::size_t blk_size_bits = CHAR_BIT * blkSize;
    const std::size_t compression_factor =
        (size > blk_size_bits) ?
            1 :
            ((size == 0) ?
                 blk_size_bits :
                 alignToPowerOfTwo(std::floor(double(blk_size_bits) / size)));
    return std::min(compression_factor, blks.size());
}

uint8_t
SuperBlk::getCompressionFactor() const
{
    return compressionFactor;
}

void
SuperBlk::setCompressionFactor(const uint8_t compression_factor)
{
    // Either the block is alone, in which case the compression factor
    // must be set, or it co-allocates with someone with a worse or
    // equal compression factor, in which case it should not be updated
    if (getNumValid() <= 1) {
        compressionFactor = compression_factor;
    }
}

std::string
SuperBlk::print() const
{
    return csprintf("CF: %d %s", getCompressionFactor(), SectorBlk::print());
}

} // namespace gem5
