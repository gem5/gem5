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
{
}

CacheBlk&
CompressionBlk::operator=(CacheBlk&& other)
{
    operator=(std::move(static_cast<CompressionBlk&&>(other)));
    return *this;
}

CompressionBlk&
CompressionBlk::operator=(CompressionBlk&& other)
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

    SectorSubBlk::operator=(std::move(other));
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

    SuperBlk* superblock = static_cast<SuperBlk*>(getSectorBlock());
    assert(superblock);
    superblock->updateCompressionFactor();

    const uint8_t compression_factor =
        superblock->calculateCompressionFactor(size);
    if (compression_factor != 1) {
        setCompressed();
    } else {
        setUncompressed();
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
    _size = 0;
    SuperBlk *superblock = static_cast<SuperBlk *>(getSectorBlock());
    if (superblock) {
        superblock->updateCompressionFactor();
    }
}

CompressionBlk::OverwriteType
CompressionBlk::checkExpansionContraction(const std::size_t size) const
{
    // An expansion happens when a block passes from a compressible state
    // to a less compressible state (e.g., blkSize/4 to (blkSize/2 or blkSize),
    // or blkSize/2 to blkSize). A contraction happens when a block passes
    // from a less compressible state to a more compressible state (i.e., the
    // opposite of expansion)
    const SuperBlk* superblock =
        static_cast<const SuperBlk*>(getSectorBlock());
    const uint8_t prev_cf = superblock->getCompressionFactor();
    const uint8_t new_cf = superblock->calculateCompressionFactor(size);
    return (new_cf < prev_cf) ? DATA_EXPANSION :
        ((new_cf > prev_cf) ? DATA_CONTRACTION : UNCHANGED);
}

std::string
CompressionBlk::print() const
{
    return csprintf("%s compressed: %d size: %llu decompression latency: %d",
                    SectorSubBlk::print(), isCompressed(), getSizeBits(),
                    getDecompressionLatency());
}

SuperBlk::SuperBlk()
    : SectorBlk(), blkSize(0), compressionFactor(1)
{
}

void
SuperBlk::invalidate()
{
    SectorBlk::invalidate();
    compressionFactor = 1;
}

bool
SuperBlk::isCompressed(const CompressionBlk* ignored_blk) const
{
    for (const auto& blk : blks) {
        if (blk->isValid() && (blk != ignored_blk)) {
            if (!static_cast<const CompressionBlk *>(blk)->isCompressed()) {
                return false;
            }
        }
    }

    // An invalid block is seen as compressed
    return true;
}

bool
SuperBlk::canCoAllocate(const std::size_t compressed_size) const
{
    if (!isCompressed()) {
        return false;
    }

    const uint8_t new_blk_cf = calculateCompressionFactor(compressed_size);
    if (new_blk_cf <= 1) {
        return false;
    }

    std::size_t bit_sum = 0;
    for (const auto &blk : blks) {
        if (blk->isValid()) {
            const CompressionBlk *cblk =
                static_cast<const CompressionBlk *>(blk);
            bit_sum += cblk->getSizeBits();
        }
    }

    return (bit_sum + compressed_size) <= (blkSize * CHAR_BIT);
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
    const std::size_t compression_factor = (size > blk_size_bits) ? 1 :
        ((size == 0) ? blk_size_bits :
        alignToPowerOfTwo(std::floor(double(blk_size_bits) / size)));
    return std::min<std::size_t>(compression_factor, blks.size());
}

uint8_t
SuperBlk::getCompressionFactor() const
{
    return compressionFactor;
}

void
SuperBlk::setCompressionFactor(const uint8_t compression_factor)
{
    compressionFactor = compression_factor;
}

void
SuperBlk::updateCompressionFactor()
{
    uint8_t min_cf = blks.size();
    bool has_valid = false;
    for (const auto &blk : blks) {
        if (blk->isValid()) {
            has_valid = true;
            const CompressionBlk *cblk =
                static_cast<const CompressionBlk *>(blk);
            uint8_t cf = calculateCompressionFactor(cblk->getSizeBits());
            if (cf < min_cf) {
                min_cf = cf;
            }
        }
    }
    setCompressionFactor(has_valid ? min_cf : 1);
}

std::string
SuperBlk::print() const
{
    return csprintf("CF: %d %s", getCompressionFactor(), SectorBlk::print());
}

} // namespace gem5
