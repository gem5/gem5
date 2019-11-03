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
 *
 * Authors: Daniel Carvalho
 */

/** @file
 * Implementation of a simple superblock class. Each superblock consists of a
 * number of compressed cache blocks limited by the maximum compression factor
 * that may or may not be present in the cache.
 */

#include "mem/cache/tags/super_blk.hh"

#include "base/logging.hh"

CompressionBlk::CompressionBlk()
    : SectorSubBlk(), _size(0), _decompressionLatency(0)
{
}

bool
CompressionBlk::isCompressed() const
{
    return (status & BlkCompressed) != 0;
}

void
CompressionBlk::setCompressed()
{
    status |= BlkCompressed;
}

void
CompressionBlk::setUncompressed()
{
    status &= ~BlkCompressed;
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

std::string
CompressionBlk::print() const
{
    return csprintf("%s compressed: %d size: %llu decompression latency: %d",
                    SectorSubBlk::print(), isCompressed(), getSizeBits(),
                    getDecompressionLatency());
}

bool
SuperBlk::isCompressed(const CompressionBlk* ignored_blk) const
{
    for (const auto& blk : blks) {
        if (blk->isValid() && (blk != ignored_blk)) {
            return static_cast<CompressionBlk*>(blk)->isCompressed();
        }
    }

    // An invalid block is seen as compressed
    return true;
}

bool
SuperBlk::canCoAllocate(const std::size_t compressed_size) const
{
    // Simple co-allocation function: at most numBlocksPerSector blocks that
    // compress at least to (100/numBlocksPerSector)% of their original size
    // can share a superblock
    return (compressed_size <= (blkSize * 8) / blks.size());
}

void
SuperBlk::setBlkSize(const std::size_t blk_size)
{
    assert(blkSize == 0);
    blkSize = blk_size;
}
