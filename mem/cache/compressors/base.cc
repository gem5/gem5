/*
 * Copyright (c) 2018-2020 Inria
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
 * Definition of a basic cache compressor.
 */

#include "mem/cache/compressors/base.hh"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <string>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/CacheComp.hh"
#include "mem/cache/base.hh"
#include "mem/cache/tags/super_blk.hh"
#include "params/BaseCacheCompressor.hh"

namespace gem5
{

namespace compression
{

// Uncomment this line if debugging compression
//#define DEBUG_COMPRESSION

Base::CompressionData::CompressionData()
    : _size(0)
{
}

Base::CompressionData::~CompressionData()
{
}

void
Base::CompressionData::setSizeBits(std::size_t size)
{
    _size = size;
}

std::size_t
Base::CompressionData::getSizeBits() const
{
    return _size;
}

std::size_t
Base::CompressionData::getSize() const
{
    return std::ceil(_size/8);
}

Base::Base(const Params &p)
  : SimObject(p), blkSize(p.block_size), chunkSizeBits(p.chunk_size_bits),
    sizeThreshold((blkSize * p.size_threshold_percentage) / 100),
    compChunksPerCycle(p.comp_chunks_per_cycle),
    compExtraLatency(p.comp_extra_latency),
    decompChunksPerCycle(p.decomp_chunks_per_cycle),
    decompExtraLatency(p.decomp_extra_latency),
    cache(nullptr), stats(*this)
{
    fatal_if(64 % chunkSizeBits,
        "64 must be a multiple of the chunk granularity.");

    fatal_if(((CHAR_BIT * blkSize) / chunkSizeBits) < compChunksPerCycle,
        "Compressor processes more chunks per cycle than the number of "
        "chunks in the input");
    fatal_if(((CHAR_BIT * blkSize) / chunkSizeBits) < decompChunksPerCycle,
        "Decompressor processes more chunks per cycle than the number of "
        "chunks in the input");

    fatal_if(blkSize < sizeThreshold, "Compressed data must fit in a block");
}

void
Base::setCache(BaseCache *_cache)
{
    assert(!cache);
    cache = _cache;
}

std::vector<Base::Chunk>
Base::toChunks(const uint64_t* data) const
{
    // Number of chunks in a 64-bit value
    const unsigned num_chunks_per_64 =
        (sizeof(uint64_t) * CHAR_BIT) / chunkSizeBits;

    // Turn a 64-bit array into a chunkSizeBits-array
    std::vector<Chunk> chunks((blkSize * CHAR_BIT) / chunkSizeBits, 0);
    for (int i = 0; i < chunks.size(); i++) {
        const int index_64 = std::floor(i / (double)num_chunks_per_64);
        const unsigned start = i % num_chunks_per_64;
        chunks[i] = bits(data[index_64],
            (start + 1) * chunkSizeBits - 1, start * chunkSizeBits);
    }

    return chunks;
}

void
Base::fromChunks(const std::vector<Chunk>& chunks, uint64_t* data) const
{
    // Number of chunks in a 64-bit value
    const unsigned num_chunks_per_64 =
        (sizeof(uint64_t) * CHAR_BIT) / chunkSizeBits;

    // Turn a chunkSizeBits-array into a 64-bit array
    std::memset(data, 0, blkSize);
    for (int i = 0; i < chunks.size(); i++) {
        const int index_64 = std::floor(i / (double)num_chunks_per_64);
        const unsigned start = i % num_chunks_per_64;
        replaceBits(data[index_64], (start + 1) * chunkSizeBits - 1,
            start * chunkSizeBits, chunks[i]);
    }
}

std::unique_ptr<Base::CompressionData>
Base::compress(const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat)
{
    // Apply compression
    std::unique_ptr<CompressionData> comp_data =
        compress(toChunks(data), comp_lat, decomp_lat);

    // If we are in debug mode apply decompression just after the compression.
    // If the results do not match, we've got an error
    #ifdef DEBUG_COMPRESSION
    uint64_t decomp_data[blkSize/8];

    // Apply decompression
    decompress(comp_data.get(), decomp_data);

    // Check if decompressed line matches original cache line
    fatal_if(std::memcmp(data, decomp_data, blkSize),
             "Decompressed line does not match original line.");
    #endif

    // Get compression size. If compressed size is greater than the size
    // threshold, the compression is seen as unsuccessful
    std::size_t comp_size_bits = comp_data->getSizeBits();
    if (comp_size_bits > sizeThreshold * CHAR_BIT) {
        comp_size_bits = blkSize * CHAR_BIT;
        comp_data->setSizeBits(comp_size_bits);
        stats.failedCompressions++;
    }

    // Update stats
    stats.compressions++;
    stats.compressionSizeBits += comp_size_bits;
    if (comp_size_bits != 0) {
        stats.compressionSize[1 + std::ceil(std::log2(comp_size_bits))]++;
    } else {
        stats.compressionSize[0]++;
    }

    // Print debug information
    DPRINTF(CacheComp, "Compressed cache line from %d to %d bits. " \
            "Compression latency: %llu, decompression latency: %llu\n",
            blkSize*8, comp_size_bits, comp_lat, decomp_lat);

    return comp_data;
}

Cycles
Base::getDecompressionLatency(const CacheBlk* blk)
{
    const CompressionBlk* comp_blk = static_cast<const CompressionBlk*>(blk);

    // If block is compressed, return its decompression latency
    if (comp_blk && comp_blk->isCompressed()){
        const Cycles decomp_lat = comp_blk->getDecompressionLatency();
        DPRINTF(CacheComp, "Decompressing block: %s (%d cycles)\n",
                comp_blk->print(), decomp_lat);
        stats.decompressions += 1;
        return decomp_lat;
    }

    // Block is not compressed, so there is no decompression latency
    return Cycles(0);
}

void
Base::setDecompressionLatency(CacheBlk* blk, const Cycles lat)
{
    // Sanity check
    assert(blk != nullptr);

    // Assign latency
    static_cast<CompressionBlk*>(blk)->setDecompressionLatency(lat);
}

void
Base::setSizeBits(CacheBlk* blk, const std::size_t size_bits)
{
    // Sanity check
    assert(blk != nullptr);

    // Assign size
    static_cast<CompressionBlk*>(blk)->setSizeBits(size_bits);
}

Base::BaseStats::BaseStats(Base& _compressor)
  : statistics::Group(&_compressor), compressor(_compressor),
    ADD_STAT(compressions, statistics::units::Count::get(),
             "Total number of compressions"),
    ADD_STAT(failedCompressions, statistics::units::Count::get(),
             "Total number of failed compressions"),
    ADD_STAT(compressionSize, statistics::units::Count::get(),
             "Number of blocks that were compressed to this power of two "
             "size"),
    ADD_STAT(compressionSizeBits, statistics::units::Bit::get(),
             "Total compressed data size"),
    ADD_STAT(avgCompressionSizeBits, statistics::units::Rate<
                statistics::units::Bit, statistics::units::Count>::get(),
             "Average compression size"),
    ADD_STAT(decompressions, statistics::units::Count::get(),
             "Total number of decompressions")
{
}

void
Base::BaseStats::regStats()
{
    statistics::Group::regStats();

    // Values comprised are {0, 1, 2, 4, ..., blkSize}
    compressionSize.init(std::log2(compressor.blkSize*8) + 2);
    compressionSize.subname(0, "0");
    compressionSize.subdesc(0,
        "Number of blocks that compressed to fit in 0 bits");
    for (unsigned i = 0; i <= std::log2(compressor.blkSize*8); ++i) {
        std::string str_i = std::to_string(1 << i);
        compressionSize.subname(1+i, str_i);
        compressionSize.subdesc(1+i,
            "Number of blocks that compressed to fit in " + str_i + " bits");
    }

    avgCompressionSizeBits.flags(statistics::total | statistics::nozero |
        statistics::nonan);
    avgCompressionSizeBits = compressionSizeBits / compressions;
}

} // namespace compression
} // namespace gem5
