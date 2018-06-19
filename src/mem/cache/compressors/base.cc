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

/** @file
 * Definition of a basic cache compressor.
 */

#include "mem/cache/compressors/base.hh"

#include <algorithm>
#include <cstdint>
#include <string>

#include "debug/CacheComp.hh"
#include "mem/cache/tags/super_blk.hh"
#include "params/BaseCacheCompressor.hh"

// Uncomment this line if debugging compression
//#define DEBUG_COMPRESSION

BaseCacheCompressor::CompressionData::CompressionData()
    : _size(0)
{
}

BaseCacheCompressor::CompressionData::~CompressionData()
{
}

void
BaseCacheCompressor::CompressionData::setSizeBits(std::size_t size)
{
    _size = size;
}

std::size_t
BaseCacheCompressor::CompressionData::getSizeBits() const
{
    return _size;
}

std::size_t
BaseCacheCompressor::CompressionData::getSize() const
{
    return std::ceil(_size/8);
}

BaseCacheCompressor::BaseCacheCompressor(const Params *p)
    : SimObject(p), blkSize(p->block_size)
{
}

void
BaseCacheCompressor::compress(const uint64_t* data, Cycles& comp_lat,
                              Cycles& decomp_lat, std::size_t& comp_size_bits)
{
    // Apply compression
    std::unique_ptr<CompressionData> comp_data =
        compress(data, comp_lat, decomp_lat);

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

    // Get compression size
    comp_size_bits = comp_data->getSizeBits();

    // Update stats
    compressionSize[std::ceil(std::log2(comp_size_bits))]++;

    // Print debug information
    DPRINTF(CacheComp, "Compressed cache line from %d to %d bits. " \
            "Compression latency: %llu, decompression latency: %llu\n",
            blkSize*8, comp_size_bits, comp_lat, decomp_lat);
}

Cycles
BaseCacheCompressor::getDecompressionLatency(const CacheBlk* blk) const
{
    const CompressionBlk* comp_blk = static_cast<const CompressionBlk*>(blk);

    // If block is compressed, return its decompression latency
    if (comp_blk && comp_blk->isCompressed()){
        const Cycles decomp_lat = comp_blk->getDecompressionLatency();
        DPRINTF(CacheComp, "Decompressing block: %s (%d cycles)\n",
                comp_blk->print(), decomp_lat);
        return decomp_lat;
    }

    // Block is not compressed, so there is no decompression latency
    return Cycles(0);
}

void
BaseCacheCompressor::setDecompressionLatency(CacheBlk* blk, const Cycles lat)
{
    // Sanity check
    assert(blk != nullptr);

    // Assign latency
    static_cast<CompressionBlk*>(blk)->setDecompressionLatency(lat);
}

void
BaseCacheCompressor::setSizeBits(CacheBlk* blk, const std::size_t size_bits)
{
    // Sanity check
    assert(blk != nullptr);

    // Assign size
    static_cast<CompressionBlk*>(blk)->setSizeBits(size_bits);
}

void
BaseCacheCompressor::regStats()
{
    SimObject::regStats();

    // We also store when compression is bigger than original block size
    compressionSize
        .init(std::log2(blkSize*8) + 2)
        .name(name() + ".compression_size")
        .desc("Number of blocks that were compressed to this power of" \
              "two size.")
        ;

    for (unsigned i = 0; i <= std::log2(blkSize*8) + 1; ++i) {
        compressionSize.subname(i, std::to_string(1 << i));
        compressionSize.subdesc(i, "Number of blocks that compressed to fit " \
                                   "in " + std::to_string(1 << i) + " bits");
    }
}

