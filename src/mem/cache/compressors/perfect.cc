/*
 * Copyright (c) 2019 Inria
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
 * Implementation of a perfect compressor, which compresses data to its
 * maximum allowed compression ratio.
 */

#include "mem/cache/compressors/perfect.hh"

#include <algorithm>

#include "debug/CacheComp.hh"
#include "params/PerfectCompressor.hh"

PerfectCompressor::CompData::CompData(const uint64_t* data,
    std::size_t num_entries)
    : CompressionData(), entries(data, data + num_entries)
{
}

PerfectCompressor::PerfectCompressor(const Params *p)
    : BaseCacheCompressor(p),
      compressedSize(8 * blkSize / p->max_compression_ratio),
      compressionLatency(p->compression_latency),
      decompressionLatency(p->decompression_latency)
{
}

std::unique_ptr<BaseCacheCompressor::CompressionData>
PerfectCompressor::compress(const uint64_t* cache_line, Cycles& comp_lat,
    Cycles& decomp_lat)
{
    // Compress every word sequentially
    std::unique_ptr<BaseCacheCompressor::CompressionData> comp_data(
        new CompData(cache_line, blkSize/8));

    // Set relevant metadata
    comp_data->setSizeBits(compressedSize);
    comp_lat = compressionLatency;
    decomp_lat = decompressionLatency;

    return comp_data;
}

void
PerfectCompressor::decompress(const CompressionData* comp_data,
    uint64_t* data)
{
    // Decompress every entry sequentially
    const std::vector<uint64_t>& entries =
        static_cast<const CompData*>(comp_data)->entries;
    assert(entries.size() == (blkSize/8));
    std::copy(entries.begin(), entries.end(), data);
}

PerfectCompressor*
PerfectCompressorParams::create()
{
    return new PerfectCompressor(this);
}
