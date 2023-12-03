/*
 * Copyright (c) 2019-2020 Inria
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

namespace gem5
{

namespace compression
{

Perfect::Perfect(const Params &p)
    : Base(p), compressedSize(8 * blkSize / p.max_compression_ratio)
{}

std::unique_ptr<Base::CompressionData>
Perfect::compress(const std::vector<Chunk> &chunks, Cycles &comp_lat,
                  Cycles &decomp_lat)
{
    // Compress every word sequentially
    std::unique_ptr<Base::CompressionData> comp_data(new CompData(chunks));

    // Set relevant metadata
    comp_data->setSizeBits(compressedSize);

    // Set latencies based on the degree of parallelization, and any extra
    // latencies due to shifting or packaging
    comp_lat = Cycles((chunks.size() / compChunksPerCycle) + compExtraLatency);
    decomp_lat =
        Cycles((chunks.size() / decompChunksPerCycle) + decompExtraLatency);

    return comp_data;
}

void
Perfect::decompress(const CompressionData *comp_data, uint64_t *data)
{
    // Decompress every entry sequentially
    fromChunks(static_cast<const CompData *>(comp_data)->chunks, data);
}

} // namespace compression
} // namespace gem5
