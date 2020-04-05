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
 * Implementation of the a multi compressor that choses the best compression
 * among multiple compressors.
 */

#include "mem/cache/compressors/multi.hh"

#include <cmath>
#include <queue>

#include "base/bitfield.hh"
#include "debug/CacheComp.hh"
#include "params/MultiCompressor.hh"

MultiCompressor::MultiCompData::MultiCompData(unsigned index,
    std::unique_ptr<BaseCacheCompressor::CompressionData> comp_data)
    : CompressionData(), index(index), compData(std::move(comp_data))
{
    setSizeBits(compData->getSizeBits());
}

uint8_t
MultiCompressor::MultiCompData::getIndex() const
{
    return index;
}

MultiCompressor::MultiCompressor(const Params *p)
    : BaseCacheCompressor(p), compressors(p->compressors)
{
    fatal_if(compressors.size() == 0, "There must be at least one compressor");
}

MultiCompressor::~MultiCompressor()
{
    for (auto& compressor : compressors) {
        delete compressor;
    }
}

std::unique_ptr<BaseCacheCompressor::CompressionData>
MultiCompressor::compress(const uint64_t* cache_line, Cycles& comp_lat,
    Cycles& decomp_lat)
{
    struct Results
    {
        unsigned index;
        std::unique_ptr<BaseCacheCompressor::CompressionData> compData;
        Cycles decompLat;
        uint8_t compressionFactor;

        Results(unsigned index,
            std::unique_ptr<BaseCacheCompressor::CompressionData> comp_data,
            Cycles decomp_lat, std::size_t blk_size)
            : index(index), compData(std::move(comp_data)),
              decompLat(decomp_lat)
        {
            const std::size_t size = compData->getSize();
            // If the compressed size is worse than the uncompressed size,
            // we assume the size is the uncompressed size, and thus the
            // compression factor is 1
            compressionFactor = (size > blk_size) ? 1 :
                alignToPowerOfTwo(std::floor(blk_size / (double) size));
        }
    };
    struct ResultsComparator
    {
        bool
        operator()(const std::shared_ptr<Results>& lhs,
            const std::shared_ptr<Results>& rhs) const
        {
            const std::size_t lhs_cf = lhs->compressionFactor;
            const std::size_t rhs_cf = rhs->compressionFactor;

            if (lhs_cf == rhs_cf) {
                // When they have similar compressed sizes, give the one
                // with fastest decompression privilege
                return lhs->decompLat > rhs->decompLat;
            }
            return lhs_cf < rhs_cf;
        }
    };

    // Find the ranking of the compressor outputs
    std::priority_queue<std::shared_ptr<Results>,
        std::vector<std::shared_ptr<Results>>, ResultsComparator> results;
    Cycles max_comp_lat;
    for (unsigned i = 0; i < compressors.size(); i++) {
        Cycles temp_decomp_lat;
        auto temp_comp_data =
            compressors[i]->compress(cache_line, comp_lat, temp_decomp_lat);
        results.push(std::make_shared<Results>(i, std::move(temp_comp_data),
            temp_decomp_lat, blkSize));
        max_comp_lat = std::max(max_comp_lat, comp_lat);
    }

    // Assign best compressor to compression data
    const unsigned best_index = results.top()->index;
    std::unique_ptr<CompressionData> multi_comp_data =
        std::unique_ptr<MultiCompData>(
            new MultiCompData(best_index, std::move(results.top()->compData)));
    DPRINTF(CacheComp, "Best compressor: %d\n", best_index);

    // Set decompression latency of the best compressor
    decomp_lat = results.top()->decompLat;

    // Update compressor ranking stats
    for (int rank = 0; rank < compressors.size(); rank++) {
        rankStats[results.top()->index][rank]++;
        results.pop();
    }

    // Set compression latency (compression latency of the slowest compressor
    // and 1 cycle to pack)
    comp_lat = Cycles(max_comp_lat + 1);

    return multi_comp_data;
}

void
MultiCompressor::decompress(const CompressionData* comp_data,
    uint64_t* cache_line)
{
    const MultiCompData* casted_comp_data =
        static_cast<const MultiCompData*>(comp_data);
    compressors[casted_comp_data->getIndex()]->decompress(
        casted_comp_data->compData.get(), cache_line);
}

void
MultiCompressor::regStats()
{
    BaseCacheCompressor::regStats();

    rankStats
        .init(compressors.size(), compressors.size())
        .name(name() + ".rank")
        .desc("Number of times each compressor had the nth best compression.")
        ;

    for (int compressor = 0; compressor < compressors.size(); compressor++) {
        rankStats.subname(compressor, std::to_string(compressor));
        rankStats.subdesc(compressor, "Number of times compressor " +
            std::to_string(compressor) + " had the nth best compression.");
        for (unsigned rank = 0; rank < compressors.size(); rank++) {
            rankStats.ysubname(rank, std::to_string(rank));
        }
    }
}

MultiCompressor*
MultiCompressorParams::create()
{
    return new MultiCompressor(this);
}
