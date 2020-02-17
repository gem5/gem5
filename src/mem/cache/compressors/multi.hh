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
 * Definition of the a multi compressor that choses the best compression
 * among multiple compressors.
 */

#ifndef __MEM_CACHE_COMPRESSORS_MULTI_HH__
#define __MEM_CACHE_COMPRESSORS_MULTI_HH__

#include <cstdint>
#include <vector>

#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

struct MultiCompressorParams;

class MultiCompressor : public BaseCacheCompressor
{
  protected:
    /**
     * Compression data for the multi compressor. It contains the compression
     * data of the best compressor, along with its index in the list of
     * sub-compressors.
     */
    class MultiCompData;

    /** List of sub-compressors. */
    std::vector<BaseCacheCompressor*> compressors;

    /**
     * @defgroup CompressionStats Compression specific statistics.
     * @{
     */

    /** Number of times each compressor provided the nth best compression. */
    Stats::Vector2d rankStats;

    /**
     * @}
     */

  public:
    /** Convenience typedef. */
     typedef MultiCompressorParams Params;

    /** Default constructor. */
    MultiCompressor(const Params *p);

    /** Default destructor. */
    ~MultiCompressor();

    void regStats() override;

    std::unique_ptr<BaseCacheCompressor::CompressionData> compress(
        const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat) override;

    void decompress(const CompressionData* comp_data, uint64_t* data) override;
};

class MultiCompressor::MultiCompData : public CompressionData
{
  private:
    /** Index of the compressor that provided these compression results. */
    const uint8_t index;

  public:
    /** Compression data of the best compressor. */
    std::unique_ptr<BaseCacheCompressor::CompressionData> compData;

    /**
     * Default constructor.
     *
     * @param index Index of the compressor that provided this compression.
     * @param comp_data Compression data of the best compressor.
     */
    MultiCompData(unsigned index,
        std::unique_ptr<BaseCacheCompressor::CompressionData> comp_data);

    /** Default destructor. */
    ~MultiCompData() = default;

    /** Get the index of the best compressor. */
    uint8_t getIndex() const;
};

#endif //__MEM_CACHE_COMPRESSORS_MULTI_HH__
