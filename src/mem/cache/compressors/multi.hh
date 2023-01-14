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
 * Definition of the a multi compressor that choses the best compression
 * among multiple compressors.
 */

#ifndef __MEM_CACHE_COMPRESSORS_MULTI_HH__
#define __MEM_CACHE_COMPRESSORS_MULTI_HH__

#include <cstdint>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

namespace gem5
{

struct MultiCompressorParams;

namespace compression
{

class Multi : public Base
{
  protected:
    /**
     * Compression data for the multi compressor. It contains the compression
     * data of the best compressor, along with its index in the list of
     * sub-compressors.
     */
    class MultiCompData;

    /** List of sub-compressors. */
    std::vector<Base*> compressors;

    /**
     * An encoding is associated to each sub-compressor to inform which
     * sub-compressor to use when decompressing data. This information can
     * be added either to the tag entry, in which case no extra bits are
     * added to the compressed data (numEncodingBits = 0), or to the
     * compressed data itself.
     *
     * There is no encoding reserved for the uncompressed case; it is assumed
     * that an "is compressed" bit is stored in the tags. Therefore, even if
     * storing the encoding within the compressed data, these extra bits are
     * not added when the data is uncompressible.
     *
     * These extra bits are taken into account when thresholding the
     * compressed data's size.
     */
    const std::size_t numEncodingBits;

    /**
     * Extra decompression latency to be added to the sub-compressor's
     * decompression latency. This can different from zero due to decoding,
     * shifting, or packaging, for example.
     */
    const Cycles extraDecompressionLatency;

    struct MultiStats : public statistics::Group
    {
        const Multi& compressor;

        MultiStats(BaseStats &base_group, Multi& _compressor);

        void regStats() override;

        /**
         * Number of times each compressor provided the nth best compression.
         */
        statistics::Vector2d ranks;
    } multiStats;

  public:
    typedef MultiCompressorParams Params;
    Multi(const Params &p);
    ~Multi();

    void setCache(BaseCache *_cache) override;

    std::unique_ptr<Base::CompressionData> compress(
        const std::vector<Base::Chunk>& chunks,
        Cycles& comp_lat, Cycles& decomp_lat) override;

    void decompress(const CompressionData* comp_data, uint64_t* data) override;
};

class Multi::MultiCompData : public CompressionData
{
  private:
    /** Index of the compressor that provided these compression results. */
    const uint8_t index;

  public:
    /** Compression data of the best compressor. */
    std::unique_ptr<Base::CompressionData> compData;

    /**
     * Default constructor.
     *
     * @param index Index of the compressor that provided this compression.
     * @param comp_data Compression data of the best compressor.
     */
    MultiCompData(unsigned index,
        std::unique_ptr<Base::CompressionData> comp_data);

    /** Default destructor. */
    ~MultiCompData() = default;

    /** Get the index of the best compressor. */
    uint8_t getIndex() const;
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_MULTI_HH__
