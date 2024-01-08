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
 * Definition of a perfect compressor, that always manages to compress to
 * its maximum compression ratio.
 */

#ifndef __MEM_CACHE_COMPRESSORS_PERFECT_COMPRESSOR_HH__
#define __MEM_CACHE_COMPRESSORS_PERFECT_COMPRESSOR_HH__

#include <cstdint>
#include <memory>
#include <vector>

#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

namespace gem5
{

struct PerfectCompressorParams;

namespace compression
{

class Perfect : public Base
{
  protected:
    class CompData;

    /** Size to which this compressor always compresses. */
    const int compressedSize;

    /** Number of cycles needed to perform compression. */
    const Cycles compressionLatency;

    /** Number of cycles needed to perform decompression. */
    const Cycles decompressionLatency;

    std::unique_ptr<CompressionData> compress(
        const std::vector<Chunk>& chunks, Cycles& comp_lat,
        Cycles& decomp_lat) override;

    void decompress(const CompressionData* comp_data, uint64_t* data) override;

  public:
    typedef PerfectCompressorParams Params;
    Perfect(const Params &p);
    ~Perfect() = default;
};

class Perfect::CompData : public CompressionData
{
  public:
    /** The original data is simply copied over to this vector. */
    std::vector<Chunk> chunks;

    /**
     * Default constructor that creates a copy of the original data.
     *
     * @param chunks The data to be compressed.
     */
    CompData(const std::vector<Chunk>& chunks)
      : CompressionData(), chunks(chunks)
    {
    }
    ~CompData() = default;
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_PERFECT_COMPRESSOR_HH__
