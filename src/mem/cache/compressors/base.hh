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
 * A cache compressor must consist of a compression and a decompression
 * methods. It must also be aware of the size of an uncompressed cache
 * line.
 */

#ifndef __MEM_CACHE_COMPRESSORS_BASE_HH__
#define __MEM_CACHE_COMPRESSORS_BASE_HH__

#include <cstdint>

#include "base/compiler.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseCache;
class CacheBlk;
struct BaseCacheCompressorParams;

namespace compression
{

/**
 * Base cache compressor interface. Every cache compressor must implement a
 * compression and a decompression method.
 *
 * Compressors usually cannot parse all data input at once. Therefore, they
 * typically divide the input into multiple *chunks*, and parse them one at
 * a cycle.
 */
class Base : public SimObject
{
  public:
    /**
     * Forward declaration of compression data. Every new compressor must
     * create a new compression data based on it.
     */
    class CompressionData;

  protected:
    /**
     * A chunk is a basic lexical unit. The data being compressed is received
     * by the compressor as a raw pointer. In order to parse this data, the
     * compressor must divide it into smaller units. Typically, state-of-the-
     * art compressors interpret cache lines as sequential 32-bit chunks
     * (chunks), but any size is valid.
     * @sa chunkSizeBits
     */
    typedef uint64_t Chunk;

    /**
     * This compressor must be able to access the protected functions of
     * its sub-compressors.
     */
    friend class Multi;

    /**
     * Uncompressed cache line size (in bytes).
     */
    const std::size_t blkSize;

    /** Chunk size, in number of bits. */
    const unsigned chunkSizeBits;

    /**
     * Size in bytes at which a compression is classified as bad and therefore
     * the compressed block is restored to its uncompressed format.
     */
    const std::size_t sizeThreshold;

    /**
     * Degree of parallelization of the compression process. It is the
     * number of chunks that can be processed in a cycle.
     */
    const Cycles compChunksPerCycle;

    /**
     * Extra latency added to compression due to packaging, shifting or
     * other operations.
     */
    const Cycles compExtraLatency;

    /**
     * Degree of parallelization of the decompression process. It is the
     * number of chunks that can be processed in a cycle.
     */
    const Cycles decompChunksPerCycle;

    /**
     * Extra latency added to decompression due to packaging, shifting or
     * other operations.
     */
    const Cycles decompExtraLatency;

    /** Pointer to the parent cache. */
    BaseCache* cache;

    struct BaseStats : public statistics::Group
    {
        const Base& compressor;

        BaseStats(Base& compressor);

        void regStats() override;

        /** Number of compressions performed. */
        statistics::Scalar compressions;

        /** Number of failed compressions. */
        statistics::Scalar failedCompressions;

        /** Number of blocks that were compressed to this power of two size. */
        statistics::Vector compressionSize;

        /** Total compressed data size, in number of bits. */
        statistics::Scalar compressionSizeBits;

        /** Average data size after compression, in number of bits. */
        statistics::Formula avgCompressionSizeBits;

        /** Number of decompressions performed. */
        statistics::Scalar decompressions;
    } stats;

    /**
     * This function splits the raw data into chunks, so that it can be
     * parsed by the compressor.
     *
     * @param data The raw pointer to the data being compressed.
     * @return The raw data divided into a vector of sequential chunks.
     */
    std::vector<Chunk> toChunks(const uint64_t* data) const;

    /**
     * This function re-joins the chunks to recreate the original data.
     *
     * @param chunks The raw data divided into a vector of sequential chunks.
     * @param data The raw pointer to the data.
     */
    void fromChunks(const std::vector<Chunk>& chunks, uint64_t* data) const;

    /**
     * Apply the compression process to the cache line.
     * Returns the number of cycles used by the compressor, however it is
     * usually covered by a good pipelined execution, and is currently ignored.
     * The decompression latency is also returned, in order to avoid
     * increasing simulation time and memory consumption.
     *
     * @param chunks The cache line to be compressed, divided into chunks.
     * @param comp_lat Compression latency in number of cycles.
     * @param decomp_lat Decompression latency in number of cycles.
     * @return Cache line after compression.
     */
    virtual std::unique_ptr<CompressionData> compress(
        const std::vector<Chunk>& chunks, Cycles& comp_lat,
        Cycles& decomp_lat) = 0;

    /**
     * Apply the decompression process to the compressed data.
     *
     * @param comp_data Compressed cache line.
     * @param cache_line The cache line to be decompressed.
     */
    virtual void decompress(const CompressionData* comp_data,
                              uint64_t* cache_line) = 0;

  public:
    typedef BaseCacheCompressorParams Params;
    Base(const Params &p);
    virtual ~Base() = default;

    /** The cache can only be set once. */
    virtual void setCache(BaseCache *_cache);

    /**
     * Apply the compression process to the cache line. Ignores compression
     * cycles.
     *
     * @param data The cache line to be compressed.
     * @param comp_lat Compression latency in number of cycles.
     * @param decomp_lat Decompression latency in number of cycles.
     * @return Cache line after compression.
     */
    std::unique_ptr<CompressionData>
    compress(const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat);

    /**
     * Get the decompression latency if the block is compressed. Latency is 0
     * otherwise.
     *
     * @param blk The compressed block.
     */
    Cycles getDecompressionLatency(const CacheBlk* blk);

    /**
     * Set the decompression latency of compressed block.
     *
     * @param blk The compressed block.
     * @param lat The decompression latency.
     */
    static void setDecompressionLatency(CacheBlk* blk, const Cycles lat);

    /**
     * Set the size of the compressed block, in bits.
     *
     * @param blk The compressed block.
     * @param size_bits The block size.
     */
    static void setSizeBits(CacheBlk* blk, const std::size_t size_bits);
};

class Base::CompressionData
{
  private:
    /**
     * Compressed cache line size (in bits).
     */
    std::size_t _size;

  public:
    /**
     * Default constructor.
     */
    CompressionData();

    /**
     * Virtual destructor. Without it unique_ptr will cause mem leak.
     */
    virtual ~CompressionData();

    /**
     * Set compression size (in bits).
     *
     * @param size Compressed data size.
     */
    void setSizeBits(std::size_t size);

    /**
     * Get compression size (in bits).
     *
     * @return Compressed data size.
     */
    std::size_t getSizeBits() const;

    /**
     * Get compression size (in bytes).
     *
     * @return Compressed data size.
     */
    std::size_t getSize() const;
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_BASE_HH__
