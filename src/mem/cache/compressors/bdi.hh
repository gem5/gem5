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
 * Definition of "Base-Delta-Immediate Compression: Practical Data Compression
 * for On-Chip Caches".
 */

#ifndef __MEM_CACHE_COMPRESSORS_BDI_HH__
#define __MEM_CACHE_COMPRESSORS_BDI_HH__

#include <cstdint>
#include <memory>
#include <vector>

#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

struct BDIParams;

/**
 * Default maximum number of bases in the original BDI.
 */
#define BDI_DEFAULT_MAX_NUM_BASES 2

class BDI : public BaseCacheCompressor
{
  protected:
    /**
     * Forward declaration of comp data classes.
     */
    class BDICompData;
    class BDICompDataZeros;
    class BDICompDataRep;
    class BDICompDataUncompressed;
    template <class TB, class TD> class BDICompDataBaseDelta;

    /**
     * The possible encoding values. If modified, ENCODING_NAMES must be too.
     */
    enum ENCODING {ZERO, REP_VALUES, BASE8_1, BASE8_2, BASE8_4, BASE4_1,
                   BASE4_2, BASE2_1, UNCOMPRESSED, NUM_ENCODINGS};

    /**
     * The respective encoding names. They are indexed by the ENCODING enum.
     * The values are assigned in the source file, and should be modified if
     * ENCODING is changed.
     */
    static const char* ENCODING_NAMES[];

    /**
     * If set, create multiple compressor instances for each possible
     * combination of base and delta size. Otherwise, just create a
     * compressor for each base size with the highest available delta
     * size. This can be used to save area and power (having less
     * compressors). True by default.
     */
    const bool useMoreCompressors;

    /**
     * Number of qwords in a cache line.
     */
    const std::size_t qwordsPerCacheLine;

    /**
     * @defgroup CompressionStats Compression specific statistics.
     * @{
     */

    /**
     * Number of data entries that were compressed to each encoding.
     */
    Stats::Vector encodingStats;

    /**
     * @}
     */

    /**
     * Check if the cache line consists of only zero values.
     *
     * @param data The cache line.
     * @return True if it is a ZERO cache line.
     */
    bool isZeroPackable(const uint64_t* data) const;

    /**
     * Check if the cache line consists only of same values.
     *
     * @param data The cache line.
     * @return True if it is a REP_VALUES cache line.
     */
    bool isSameValuePackable(const uint64_t* data) const;

    /**
     * Instantiate a BaseDelta compressor with given TB and TD, and try to
     * compress the cache line. If the compression fails, it returns a nullptr.
     * @sa BDICompDataBaseDelta
     *
     * @tparam TB Type of a base entry.
     * @tparam TD Type of a delta entry.
     * @param data The cache line to be compressed.
     * @param encoding Encoding value for given TB-TD combination.
     * @return Cache line after compression or nullptr.
     */
    template <class TB, class TD>
    std::unique_ptr<BDICompData> tryCompress(const uint64_t* data,
                                             const uint8_t encoding) const;

    /**
     * Apply compression.
     *
     * @param data The cache line to be compressed.
     * @param comp_lat Compression latency in number of cycles.
     * @param decomp_lat Decompression latency in number of cycles.
     * @param comp_size Compressed data size.
     */
    std::unique_ptr<BaseCacheCompressor::CompressionData> compress(
        const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat) override;

    /**
     * Decompress data.
     *
     * @param comp_data Compressed cache line.
     * @param data The cache line to be decompressed.
     * @return Decompression latency in number of cycles.
     */
    void decompress(const BaseCacheCompressor::CompressionData* comp_data,
                                           uint64_t* data) override;

  public:
    /** Convenience typedef. */
    typedef BDIParams Params;

    /**
     * Default constructor.
     */
    BDI(const Params *p);

    /**
     * Default destructor.
     */
    ~BDI() = default;

    /**
     * Register local statistics.
     */
    void regStats() override;
};

/**
 * Template for the BDI compression data.
 */
class BDI::BDICompData : public CompressionData
{
  private:
    /**
     * Data encoding.
     * @sa BDI
     */
    const uint8_t _encoding;

  protected:
    /**
     * Number of bits needed for the encoding field.
     */
    static const std::size_t encodingBits = 4;

    /**
     * Calculate and set compressed data size.
     * Each BDI compressor generates compressed data with different sizes.
    */
    virtual void calculateCompressedSize() = 0;

  public:
    /**
     * Default constructor.
     *
     * @param encoding The encoding value.
     */
    BDICompData(const uint8_t encoding);

    /**
     * Default destructor.
     */
    virtual ~BDICompData() = default;

    /**
     * Get and decompress data at given index.
     *
     * The index is given relative to 64-bit entries, therefore if the base
     * size of the given compressed data is smaller than that, this function
     * joins multiple base-delta entries to generate the respective 64-bit
     * entry.
     *
     * @param index The index of the compressed data.
     * @return Decompressed data for the given index.
     */
    virtual uint64_t access(const int index) const = 0;

    /**
     * Get encoding.
     *
     * @return The encoding.
     */
    uint8_t getEncoding() const;

    /**
     * Get encoding name.
     *
     * @return The encoding name.
     */
    std::string getName() const;
};

/**
 * BDI compressed data containing the ZERO encoding.
 */
class BDI::BDICompDataZeros : public BDICompData
{
  protected:
    /**
     * Calculate compressed data size using ZERO encoding.
     */
    void calculateCompressedSize() override;

  public:
    /**
     * Default constructor.
    */
    BDICompDataZeros();

    /**
     * Get and decompress data at given index. Must always return 0.
     *
     * @param index The index of the compressed data.
     * @return Decompressed data for the given index.
     */
    uint64_t access(const int index) const override;
};

/**
 * BDI compressed data containing the REP_VALUES encoding.
 */
class BDI::BDICompDataRep : public BDICompData
{
  private:
    /**
     * The repeated value.
     */
    uint64_t base;

  protected:
    /**
     * Calculate compressed data size using REP_VALUES encoding.
     */
    void calculateCompressedSize() override;

  public:
    /**
     * Default constructor.
     *
     * @param rep_value The repeated value.
     */
    BDICompDataRep(const uint64_t rep_value);

    /**
     * Get and decompress data at given index. Must always return the same
     * value as data[0].
     *
     * @param index The index of the compressed data.
     * @return Decompressed data for the given index.
     */
    uint64_t access(const int index) const override;
};

/**
 * BDI compressed data containing the UNCOMPRESSED encoding.
 */
class BDI::BDICompDataUncompressed : public BDICompData
{
  private:
    /**
     * Uncompressed cache line size (in bytes).
     */
    const std::size_t blkSize;

    /**
     * The compressed data is the original cache line.
     */
    const std::vector<uint64_t> _data;

  protected:
    /**
     * Calculate compressed data size using UNCOMPRESSED encoding.
     */
    void calculateCompressedSize() override;

  public:
    /**
     * Default constructor.
     *
     * @param data The data on which compression was applied.
     * @param blk_size Size of a cache line in bytes.
     */
    BDICompDataUncompressed(const uint64_t* data,
                            const std::size_t blk_size);

    /**
     * Get and decompress data at given index. Must return the same
     * value as _data[index].
     *
     * @param index The index of the compressed data.
     * @return Decompressed data for the given index.
     */
    uint64_t access(const int index) const override;
};

/**
 * Template class for BDI compressed data containing all the BASE_DELTA
 * encodings. TB's size must always be greater than TD's.
 *
 * @tparam TB Type of a base entry.
 * @tparam TD Type of a delta entry.
*/
template <class TB, class TD>
class BDI::BDICompDataBaseDelta : public BDICompData
{
  protected:
    /**
     * Maximum number of bases.
     */
    const std::size_t maxNumBases;

    /**
     * Bit mask to differentiate between the bases.
     */
    std::vector<uint8_t> bitMask;

    /**
     * Bases. bases[0] is 0 and is not stored in a hardware implementation.
     */
    std::vector<TB> bases;

    /**
     * Array of deltas (or immediate values).
     */
    std::vector<TD> deltas;

    /**
     * Add a base to the bases vector.
     *
     * @param base The base to be added.
     * @return True on success, false if already used all base slots.
     */
    bool addBase(const TB base);

    /**
     * Add a delta to the deltas vector.
     *
     * @param base_index Base to which the delta refers.
     * @param delta The delta value.
     */
    void addDelta(const std::size_t base_index, const TD delta);

    /**
     * Calculate compressed data size using number of bases, the base size and
     * the delta size.
     */
    void calculateCompressedSize() override;

  public:
    /**
     * Default constructor.
     *
     * @param encoding The encoding value for this compressor.
     * @param blk_size Size of a cache line in bytes.
     * @param max_num_bases Maximum number of bases allowed to be stored.
     */
    BDICompDataBaseDelta(const uint8_t encoding, const std::size_t blk_size,
        const std::size_t max_num_bases = BDI_DEFAULT_MAX_NUM_BASES);

    /**
     * Get and decompress data at given index.
     *
     * @param index The index of the compressed data.
     * @return Decompressed data for the given index.
     */
    uint64_t access(const int index) const override;

    /**
     * Apply base delta compression.
     *
     * @param data The data on which compression was applied.
     * @param blk_size Size of a cache line in bytes.
     * @return True on success.
     */
    bool compress(const uint64_t* data, const std::size_t blk_size);
};

#endif //__MEM_CACHE_COMPRESSORS_BDI_HH__
