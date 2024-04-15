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
 * Definition of a base delta immediate compressor. @see BDI
 */

#ifndef __MEM_CACHE_COMPRESSORS_BASE_DELTA_HH__
#define __MEM_CACHE_COMPRESSORS_BASE_DELTA_HH__

#include <array>
#include <cstdint>
#include <map>
#include <memory>

#include "base/bitfield.hh"
#include "mem/cache/compressors/dictionary_compressor.hh"

namespace gem5
{

struct BaseDictionaryCompressorParams;
struct Base64Delta8Params;
struct Base64Delta16Params;
struct Base64Delta32Params;
struct Base32Delta8Params;
struct Base32Delta16Params;
struct Base16Delta8Params;

namespace compression
{

/**
 * Base class for all base-delta-immediate compressors. Although not proposed
 * like this in the original paper, the sub-compressors of BDI are dictionary
 * based with 2 possible patterns: no match, where a dictionary entry must be
 * allocated, and masked delta match with a dictionary entry, where the delta
 * must be stored instead. The maximum number of dictionary entries is 2, and
 * one of them is reserved for a zero base if using immediate compression.
 *
 * @tparam BaseType Type of a base (dictionary) entry.
 */
template <class BaseType, std::size_t DeltaSizeBits>
class BaseDelta : public DictionaryCompressor<BaseType>
{
  protected:
    static constexpr int DEFAULT_MAX_NUM_BASES = 2;

    using DictionaryEntry =
        typename DictionaryCompressor<BaseType>::DictionaryEntry;

    // Forward declaration of all possible patterns
    class PatternX;
    class PatternM;

    /**
     * The patterns proposed in the paper. Each letter represents a byte:
     * Z is a null byte, M is a dictionary match, X is a new value.
     * These are used as indexes to reference the pattern data. If a new
     * pattern is added, it must be done before NUM_PATTERNS.
     */
    enum PatternNumber
    {
        X,
        M,
        NUM_PATTERNS
    };

    uint64_t
    getNumPatterns() const override
    {
        return NUM_PATTERNS;
    }

    /**
     * Convenience factory declaration. The templates must be organized by
     * size, with the smallest first, and "no-match" last.
     */
    using PatternFactory =
        typename DictionaryCompressor<BaseType>::template Factory<PatternM,
                                                                  PatternX>;

    std::unique_ptr<typename DictionaryCompressor<BaseType>::Pattern>
    getPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
               const int match_location) const override
    {
        return PatternFactory::getPattern(bytes, dict_bytes, match_location);
    }

    std::string
    getName(int number) const override
    {
        static std::map<int, std::string> pattern_names = { { X, "X" },
                                                            { M, "M" } };

        return pattern_names[number];
    }

    void resetDictionary() override;

    void addToDictionary(DictionaryEntry data) override;

    std::unique_ptr<Base::CompressionData>
    compress(const std::vector<Base::Chunk> &chunks, Cycles &comp_lat,
             Cycles &decomp_lat) override;

  public:
    typedef BaseDictionaryCompressorParams Params;
    BaseDelta(const Params &p);
    ~BaseDelta() = default;
};

template <class BaseType, std::size_t DeltaSizeBits>
class BaseDelta<BaseType, DeltaSizeBits>::PatternX :
    public DictionaryCompressor<BaseType>::UncompressedPattern
{
  public:
    // A delta entry containing the value 0 is added even if it is an entirely
    // new base
    PatternX(const DictionaryEntry bytes, const int match_location)
        : DictionaryCompressor<BaseType>::UncompressedPattern(
              X, 0,
              std::ceil(std::log2(DEFAULT_MAX_NUM_BASES)) + DeltaSizeBits,
              match_location, bytes)
    {}
};

template <class BaseType, std::size_t DeltaSizeBits>
class BaseDelta<BaseType, DeltaSizeBits>::PatternM :
    public DictionaryCompressor<BaseType>::template DeltaPattern<DeltaSizeBits>
{
  public:
    // The number of bits reserved for the bitmask entry is proportional to
    // the maximum number of bases
    PatternM(const DictionaryEntry bytes, const int match_location)
        : DictionaryCompressor<BaseType>::template DeltaPattern<DeltaSizeBits>(
              M, 1, std::ceil(std::log2(DEFAULT_MAX_NUM_BASES)),
              match_location, bytes)
    {}
};

class Base64Delta8 : public BaseDelta<uint64_t, 8>
{
  public:
    typedef Base64Delta8Params Params;
    Base64Delta8(const Params &p);
    ~Base64Delta8() = default;
};

class Base64Delta16 : public BaseDelta<uint64_t, 16>
{
  public:
    typedef Base64Delta16Params Params;
    Base64Delta16(const Params &p);
    ~Base64Delta16() = default;
};

class Base64Delta32 : public BaseDelta<uint64_t, 32>
{
  public:
    typedef Base64Delta32Params Params;
    Base64Delta32(const Params &p);
    ~Base64Delta32() = default;
};

class Base32Delta8 : public BaseDelta<uint32_t, 8>
{
  public:
    typedef Base32Delta8Params Params;
    Base32Delta8(const Params &p);
    ~Base32Delta8() = default;
};

class Base32Delta16 : public BaseDelta<uint32_t, 16>
{
  public:
    typedef Base32Delta16Params Params;
    Base32Delta16(const Params &p);
    ~Base32Delta16() = default;
};

class Base16Delta8 : public BaseDelta<uint16_t, 8>
{
  public:
    typedef Base16Delta8Params Params;
    Base16Delta8(const Params &p);
    ~Base16Delta8() = default;
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_BASE_DELTA_HH__
