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
 * Definition of a repeated qwords compressor, which compresses data if
 * it is entirely composed of repeated qwords.
 */

#ifndef __MEM_CACHE_COMPRESSORS_REPEATED_QWORDS_HH__
#define __MEM_CACHE_COMPRESSORS_REPEATED_QWORDS_HH__

#include <array>
#include <cstdint>
#include <map>
#include <memory>

#include "mem/cache/compressors/dictionary_compressor.hh"

namespace gem5
{

struct RepeatedQwordsCompressorParams;

namespace compression
{

class RepeatedQwords : public DictionaryCompressor<uint64_t>
{
  protected:
    using DictionaryEntry = DictionaryCompressor<uint64_t>::DictionaryEntry;

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
        X, M, NUM_PATTERNS
    };

    /**
     * Convenience factory declaration. The templates must be organized by
     * size, with the smallest first, and "no-match" last.
     */
    using PatternFactory = Factory<PatternM, PatternX>;

    uint64_t getNumPatterns() const override { return NUM_PATTERNS; }

    std::string
    getName(int number) const override
    {
        static std::map<int, std::string> pattern_names = {
            {X, "X"}, {M, "M"}
        };

        return pattern_names[number];
    };

    std::unique_ptr<Pattern>
    getPattern(const DictionaryEntry& bytes, const DictionaryEntry& dict_bytes,
        const int match_location) const override
    {
        return PatternFactory::getPattern(bytes, dict_bytes, match_location);
    }

    void addToDictionary(DictionaryEntry data) override;

    std::unique_ptr<Base::CompressionData> compress(
        const std::vector<Base::Chunk>& chunks,
        Cycles& comp_lat, Cycles& decomp_lat) override;

  public:
    typedef RepeatedQwordsCompressorParams Params;
    RepeatedQwords(const Params &p);
    ~RepeatedQwords() = default;
};

class RepeatedQwords::PatternX
    : public DictionaryCompressor::UncompressedPattern
{
  public:
    PatternX(const DictionaryEntry bytes, const int match_location)
        : UncompressedPattern(X, 0, 0, match_location, bytes)
    {
    }
};

class RepeatedQwords::PatternM
    : public DictionaryCompressor::LocatedMaskedPattern<0xFFFFFFFFFFFFFFFF, 0>
{
  public:
    PatternM(const DictionaryEntry bytes, const int match_location)
        : LocatedMaskedPattern<0xFFFFFFFFFFFFFFFF, 0>(M, 1, 0, match_location,
          bytes, false)
    {
    }
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_REPEATED_QWORDS_HH__
