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
 * Definition of CPack compression, from "C-Pack: A High-Performance
 * Microprocessor Cache Compression Algorithm".
 */

#ifndef __MEM_CACHE_COMPRESSORS_CPACK_HH__
#define __MEM_CACHE_COMPRESSORS_CPACK_HH__

#include <cstdint>
#include <map>
#include <memory>

#include "base/types.hh"
#include "mem/cache/compressors/dictionary_compressor.hh"

struct CPackParams;

namespace Compressor {

class CPack : public DictionaryCompressor<uint32_t>
{
  private:
    using DictionaryEntry = DictionaryCompressor<uint32_t>::DictionaryEntry;

    // Forward declaration of all possible patterns
    class PatternZZZZ;
    class PatternXXXX;
    class PatternMMMM;
    class PatternMMXX;
    class PatternZZZX;
    class PatternMMMX;

    /**
     * The patterns proposed in the paper. Each letter represents a byte:
     * Z is a null byte, M is a dictionary match, X is a new value.
     * These are used as indexes to reference the pattern data. If a new
     * pattern is added, it must be done before NUM_PATTERNS.
     */
    typedef enum {
        ZZZZ, XXXX, MMMM, MMXX, ZZZX, MMMX, NUM_PATTERNS
    } PatternNumber;

    /**
     * Convenience factory declaration. The templates must be organized by
     * size, with the smallest first, and "no-match" last.
     */
    using PatternFactory = Factory<PatternZZZZ, PatternMMMM, PatternZZZX,
                                   PatternMMMX, PatternMMXX, PatternXXXX>;

    uint64_t getNumPatterns() const override { return NUM_PATTERNS; }

    std::string
    getName(int number) const override
    {
        static std::map<int, std::string> patternNames = {
            {ZZZZ, "ZZZZ"}, {XXXX, "XXXX"}, {MMMM, "MMMM"},
            {MMXX, "MMXX"}, {ZZZX, "ZZZX"}, {MMMX, "MMMX"}
        };

        return patternNames[number];
    };

    std::unique_ptr<Pattern> getPattern(
        const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location) const override
    {
        return PatternFactory::getPattern(bytes, dict_bytes, match_location);
    }

    void addToDictionary(DictionaryEntry data) override;

    std::unique_ptr<Base::CompressionData> compress(
        const std::vector<Base::Chunk>& chunks,
        Cycles& comp_lat, Cycles& decomp_lat) override;

  public:
    /** Convenience typedef. */
     typedef CPackParams Params;

    /**
     * Default constructor.
     */
    CPack(const Params *p);

    /**
     * Default destructor.
     */
    ~CPack() {};
};

class CPack::PatternZZZZ : public MaskedValuePattern<0, 0xFFFFFFFF>
{
  public:
    PatternZZZZ(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFFFFFF>(ZZZZ, 0x0, 2, match_location,
          bytes)
    {
    }
};

class CPack::PatternXXXX : public UncompressedPattern
{
  public:
    PatternXXXX(const DictionaryEntry bytes, const int match_location)
        : UncompressedPattern(XXXX, 0x1, 2, match_location, bytes)
    {
    }
};

class CPack::PatternMMMM : public MaskedPattern<0xFFFFFFFF>
{
  public:
    PatternMMMM(const DictionaryEntry bytes, const int match_location)
        : MaskedPattern<0xFFFFFFFF>(MMMM, 0x2, 6, match_location, bytes, true)
    {
    }
};

class CPack::PatternMMXX : public MaskedPattern<0xFFFF0000>
{
  public:
    PatternMMXX(const DictionaryEntry bytes, const int match_location)
        : MaskedPattern<0xFFFF0000>(MMXX, 0xC, 8, match_location, bytes, true)
    {
    }
};

class CPack::PatternZZZX : public MaskedValuePattern<0, 0xFFFFFF00>
{
  public:
    PatternZZZX(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFFFF00>(ZZZX, 0xD, 4, match_location,
          bytes)
    {
    }
};

class CPack::PatternMMMX : public MaskedPattern<0xFFFFFF00>
{
  public:
    PatternMMMX(const DictionaryEntry bytes, const int match_location)
        : MaskedPattern<0xFFFFFF00>(MMMX, 0xE, 8, match_location, bytes, true)
    {
    }
};

} // namespace Compressor

#endif //__MEM_CACHE_COMPRESSORS_CPACK_HH__
