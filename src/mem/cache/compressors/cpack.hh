/*
 * Copyright (c) 2018-2019 Inria
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

    /**
     * Apply compression.
     *
     * @param data The cache line to be compressed.
     * @param comp_lat Compression latency in number of cycles.
     * @param decomp_lat Decompression latency in number of cycles.
     * @return Cache line after compression.
     */
    std::unique_ptr<BaseCacheCompressor::CompressionData> compress(
        const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat) override;

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

class CPack::PatternZZZZ : public DictionaryCompressor::Pattern
{
  public:
    PatternZZZZ(const DictionaryEntry bytes, const int match_location)
        : Pattern(ZZZZ, 0x0, 2, 0, 0, false) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        return (bytes[3] == 0) && (bytes[2] == 0) && (bytes[1] == 0) &&
               (bytes[0] == 0);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return {0, 0, 0, 0};
    }
};

class CPack::PatternXXXX : public DictionaryCompressor::Pattern
{
  private:
    /**
     * A copy of the word.
     */
    const DictionaryEntry bytes;

  public:
    PatternXXXX(const DictionaryEntry bytes, const int match_location)
        : Pattern(XXXX, 0x1, 2, 4, 0, true), bytes(bytes) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        // It can always be an unmatch, as it is set to this class when other
        // patterns fail
        return true;
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return bytes;
    }
};

class CPack::PatternMMMM : public DictionaryCompressor::Pattern
{
  public:
    PatternMMMM(const DictionaryEntry bytes, const int match_location)
        : Pattern(MMMM, 0x2, 6, 0, match_location, true) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        return (bytes == dict_bytes) && (match_location >= 0);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return dict_bytes;
    }
};

class CPack::PatternMMXX : public DictionaryCompressor::Pattern
{
  private:
    /**
     * A copy of the unmatched bytes.
     */
    const uint8_t byte0;
    const uint8_t byte1;

  public:
    PatternMMXX(const DictionaryEntry bytes, const int match_location)
        : Pattern(MMXX, 0xC, 8, 2, match_location, true),
                  byte0(bytes[0]), byte1(bytes[1]) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        // Notice we don't compare bytes[0], as otherwise we'd be unnecessarily
        // discarding MMXM. If that pattern is added this should be modified
        return (bytes[3] == dict_bytes[3]) && (bytes[2] == dict_bytes[2]) &&
               (bytes[1] != dict_bytes[1]) && (match_location >= 0);

    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return {byte0, byte1, dict_bytes[2], dict_bytes[3]};
    }
};

class CPack::PatternZZZX : public DictionaryCompressor::Pattern
{
  private:
    /**
     * A copy of the unmatched byte.
     */
    const uint8_t byte;

  public:
    PatternZZZX(const DictionaryEntry bytes, const int match_location)
        : Pattern(ZZZX, 0xD, 4, 1, 0, false), byte(bytes[0]) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        return (bytes[3] == 0) && (bytes[2] == 0) && (bytes[1] == 0) &&
               (bytes[0] != 0);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return {byte, 0, 0, 0};
    }
};

class CPack::PatternMMMX : public DictionaryCompressor::Pattern
{
  private:
    /**
     * A copy of the unmatched byte.
     */
    const uint8_t byte;

  public:
    PatternMMMX(const DictionaryEntry bytes, const int match_location)
        : Pattern(MMMX, 0xE, 8, 1, match_location, true),
                  byte(bytes[0]) {}

    static bool isPattern(const DictionaryEntry& bytes,
        const DictionaryEntry& dict_bytes,
        const int match_location)
    {
        return (bytes[3] == dict_bytes[3]) && (bytes[2] == dict_bytes[2]) &&
               (bytes[1] == dict_bytes[1]) && (bytes[0] != dict_bytes[0]) &&
               (match_location >= 0);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return {byte, dict_bytes[1], dict_bytes[2], dict_bytes[3]};
    }
};

#endif //__MEM_CACHE_COMPRESSORS_CPACK_HH__
