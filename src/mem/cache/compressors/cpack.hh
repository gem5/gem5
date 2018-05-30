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
 * Definition of CPack compression, from "C-Pack: A High-Performance
 * Microprocessor Cache Compression Algorithm".
 *
 * The dictionary is composed of 32-bit entries.
 *
 * The patterns are implemented as individual classes that have a checking
 * function isPattern(), to determine if the data fits the pattern, and a
 * decompress() function, which decompresses the contents of a pattern.
 * Every new pattern must inherit from the Pattern class and be added to the
 * patternFactory.
 */

#ifndef __MEM_CACHE_COMPRESSORS_CPACK_HH__
#define __MEM_CACHE_COMPRESSORS_CPACK_HH__

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

struct CPackParams;

class CPack : public BaseCacheCompressor
{
  private:
    /**
     * Compression data for CPack. It consists of a vector of patterns.
     */
    class CompData;

    // Forward declaration of all possible patterns
    class Pattern;
    class PatternZZZZ;
    class PatternXXXX;
    class PatternMMMM;
    class PatternMMXX;
    class PatternZZZX;
    class PatternMMMX;

    /**
     * Create a factory to determine if input matches a pattern. The if else
     * chains are constructed by recursion. The patterns should be explored
     * sorted by size for correct behaviour.
     */
    template <class Head, class... Tail>
    struct Factory
    {
        static std::unique_ptr<Pattern> getPattern(
            const std::array<uint8_t, 4>& bytes,
            const std::array<uint8_t, 4>& dict_bytes, const int match_location)
        {
            // If match this pattern, instantiate it. If a negative match
            // location is used, the patterns that use the dictionary bytes
            // must return false. This is used when there are no dictionary
            // entries yet
            if (Head::isPattern(bytes, dict_bytes, match_location)) {
                return std::unique_ptr<Pattern>(
                            new Head(bytes, match_location));
            // Otherwise, go for next pattern
            } else {
                return Factory<Tail...>::getPattern(bytes, dict_bytes,
                                                    match_location);
            }
        }
    };

    /**
     * Specialization to end the recursion.
     */
    template <class Head>
    struct Factory<Head>
    {
        static std::unique_ptr<Pattern> getPattern(
            const std::array<uint8_t, 4>& bytes,
            const std::array<uint8_t, 4>& dict_bytes, const int match_location)
        {
            // Instantiate last pattern. Should be the XXXX pattern.
            return std::unique_ptr<Pattern>(new Head(bytes, match_location));
        }
    };

    /**
     * Convenience factory declaration. The templates must be organized by
     * size, with the smallest first, and "no-match" last.
     */
    using PatternFactory = Factory<PatternZZZZ, PatternMMMM, PatternZZZX,
                                   PatternMMMX, PatternMMXX, PatternXXXX>;

    /**
     * The dictionary.
     */
    std::vector<std::array<uint8_t, 4>> dictionary;

    /**
     * Dictionary size.
     */
    const std::size_t dictionarySize;

    /**
     * Number of valid entries in the dictionary.
     */
    std::size_t numEntries;

    /**
     * @defgroup CompressionStats Compression specific statistics.
     * @{
     */

    /**
     * Number of data entries that were compressed to each pattern.
     */
    Stats::Vector patternStats;

    /**
     * @}
     */

    /**
     * Compress data.
     *
     * @param data Data to be compressed.
     * @return The pattern this data matches.
     */
    std::unique_ptr<Pattern> compressWord(const uint32_t data);

    /**
     * Decompress a word.
     *
     * @param pattern The pattern to be decompressed.
     * @return The decompressed word.
     */
    uint32_t decompressWord(const Pattern* pattern);

    /**
     * Clear all dictionary entries.
     */
    void resetDictionary();

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

    /**
     * Decompress data.
     *
     * @param comp_data Compressed cache line.
     * @param data The cache line to be decompressed.
     */
    void decompress(const CompressionData* comp_data, uint64_t* data) override;

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

    /**
     * Register local statistics.
     */
    void regStats() override;
};

/**
 * The compressed data is composed of multiple pattern entries. To add a new
 * pattern one should inherit from this class and implement isPattern() and
 * decompress. Then the new pattern must be added to the PatternFactory
 * declaration in crescent order of size (in the CPack class). The pattern
 * must be also added to the Name enum in the CPack::Pattern class before
 * NUM_PATTERNS.
 */
class CPack::Pattern
{
  protected:

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
     * Pattern enum number.
     */
    const PatternNumber patternNumber;

    /**
     * Code associated to the pattern.
     */
    const uint8_t code;

    /**
     * Length, in bits, of the code and match location.
     */
    const uint8_t length;

    /**
     * Number of unmatched bytes;
     */
    const uint8_t numUnmatchedBytes;

    /**
     * Index representing the the match location.
     */
    const int matchLocation;

    /**
     * Wether the pattern allocates a dictionary entry or not.
     */
    const bool allocate;

    /**
     * Get code of this pattern.
     *
     * @return The code.
     */
    uint8_t getCode() const { return code; }

  public:
    /**
     * Default constructor.
     *
     * @param number Pattern number.
     * @param code Code associated to this pattern.
     * @param metadata_length Length, in bits, of the code and match location.
     * @param num_unmatched_bytes Number of unmatched bytes.
     * @param match_location Index of the match location.
     */
    Pattern(const PatternNumber number, const uint64_t code,
            const uint64_t metadata_length, const uint64_t num_unmatched_bytes,
            const int match_location, const bool allocate = true)
        : patternNumber(number), code(code), length(metadata_length),
          numUnmatchedBytes(num_unmatched_bytes),
          matchLocation(match_location), allocate(allocate) {};

    /**
     * Default destructor.
     */
    virtual ~Pattern() = default;

    /**
     * Trick function to get the number of patterns.
     *
     * @return The number of defined patterns.
     */
    static uint64_t getNumPatterns() { return NUM_PATTERNS; };

    /**
     * Get enum number associated to this pattern.
     *
     * @return The pattern enum number.
     */
    PatternNumber getPatternNumber() const { return patternNumber; };

    /**
     * Get meta-name assigned to the given pattern.
     *
     * @param number The number of the pattern.
     * @return The meta-name of the pattern.
     */
    static std::string getName(int number)
    {
        static std::map<PatternNumber, std::string> patternNames = {
            {ZZZZ, "ZZZZ"}, {XXXX, "XXXX"}, {MMMM, "MMMM"},
            {MMXX, "MMXX"}, {ZZZX, "ZZZX"}, {MMMX, "MMMX"}
        };

        return patternNames[(PatternNumber)number];
    };

    /**
     * Get the index of the dictionary match location.
     *
     * @return The index of the match location.
     */
    uint8_t getMatchLocation() const { return matchLocation; }

    /**
     * Get size, in bits, of the pattern (excluding prefix). Corresponds to
     * unmatched_data_size + code_length.
     *
     * @return The size.
     */
    std::size_t getSizeBits() const {
        return numUnmatchedBytes*CHAR_BIT + length;
    }

    /**
     * Determine if pattern allocates a dictionary entry.
     *
     * @return True if should allocate a dictionary entry.
     */
    bool shouldAllocate() const {
        return allocate;
    }

    std::string print() const {
        return csprintf("pattern %s (encoding %x, size %u bits)",
                        getName(patternNumber), getCode(), getSizeBits());
    }

    /**
     * Decompress the pattern. Each pattern has its own way of interpreting
     * its data.
     *
     * @param dict_bytes The bytes in the corresponding matching entry.
     * @param data The decompressed pattern.
     * @return Whether entry should be added to dictionary or not.
     */
    virtual bool decompress(const std::array<uint8_t, 4> dict_bytes,
                            std::array<uint8_t, 4>& data) const = 0;
};

class CPack::PatternZZZZ : public Pattern
{
  public:
    PatternZZZZ(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(ZZZZ, 0x0, 2, 0, 0, false) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        return (bytes[3] == 0) && (bytes[2] == 0) && (bytes[1] == 0) &&
               (bytes[0] == 0);
    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = {0, 0, 0, 0};
        return false;
    }
};

class CPack::PatternXXXX : public Pattern
{
  private:
    /**
     * A copy of the word.
     */
    const std::array<uint8_t, 4> bytes;

  public:
    PatternXXXX(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(XXXX, 0x1, 2, 4, 0, true), bytes(bytes) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        // It can always be an unmatch, as it is set to this class when other
        // patterns fail
        return true;
    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = bytes;
        return true;
    }
};

class CPack::PatternMMMM : public Pattern
{
  public:
    PatternMMMM(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(MMMM, 0x2, 6, 0, match_location, true) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        return (bytes == dict_bytes) && (match_location >= 0);
    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = dict_bytes;
        return true;
    }
};

class CPack::PatternMMXX : public Pattern
{
  private:
    /**
     * A copy of the unmatched bytes.
     */
    const uint8_t byte0;
    const uint8_t byte1;

  public:
    PatternMMXX(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(MMXX, 0xC, 8, 2, match_location, true),
                  byte0(bytes[0]), byte1(bytes[1]) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        // Notice we don't compare bytes[0], as otherwise we'd be unnecessarily
        // discarding MMXM. If that pattern is added this should be modified
        return (bytes[3] == dict_bytes[3]) && (bytes[2] == dict_bytes[2]) &&
               (bytes[1] != dict_bytes[1]) && (match_location >= 0);

    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = {byte0, byte1, dict_bytes[2], dict_bytes[3]};
        return true;
    }
};

class CPack::PatternZZZX : public Pattern
{
  private:
    /**
     * A copy of the unmatched byte.
     */
    const uint8_t byte;

  public:
    PatternZZZX(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(ZZZX, 0xD, 4, 1, 0, false), byte(bytes[0]) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        return (bytes[3] == 0) && (bytes[2] == 0) && (bytes[1] == 0) &&
               (bytes[0] != 0);
    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = {byte, 0, 0, 0};
        return false;
    }
};

class CPack::PatternMMMX : public Pattern
{
  private:
    /**
     * A copy of the unmatched byte.
     */
    const uint8_t byte;

  public:
    PatternMMMX(const std::array<uint8_t, 4> bytes, const int match_location)
        : Pattern(MMMX, 0xE, 8, 1, match_location, true),
                  byte(bytes[0]) {}

    static bool isPattern(const std::array<uint8_t, 4>& bytes,
                          const std::array<uint8_t, 4>& dict_bytes,
                          const int match_location)
    {
        return (bytes[3] == dict_bytes[3]) && (bytes[2] == dict_bytes[2]) &&
               (bytes[1] == dict_bytes[1]) && (bytes[0] != dict_bytes[0]) &&
               (match_location >= 0);
    }

    bool decompress(const std::array<uint8_t, 4> dict_bytes,
                    std::array<uint8_t, 4>& data) const override
    {
        data = {byte, dict_bytes[1], dict_bytes[2], dict_bytes[3]};
        return true;
    }
};

class CPack::CompData : public CompressionData
{
  public:
    /**
     * The patterns matched in the original line.
     */
    std::vector<std::unique_ptr<Pattern>> entries;

    /**
     * Default constructor.
     *
     * @param dictionary_size Number of entries in the dictionary.
     */
    CompData(const std::size_t dictionary_size);

    /**
     * Default destructor.
     */
    ~CompData();
};

#endif //__MEM_CACHE_COMPRESSORS_CPACK_HH__
