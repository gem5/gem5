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
 * Definition of a dictionary based cache compressor. Each entry is compared
 * against a dictionary to search for matches.
 *
 * The dictionary is composed of 32-bit entries, and the comparison is done
 * byte per byte.
 *
 * The patterns are implemented as individual classes that have a checking
 * function isPattern(), to determine if the data fits the pattern, and a
 * decompress() function, which decompresses the contents of a pattern.
 * Every new pattern must inherit from the Pattern class and be added to the
 * patternFactory.
 */

#ifndef __MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_HH__
#define __MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_HH__

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

struct DictionaryCompressorParams;

class DictionaryCompressor : public BaseCacheCompressor
{
  protected:
    /**
     * Compression data for the dictionary compressor. It consists of a vector
     * of patterns.
     */
    class CompData;

    // Forward declaration of a pattern
    class Pattern;

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

    /** Specialization to end the recursion. */
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

    /** The dictionary. */
    std::vector<std::array<uint8_t, 4>> dictionary;

    /** Dictionary size. */
    const std::size_t dictionarySize;

    /** Number of valid entries in the dictionary. */
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
     * Trick function to get the number of patterns.
     *
     * @return The number of defined patterns.
     */
    virtual uint64_t getNumPatterns() const = 0;

    /**
     * Get meta-name assigned to the given pattern.
     *
     * @param number The number of the pattern.
     * @return The meta-name of the pattern.
     */
    virtual std::string getName(int number) const = 0;

    /**
     * Since the factory cannot be instantiated here, classes that inherit
     * from this base class have to implement the call to their factory's
     * getPattern.
     */
    virtual std::unique_ptr<Pattern> getPattern(
        const std::array<uint8_t, 4>& bytes,
        const std::array<uint8_t, 4>& dict_bytes,
        const int match_location) const = 0;

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

    /** Clear all dictionary entries. */
    void resetDictionary();

    /**
     * Add an entry to the dictionary.
     *
     * @param data The new entry.
     */
    virtual void addToDictionary(std::array<uint8_t, 4> data) = 0;

    /**
     * Apply compression.
     *
     * @param data The cache line to be compressed.
     * @return Cache line after compression.
     */
    std::unique_ptr<BaseCacheCompressor::CompressionData> compress(
        const uint64_t* data);

    /**
     * Decompress data.
     *
     * @param comp_data Compressed cache line.
     * @param data The cache line to be decompressed.
     */
    void decompress(const CompressionData* comp_data, uint64_t* data) override;

  public:
    /** Convenience typedef. */
    typedef DictionaryCompressorParams Params;

    /** Default constructor. */
    DictionaryCompressor(const Params *p);

    /** Default destructor. */
    ~DictionaryCompressor() {};

    /** Register local statistics. */
    void regStats() override;
};

/**
 * The compressed data is composed of multiple pattern entries. To add a new
 * pattern one should inherit from this class and implement isPattern() and
 * decompress(). Then the new pattern must be added to the PatternFactory
 * declaration in crescent order of size (in the DictionaryCompressor class).
 */
class DictionaryCompressor::Pattern
{
  protected:
    /** Pattern enum number. */
    const int patternNumber;

    /** Code associated to the pattern. */
    const uint8_t code;

    /** Length, in bits, of the code and match location. */
    const uint8_t length;

    /** Number of unmatched bytes. */
    const uint8_t numUnmatchedBytes;

    /** Index representing the the match location. */
    const int matchLocation;

    /** Wether the pattern allocates a dictionary entry or not. */
    const bool allocate;

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
    Pattern(const int number, const uint64_t code,
            const uint64_t metadata_length, const uint64_t num_unmatched_bytes,
            const int match_location, const bool allocate = true)
        : patternNumber(number), code(code), length(metadata_length),
          numUnmatchedBytes(num_unmatched_bytes),
          matchLocation(match_location), allocate(allocate)
    {
    }

    /** Default destructor. */
    virtual ~Pattern() = default;

    /**
     * Get enum number associated to this pattern.
     *
     * @return The pattern enum number.
     */
    int getPatternNumber() const { return patternNumber; };

    /**
     * Get code of this pattern.
     *
     * @return The code.
     */
    uint8_t getCode() const { return code; }

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
    std::size_t
    getSizeBits() const
    {
        return numUnmatchedBytes*CHAR_BIT + length;
    }

    /**
     * Determine if pattern allocates a dictionary entry.
     *
     * @return True if should allocate a dictionary entry.
     */
    bool shouldAllocate() const { return allocate; }

    /**
     * Extract pattern's information to a string.
     *
     * @return A string containing the relevant pattern metadata.
     */
    std::string
    print() const
    {
        return csprintf("pattern %s (encoding %x, size %u bits)",
                        getPatternNumber(), getCode(), getSizeBits());
    }

    /**
     * Decompress the pattern. Each pattern has its own way of interpreting
     * its data.
     *
     * @param dict_bytes The bytes in the corresponding matching entry.
     * @return The decompressed pattern.
     */
    virtual std::array<uint8_t, 4> decompress(
        const std::array<uint8_t, 4> dict_bytes) const = 0;
};

class DictionaryCompressor::CompData : public CompressionData
{
  public:
    /** The patterns matched in the original line. */
    std::vector<std::unique_ptr<Pattern>> entries;

    /**
     * Default constructor.
     *
     * @param dictionary_size Number of entries in the dictionary.
     */
    CompData(const std::size_t dictionary_size);

    /** Default destructor. */
    ~CompData();
};

#endif //__MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_HH__
