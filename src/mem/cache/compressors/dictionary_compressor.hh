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
#include <type_traits>
#include <vector>

#include "base/bitfield.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/compressors/base.hh"

namespace gem5
{

struct BaseDictionaryCompressorParams;

namespace compression
{

class BaseDictionaryCompressor : public Base
{
  protected:
    /** Dictionary size. */
    const std::size_t dictionarySize;

    /** Number of valid entries in the dictionary. */
    std::size_t numEntries;

    struct DictionaryStats : public statistics::Group
    {
        const BaseDictionaryCompressor &compressor;

        DictionaryStats(BaseStats &base_group,
                        BaseDictionaryCompressor &_compressor);

        void regStats() override;

        /** Number of data entries that were compressed to each pattern. */
        statistics::Vector patterns;
    } dictionaryStats;

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

  public:
    typedef BaseDictionaryCompressorParams Params;
    BaseDictionaryCompressor(const Params &p);
    ~BaseDictionaryCompressor() = default;
};

/**
 * A template version of the dictionary compressor that allows to choose the
 * dictionary size.
 *
 * @tparam The type of a dictionary entry (e.g., uint16_t, uint32_t, etc).
 */
template <class T>
class DictionaryCompressor : public BaseDictionaryCompressor
{
  protected:
    /** Convenience typedef for a dictionary entry. */
    typedef std::array<uint8_t, sizeof(T)> DictionaryEntry;

    /**
     * Compression data for the dictionary compressor. It consists of a vector
     * of patterns.
     */
    class CompData;

    // Forward declaration of a pattern
    class Pattern;
    class UncompressedPattern;
    template <T mask>
    class MaskedPattern;
    template <T value, T mask>
    class MaskedValuePattern;
    template <T mask, int location>
    class LocatedMaskedPattern;
    template <class RepT>
    class RepeatedValuePattern;
    template <std::size_t DeltaSizeBits>
    class DeltaPattern;
    template <unsigned N>
    class SignExtendedPattern;

    /**
     * Create a factory to determine if input matches a pattern. The if else
     * chains are constructed by recursion. The patterns should be explored
     * sorted by size for correct behaviour.
     */
    template <class Head, class... Tail>
    struct Factory
    {
        static std::unique_ptr<Pattern>
        getPattern(const DictionaryEntry &bytes,
                   const DictionaryEntry &dict_bytes, const int match_location)
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
     * Specialization to end the recursion. This must be called when all
     * other patterns failed, and there is no choice but to leave data
     * uncompressed. As such, this pattern must inherit from the uncompressed
     * pattern.
     */
    template <class Head>
    struct Factory<Head>
    {
        static_assert(
            std::is_base_of_v<UncompressedPattern, Head>,
            "The last pattern must always be derived from the uncompressed "
            "pattern.");

        static std::unique_ptr<Pattern>
        getPattern(const DictionaryEntry &bytes,
                   const DictionaryEntry &dict_bytes, const int match_location)
        {
            return std::unique_ptr<Pattern>(new Head(bytes, match_location));
        }
    };

    /** The dictionary. */
    std::vector<DictionaryEntry> dictionary;

    /**
     * Since the factory cannot be instantiated here, classes that inherit
     * from this base class have to implement the call to their factory's
     * getPattern.
     */
    virtual std::unique_ptr<Pattern>
    getPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
               const int match_location) const = 0;

    /**
     * Compress data.
     *
     * @param data Data to be compressed.
     * @return The pattern this data matches.
     */
    std::unique_ptr<Pattern> compressValue(const T data);

    /**
     * Decompress a pattern into a value that fits in a dictionary entry.
     *
     * @param pattern The pattern to be decompressed.
     * @return The decompressed word.
     */
    T decompressValue(const Pattern *pattern);

    /** Clear all dictionary entries. */
    virtual void resetDictionary();

    /**
     * Add an entry to the dictionary.
     *
     * @param data The new entry.
     */
    virtual void addToDictionary(const DictionaryEntry data) = 0;

    /**
     * Instantiate a compression data of the sub-class compressor.
     *
     * @return The new compression data entry.
     */
    virtual std::unique_ptr<DictionaryCompressor::CompData>
    instantiateDictionaryCompData() const;

    /**
     * Apply compression.
     *
     * @param chunks The cache line to be compressed.
     * @return Cache line after compression.
     */
    std::unique_ptr<Base::CompressionData>
    compress(const std::vector<Chunk> &chunks);

    std::unique_ptr<Base::CompressionData>
    compress(const std::vector<Chunk> &chunks, Cycles &comp_lat,
             Cycles &decomp_lat) override;

    using BaseDictionaryCompressor::compress;

    void decompress(const CompressionData *comp_data, uint64_t *data) override;

    /**
     * Turn a value into a dictionary entry.
     *
     * @param value The value to turn.
     * @return A dictionary entry containing the value.
     */
    static DictionaryEntry toDictionaryEntry(T value);

    /**
     * Turn a dictionary entry into a value.
     *
     * @param The dictionary entry to turn.
     * @return The value that the dictionary entry contained.
     */
    static T fromDictionaryEntry(const DictionaryEntry &entry);

  public:
    typedef BaseDictionaryCompressorParams Params;
    DictionaryCompressor(const Params &p);
    ~DictionaryCompressor() = default;
};

/**
 * The compressed data is composed of multiple pattern entries. To add a new
 * pattern one should inherit from this class and implement isPattern() and
 * decompress(). Then the new pattern must be added to the PatternFactory
 * declaration in crescent order of size (in the DictionaryCompressor class).
 */
template <class T>
class DictionaryCompressor<T>::Pattern
{
  protected:
    /** Pattern enum number. */
    const int patternNumber;

    /** Code associated to the pattern. */
    const uint8_t code;

    /** Length, in bits, of the code and match location. */
    const uint8_t length;

    /** Number of unmatched bits. */
    const uint8_t numUnmatchedBits;

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
     * @param num_unmatched_bits Number of unmatched bits.
     * @param match_location Index of the match location.
     */
    Pattern(const int number, const uint64_t code,
            const uint64_t metadata_length, const uint64_t num_unmatched_bits,
            const int match_location, const bool allocate = true)
        : patternNumber(number),
          code(code),
          length(metadata_length),
          numUnmatchedBits(num_unmatched_bits),
          matchLocation(match_location),
          allocate(allocate)
    {}

    /** Default destructor. */
    virtual ~Pattern() = default;

    /**
     * Get enum number associated to this pattern.
     *
     * @return The pattern enum number.
     */
    int
    getPatternNumber() const
    {
        return patternNumber;
    };

    /**
     * Get code of this pattern.
     *
     * @return The code.
     */
    uint8_t
    getCode() const
    {
        return code;
    }

    /**
     * Get the index of the dictionary match location.
     *
     * @return The index of the match location.
     */
    uint8_t
    getMatchLocation() const
    {
        return matchLocation;
    }

    /**
     * Get size, in bits, of the pattern (excluding prefix). Corresponds to
     * unmatched_data_size + code_length.
     *
     * @return The size.
     */
    virtual std::size_t
    getSizeBits() const
    {
        return numUnmatchedBits + length;
    }

    /**
     * Determine if pattern allocates a dictionary entry.
     *
     * @return True if should allocate a dictionary entry.
     */
    bool
    shouldAllocate() const
    {
        return allocate;
    }

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
    virtual DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const = 0;
};

template <class T>
class DictionaryCompressor<T>::CompData : public CompressionData
{
  public:
    /** The patterns matched in the original line. */
    std::vector<std::unique_ptr<Pattern>> entries;

    CompData();
    ~CompData() = default;

    /**
     * Add a pattern entry to the list of patterns.
     *
     * @param entry The new pattern entry.
     */
    virtual void addEntry(std::unique_ptr<Pattern>);
};

/**
 * A pattern containing the original uncompressed data. This should be the
 * worst case of every pattern factory, where if all other patterns fail,
 * an instance of this pattern is created.
 */
template <class T>
class DictionaryCompressor<T>::UncompressedPattern :
    public DictionaryCompressor<T>::Pattern
{
  private:
    /** A copy of the original data. */
    const DictionaryEntry data;

  public:
    UncompressedPattern(const int number, const uint64_t code,
                        const uint64_t metadata_length,
                        const int match_location, const DictionaryEntry bytes)
        : DictionaryCompressor<T>::Pattern(number, code, metadata_length,
                                           sizeof(T) * 8, match_location,
                                           true),
          data(bytes)
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        // An entry can always be uncompressed
        return true;
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return data;
    }
};

/**
 * A pattern that compares masked values against dictionary entries. If
 * the masked dictionary entry matches perfectly the masked value to be
 * compressed, there is a pattern match.
 *
 * For example, if the mask is 0xFF00 (that is, this pattern matches the MSB),
 * the value (V) 0xFF20 is being compressed, and the dictionary contains
 * the value (D) 0xFF03, this is a match (V & mask == 0xFF00 == D & mask),
 * and 0x0020 is added to the list of unmatched bits.
 *
 * @tparam mask A mask containing the bits that must match.
 */
template <class T>
template <T mask>
class DictionaryCompressor<T>::MaskedPattern :
    public DictionaryCompressor<T>::Pattern
{
  private:
    static_assert(mask != 0, "The pattern's value mask must not be zero. Use "
                             "the uncompressed pattern instead.");

    /** A copy of the bits that do not belong to the mask. */
    const T bits;

  public:
    MaskedPattern(const int number, const uint64_t code,
                  const uint64_t metadata_length, const int match_location,
                  const DictionaryEntry bytes, const bool allocate = true)
        : DictionaryCompressor<T>::Pattern(number, code, metadata_length,
                                           popCount(static_cast<T>(~mask)),
                                           match_location, allocate),
          bits(DictionaryCompressor<T>::fromDictionaryEntry(bytes) & ~mask)
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        const T masked_bytes =
            DictionaryCompressor<T>::fromDictionaryEntry(bytes) & mask;
        const T masked_dict_bytes =
            DictionaryCompressor<T>::fromDictionaryEntry(dict_bytes) & mask;
        return (match_location >= 0) && (masked_bytes == masked_dict_bytes);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        const T masked_dict_bytes =
            DictionaryCompressor<T>::fromDictionaryEntry(dict_bytes) & mask;
        return DictionaryCompressor<T>::toDictionaryEntry(bits |
                                                          masked_dict_bytes);
    }
};

/**
 * A pattern that compares masked values to a masked portion of a fixed value.
 * If all the masked bits match the provided non-dictionary value, there is a
 * pattern match.
 *
 * For example, assume the mask is 0xFF00 (that is, this pattern matches the
 * MSB), and we are searching for data containing only ones (i.e., the fixed
 * value is 0xFFFF).
 * If the value (V) 0xFF20 is being compressed, this is a match (V & mask ==
 * 0xFF00 == 0xFFFF & mask), and 0x20 is added to the list of unmatched bits.
 * If the value (V2) 0x0120 is being compressed, this is not a match
 * ((V2 & mask == 0x0100) != (0xFF00 == 0xFFFF & mask).
 *
 * @tparam value The value that is being matched against.
 * @tparam mask A mask containing the bits that must match the given value.
 */
template <class T>
template <T value, T mask>
class DictionaryCompressor<T>::MaskedValuePattern : public MaskedPattern<mask>
{
  private:
    static_assert(mask != 0, "The pattern's value mask must not be zero.");

  public:
    MaskedValuePattern(const int number, const uint64_t code,
                       const uint64_t metadata_length,
                       const int match_location, const DictionaryEntry bytes,
                       const bool allocate = false)
        : MaskedPattern<mask>(number, code, metadata_length, match_location,
                              bytes, allocate)
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        // Compare the masked fixed value to the value being checked for
        // patterns. Since the dictionary is not being used the match_location
        // is irrelevant.
        const T masked_bytes =
            DictionaryCompressor<T>::fromDictionaryEntry(bytes) & mask;
        return ((value & mask) == masked_bytes);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return MaskedPattern<mask>::decompress(
            DictionaryCompressor<T>::toDictionaryEntry(value));
    }
};

/**
 * A pattern that narrows the MaskedPattern by allowing a only single possible
 * dictionary entry to be matched against.
 *
 * @tparam mask A mask containing the bits that must match.
 * @tparam location The index of the single entry allowed to match.
 */
template <class T>
template <T mask, int location>
class DictionaryCompressor<T>::LocatedMaskedPattern :
    public MaskedPattern<mask>
{
  public:
    LocatedMaskedPattern(const int number, const uint64_t code,
                         const uint64_t metadata_length,
                         const int match_location, const DictionaryEntry bytes,
                         const bool allocate = true)
        : MaskedPattern<mask>(number, code, metadata_length, match_location,
                              bytes, allocate)
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        // Besides doing the regular masked pattern matching, the match
        // location must match perfectly with this instance's
        return (match_location == location) &&
               MaskedPattern<mask>::isPattern(bytes, dict_bytes,
                                              match_location);
    }
};

/**
 * A pattern that checks if dictionary entry sized values are solely composed
 * of multiple copies of a single value.
 *
 * For example, if we are looking for repeated bytes in a 1-byte granularity
 * (RepT is uint8_t), the value 0x3232 would match, however 0x3332 wouldn't.
 *
 * @tparam RepT The type of the repeated value, which must fit in a dictionary
 *              entry.
 */
template <class T>
template <class RepT>
class DictionaryCompressor<T>::RepeatedValuePattern :
    public DictionaryCompressor<T>::Pattern
{
  private:
    static_assert(sizeof(T) > sizeof(RepT),
                  "The repeated value's type must "
                  "be smaller than the dictionary entry's type.");

    /** The repeated value. */
    RepT value;

  public:
    RepeatedValuePattern(const int number, const uint64_t code,
                         const uint64_t metadata_length,
                         const int match_location, const DictionaryEntry bytes,
                         const bool allocate = true)
        : DictionaryCompressor<T>::Pattern(number, code, metadata_length,
                                           8 * sizeof(RepT), match_location,
                                           allocate),
          value(DictionaryCompressor<T>::fromDictionaryEntry(bytes))
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        // Parse the dictionary entry in a RepT granularity, and if all values
        // are equal, this is a repeated value pattern. Since the dictionary
        // is not being used, the match_location is irrelevant
        T bytes_value = DictionaryCompressor<T>::fromDictionaryEntry(bytes);
        const RepT rep_value = bytes_value;
        for (int i = 0; i < (sizeof(T) / sizeof(RepT)); i++) {
            RepT cur_value = bytes_value;
            if (cur_value != rep_value) {
                return false;
            }
            bytes_value >>= 8 * sizeof(RepT);
        }
        return true;
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        // The decompressed value is just multiple consecutive instances of
        // the same value
        T decomp_value = 0;
        for (int i = 0; i < (sizeof(T) / sizeof(RepT)); i++) {
            decomp_value <<= 8 * sizeof(RepT);
            decomp_value |= value;
        }
        return DictionaryCompressor<T>::toDictionaryEntry(decomp_value);
    }
};

/**
 * A pattern that checks whether the difference of the value and the dictionary
 * entries' is below a certain threshold. If so, the pattern is successful,
 * and only the delta bits need to be stored.
 *
 * For example, if the delta can only contain up to 4 bits, and the dictionary
 * contains the entry 0xA231, the value 0xA232 would be compressible, and
 * the delta 0x1 would be stored. The value 0xA249, on the other hand, would
 * not be compressible, since its delta (0x18) needs 5 bits to be stored.
 *
 * @tparam DeltaSizeBits Size of a delta entry, in number of bits, which
 *                       determines the threshold. Must always be smaller
 *                       than the dictionary entry type's size.
 */
template <class T>
template <std::size_t DeltaSizeBits>
class DictionaryCompressor<T>::DeltaPattern :
    public DictionaryCompressor<T>::Pattern
{
  private:
    static_assert(DeltaSizeBits < (sizeof(T) * 8),
                  "Delta size must be smaller than base size");

    /**
     * The original value. In theory we should keep only the deltas, but
     * the dictionary entry is not inserted in the dictionary before the
     * call to the constructor, so the delta cannot be calculated then.
     */
    const DictionaryEntry bytes;

  public:
    DeltaPattern(const int number, const uint64_t code,
                 const uint64_t metadata_length, const int match_location,
                 const DictionaryEntry bytes)
        : DictionaryCompressor<T>::Pattern(number, code, metadata_length,
                                           DeltaSizeBits, match_location,
                                           false),
          bytes(bytes)
    {}

    /**
     * Compares a given value against a base to calculate their delta, and
     * then determines whether it fits a limited sized container.
     *
     * @param bytes Value to be compared against base.
     * @param base_bytes Base value.
     * @return Whether the value fits in the container.
     */
    static bool
    isValidDelta(const DictionaryEntry &bytes,
                 const DictionaryEntry &base_bytes)
    {
        const typename std::make_signed<T>::type limit =
            DeltaSizeBits ? mask(DeltaSizeBits - 1) : 0;
        const T value = DictionaryCompressor<T>::fromDictionaryEntry(bytes);
        const T base =
            DictionaryCompressor<T>::fromDictionaryEntry(base_bytes);
        const typename std::make_signed<T>::type delta = value - base;
        return (delta >= -limit) && (delta <= limit);
    }

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        return (match_location >= 0) && isValidDelta(bytes, dict_bytes);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return bytes;
    }
};

/**
 * A pattern that checks whether the value is an N bits sign-extended value,
 * that is, all the MSB starting from the Nth are equal to the (N-1)th bit.
 *
 * Therefore, if N = 8, and T has 16 bits, the values within the ranges
 * [0x0000, 0x007F] and [0xFF80, 0xFFFF] would match this pattern.
 *
 * @tparam N The number of bits in the non-extended original value. It must
 *           fit in a dictionary entry.
 */
template <class T>
template <unsigned N>
class DictionaryCompressor<T>::SignExtendedPattern :
    public DictionaryCompressor<T>::Pattern
{
  private:
    static_assert(
        (N > 0) & (N <= (sizeof(T) * 8)),
        "The original data's type size must be smaller than the dictionary's");

    /** The non-extended original value. */
    const T bits : N;

  public:
    SignExtendedPattern(const int number, const uint64_t code,
                        const uint64_t metadata_length,
                        const DictionaryEntry bytes,
                        const bool allocate = false)
        : DictionaryCompressor<T>::Pattern(number, code, metadata_length, N,
                                           -1, allocate),
          bits(fromDictionaryEntry(bytes) & mask(N))
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        const T data = DictionaryCompressor<T>::fromDictionaryEntry(bytes);
        return data == (T)szext<N>(data);
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        return toDictionaryEntry(sext<N>(bits));
    }
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_HH__
