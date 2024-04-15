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
 * Definition of the Frequent Pattern Compression cache compressor, as
 * described in "Frequent Pattern Compression: A Significance-Based
 * Compression Scheme for L2 Caches".
 */

#ifndef __MEM_CACHE_COMPRESSORS_FPC_HH__
#define __MEM_CACHE_COMPRESSORS_FPC_HH__

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "base/bitfield.hh"
#include "base/types.hh"
#include "mem/cache/compressors/dictionary_compressor.hh"

namespace gem5
{

struct FPCParams;

namespace compression
{

class FPC : public DictionaryCompressor<uint32_t>
{
  private:
    using DictionaryEntry = DictionaryCompressor<uint32_t>::DictionaryEntry;

    /**
     * Compression data for FPC. It contains a list of the patterns
     * encountered while parsing the cache line.
     */
    class FPCCompData;

    // Declaration of all possible patterns
    class ZeroRun;
    class SignExtended4Bits;
    class SignExtended1Byte;
    class SignExtendedHalfword;
    class ZeroPaddedHalfword;
    class SignExtendedTwoHalfwords;
    class RepBytes;
    class Uncompressed;

    /**
     * The possible patterns. If a new pattern is added, it must be done
     * before NUM_PATTERNS.
     */
    enum PatternNumber
    {
        ZERO_RUN,
        SIGN_EXTENDED_4_BITS,
        SIGN_EXTENDED_1_BYTE,
        SIGN_EXTENDED_HALFWORD,
        ZERO_PADDED_HALFWORD,
        SIGN_EXTENDED_TWO_HALFWORDS,
        REP_BYTES,
        UNCOMPRESSED,
        NUM_PATTERNS
    };

    /**
     * Number of bits of the zero run size bitfield. If the size of the
     * zero run reaches the maximum value, it is split into ZERO_RUN entries.
     */
    const int zeroRunSizeBits;

    uint64_t
    getNumPatterns() const override
    {
        return NUM_PATTERNS;
    }

    std::string
    getName(int number) const override
    {
        static std::map<int, std::string> patternNames = {
            { ZERO_RUN, "ZERO_RUN" },
            { SIGN_EXTENDED_4_BITS, "SignExtended4Bits" },
            { SIGN_EXTENDED_1_BYTE, "SignExtended1Byte" },
            { SIGN_EXTENDED_HALFWORD, "SignExtendedHalfword" },
            { ZERO_PADDED_HALFWORD, "ZeroPaddedHalfword" },
            { SIGN_EXTENDED_TWO_HALFWORDS, "SignExtendedTwoHalfwords" },
            { REP_BYTES, "RepBytes" },
            { UNCOMPRESSED, "Uncompressed" }
        };

        return patternNames[number];
    };

    std::unique_ptr<Pattern>
    getPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
               const int match_location) const override
    {
        using PatternFactory =
            Factory<ZeroRun, SignExtended4Bits, SignExtended1Byte,
                    SignExtendedHalfword, ZeroPaddedHalfword,
                    SignExtendedTwoHalfwords, RepBytes, Uncompressed>;
        return PatternFactory::getPattern(bytes, dict_bytes, match_location);
    }

    void addToDictionary(const DictionaryEntry data) override;

    std::unique_ptr<DictionaryCompressor::CompData>
    instantiateDictionaryCompData() const override;

  public:
    typedef FPCParams Params;
    FPC(const Params &p);
    ~FPC() = default;
};

class FPC::FPCCompData : public DictionaryCompressor<uint32_t>::CompData
{
  protected:
    /**
     * Number of bits of the zero run size bitfield. If the size of the
     * zero run reaches the maximum value, it is split into ZERO_RUN entries.
     */
    const int zeroRunSizeBits;

  public:
    FPCCompData(int zeroRunSizeBits);
    ~FPCCompData() = default;

    void addEntry(std::unique_ptr<Pattern> pattern) override;
};

// Pattern implementations

class FPC::ZeroRun : public MaskedValuePattern<0, 0xFFFFFFFF>
{
  private:
    /** Run length so far. */
    int _runLength;

    /**
     * A zero run consists of a main ZeroRun pattern, which has a meaningful
     * real size (i.e., different from zero), and X-1 fake (i.e., they are
     * zero-sized, and don't exist in a real implementation) patterns, with X
     * being the size of the zero run.
     */
    int _realSize;

  public:
    ZeroRun(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFFFFFF>(ZERO_RUN, ZERO_RUN, 3, -1, bytes,
                                            false),
          _runLength(0),
          _realSize(0)
    {}

    std::size_t
    getSizeBits() const override
    {
        return _realSize;
    }

    /**
     * Get the number of zeros in the run so far.
     *
     * @return The number of zeros in this run.
     */
    int
    getRunLength() const
    {
        return _runLength;
    }

    /**
     * Set the number of zeros in the run so far.
     *
     * @param The number of zeros in this run.
     */
    void
    setRunLength(int length)
    {
        _runLength = length;
    }

    /**
     * When the real size is set it means that we are adding the main zero
     * run pattern. When that happens, the metadata length must also be taken
     * into account for the size calculation.
     *
     * @param size Number of bits used to represent the number of zeros in the
     *             run.
     */
    void
    setRealSize(int size)
    {
        _realSize = length + size;
    }
};

class FPC::SignExtended4Bits : public SignExtendedPattern<4>
{
  public:
    SignExtended4Bits(const DictionaryEntry bytes, const int match_location)
        : SignExtendedPattern<4>(SIGN_EXTENDED_4_BITS, SIGN_EXTENDED_4_BITS, 3,
                                 bytes)
    {}
};

class FPC::SignExtended1Byte : public SignExtendedPattern<8>
{
  public:
    SignExtended1Byte(const DictionaryEntry bytes, const int match_location)
        : SignExtendedPattern<8>(SIGN_EXTENDED_1_BYTE, SIGN_EXTENDED_1_BYTE, 3,
                                 bytes)
    {}
};

class FPC::SignExtendedHalfword : public SignExtendedPattern<16>
{
  public:
    SignExtendedHalfword(const DictionaryEntry bytes, const int match_location)
        : SignExtendedPattern<16>(SIGN_EXTENDED_HALFWORD,
                                  SIGN_EXTENDED_HALFWORD, 3, bytes)
    {}
};

class FPC::ZeroPaddedHalfword : public MaskedValuePattern<0, 0x0000FFFF>
{
  public:
    ZeroPaddedHalfword(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0x0000FFFF>(
              ZERO_PADDED_HALFWORD, ZERO_PADDED_HALFWORD, 3, -1, bytes, false)
    {}
};

class FPC::SignExtendedTwoHalfwords : public Pattern
{
  private:
    /** These are the bytes that are extended to form the two halfwords. */
    const int8_t extendedBytes[2];

  public:
    SignExtendedTwoHalfwords(const DictionaryEntry bytes,
                             const int match_location)
        : Pattern(SIGN_EXTENDED_TWO_HALFWORDS, SIGN_EXTENDED_TWO_HALFWORDS, 3,
                  16, -1, false),
          extendedBytes{ int8_t(fromDictionaryEntry(bytes) & mask(8)),
                         int8_t((fromDictionaryEntry(bytes) >> 16) & mask(8)) }
    {}

    static bool
    isPattern(const DictionaryEntry &bytes, const DictionaryEntry &dict_bytes,
              const int match_location)
    {
        const uint32_t data = fromDictionaryEntry(bytes);
        const int16_t halfwords[2] = { int16_t(data & mask(16)),
                                       int16_t((data >> 16) & mask(16)) };
        return (halfwords[0] == (uint16_t)szext<8>(halfwords[0])) &&
               (halfwords[1] == (uint16_t)szext<8>(halfwords[1]));
    }

    DictionaryEntry
    decompress(const DictionaryEntry dict_bytes) const override
    {
        uint16_t halfwords[2] = { (uint16_t)szext<8>(extendedBytes[0]),
                                  (uint16_t)szext<8>(extendedBytes[1]) };
        return toDictionaryEntry((halfwords[1] << 16) | halfwords[0]);
    }
};

class FPC::RepBytes : public RepeatedValuePattern<uint8_t>
{
  public:
    RepBytes(const DictionaryEntry bytes, const int match_location)
        : RepeatedValuePattern<uint8_t>(REP_BYTES, REP_BYTES, 3, -1, bytes,
                                        false)
    {}
};

class FPC::Uncompressed : public UncompressedPattern
{
  public:
    Uncompressed(const DictionaryEntry bytes, const int match_location)
        : UncompressedPattern(UNCOMPRESSED, UNCOMPRESSED, 3, -1, bytes)
    {}
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_FPC_HH__
