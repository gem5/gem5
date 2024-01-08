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
 * Definition of the Frequent Pattern Compression with limited Dictionary
 * support (FPC-D) cache compressor, as described in "Opportunistic
 * Compression for Direct-Mapped DRAM Caches", by Alameldeen et al.
 *
 * It is a pattern compressor that can only have 2 dictionary entries, and
 * as such the pattern encodings are specialized to inform to which entry it
 * refers. These entries are replaced in a FIFO manner.
 */

#ifndef __MEM_CACHE_COMPRESSORS_FPCD_HH__
#define __MEM_CACHE_COMPRESSORS_FPCD_HH__

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "base/types.hh"
#include "mem/cache/compressors/dictionary_compressor.hh"

namespace gem5
{

struct FPCDParams;

namespace compression
{

class FPCD : public DictionaryCompressor<uint32_t>
{
  private:
    using DictionaryEntry = DictionaryCompressor<uint32_t>::DictionaryEntry;

    /** Number of bits in a FPCD pattern prefix. */
    static constexpr int prefixSize = 4;

    /** Index of the previous dictionary entry. */
    static constexpr int previousIndex = 1;

    /** Index of the penultimate dictionary entry. */
    static constexpr int penultimateIndex = 0;

    // Declaration of all possible patterns, from lowest to highest sizes.
    // Penultimate is prioritized over previous to reduce the ripple effect
    // of propagating values during decompression
    class PatternZZZZ;
    class PatternFFFF;
    class PatternMMMMPenultimate;
    class PatternMMMMPrevious;
    class PatternZZZX;
    class PatternXZZZ;
    class PatternRRRR;
    class PatternMMMXPenultimate;
    class PatternMMMXPrevious;
    class PatternZZXX;
    class PatternZXZX;
    class PatternFFXX;
    class PatternXXZZ;
    class PatternMMXXPenultimate;
    class PatternMMXXPrevious;
    class PatternXXXX;

    /**
     * The patterns proposed in the paper. Each letter represents a byte:
     * Z is a null byte, M is a dictionary match, X is a new value, R is
     * a repeated value.
     * These are used as indexes to reference the pattern data. If a new
     * pattern is added, it must be done before NUM_PATTERNS.
     */
    enum PatternNumber
    {
        ZZZZ, FFFF, MMMMPenultimate, MMMMPrevious, ZZZX, XZZZ, RRRR,
        MMMXPenultimate, MMMXPrevious, ZZXX, ZXZX, FFXX, XXZZ,
        MMXXPenultimate, MMXXPrevious, XXXX, NUM_PATTERNS
    };

    /**
     * Convenience factory declaration. The templates must be organized by
     * size, with the smallest first, and "no-match" last.
     */
    using PatternFactory =
        Factory<PatternZZZZ, PatternFFFF, PatternMMMMPrevious,
                PatternMMMMPenultimate, PatternZZZX, PatternXZZZ,
                PatternRRRR, PatternMMMXPrevious, PatternMMMXPenultimate,
                PatternZZXX, PatternZXZX, PatternFFXX, PatternXXZZ,
                PatternMMXXPrevious, PatternMMXXPenultimate, PatternXXXX>;

    uint64_t getNumPatterns() const override { return NUM_PATTERNS; }

    std::string
    getName(int number) const override
    {
        static std::map<PatternNumber, std::string> pattern_names = {
            {ZZZZ, "ZZZZ"}, {FFFF, "FFFF"},
            {MMMMPenultimate, "MMMMPenultimate"},
            {MMMMPrevious, "MMMMPrevious"}, {ZZZX, "ZZZX"},
            {XZZZ, "XZZZ"}, {RRRR, "RRRR"},
            {MMMXPenultimate, "MMMXPenultimate"},
            {MMMXPrevious, "MMMXPrevious"},
            {ZZXX, "ZZXX"},
            {ZXZX, "ZXZX"}, {FFXX, "FFXX"}, {XXZZ, "XXZZ"},
            {MMXXPenultimate, "MMXXPenultimate"},
            {MMXXPrevious, "MMXXPrevious"}, {XXXX, "XXXX"}
        };

        return pattern_names[(PatternNumber)number];
    };

    std::unique_ptr<Pattern>
    getPattern(const DictionaryEntry& bytes, const DictionaryEntry& dict_bytes,
        const int match_location) const override
    {
        return PatternFactory::getPattern(bytes, dict_bytes, match_location);
    }

    void addToDictionary(DictionaryEntry data) override;

  public:
    typedef FPCDParams Params;
    FPCD(const Params &p);
    ~FPCD() = default;
};

class FPCD::PatternZZZZ : public MaskedValuePattern<0, 0xFFFFFFFF>
{
  public:
    PatternZZZZ(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFFFFFF>(ZZZZ, 0x0, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternFFFF : public MaskedValuePattern<0xFFFFFFFF, 0xFFFFFFFF>
{
  public:
    PatternFFFF(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0xFFFFFFFF, 0xFFFFFFFF>(FFFF, 0x1,
          prefixSize, match_location, bytes, true)
    {
    }
};

class FPCD::PatternMMMMPrevious
    : public LocatedMaskedPattern<0xFFFFFFFF, previousIndex>
{
  public:
    PatternMMMMPrevious(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFFFFFF, previousIndex>(MMMMPrevious,
          0x2, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternMMMMPenultimate
    : public LocatedMaskedPattern<0xFFFFFFFF, penultimateIndex>
{
  public:
    PatternMMMMPenultimate(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFFFFFF, penultimateIndex>(
          MMMMPenultimate, 0x3, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternZZZX : public MaskedValuePattern<0, 0xFFFFFF00>
{
  public:
    PatternZZZX(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFFFF00>(ZZZX, 0x4, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternXZZZ : public MaskedValuePattern<0, 0x00FFFFFF>
{
  public:
    PatternXZZZ(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0x00FFFFFF>(XZZZ, 0x5, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternRRRR : public RepeatedValuePattern<uint8_t>
{
  public:
    PatternRRRR(const DictionaryEntry bytes, const int match_location)
        : RepeatedValuePattern<uint8_t>(RRRR, 0x6, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternMMMXPrevious
    : public LocatedMaskedPattern<0xFFFFFF00, previousIndex>
{
  public:
    PatternMMMXPrevious(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFFFF00, previousIndex>(MMMXPrevious,
          0x7, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternMMMXPenultimate
    : public LocatedMaskedPattern<0xFFFFFF00, penultimateIndex>
{
  public:
    PatternMMMXPenultimate(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFFFF00, penultimateIndex>(
          MMMXPenultimate, 0x8, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternZZXX : public MaskedValuePattern<0, 0xFFFF0000>
{
  public:
    PatternZZXX(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFFFF0000>(ZZXX, 0x9, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternZXZX : public MaskedValuePattern<0, 0xFF00FF00>
{
  public:
    PatternZXZX(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0xFF00FF00>(ZXZX, 0xA, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternFFXX : public MaskedValuePattern<0xFFFFFFFF, 0xFFFF0000>
{
  public:
    PatternFFXX(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0xFFFFFFFF, 0xFFFF0000>(FFXX, 0xB,
          prefixSize, match_location, bytes, true)
    {
    }
};

class FPCD::PatternXXZZ : public MaskedValuePattern<0, 0x0000FFFF>
{
  public:
    PatternXXZZ(const DictionaryEntry bytes, const int match_location)
        : MaskedValuePattern<0, 0x0000FFFF>(XXZZ, 0xC, prefixSize,
          match_location, bytes, true)
    {
    }
};

class FPCD::PatternMMXXPrevious
    : public LocatedMaskedPattern<0xFFFF0000, previousIndex>
{
  public:
    PatternMMXXPrevious(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFF0000, previousIndex>(MMXXPrevious,
          0xD, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternMMXXPenultimate
    : public LocatedMaskedPattern<0xFFFF0000, penultimateIndex>
{
  public:
    PatternMMXXPenultimate(const DictionaryEntry bytes,
        const int match_location)
        : LocatedMaskedPattern<0xFFFF0000, penultimateIndex>(
          MMXXPenultimate, 0xE, prefixSize, match_location, bytes)
    {
    }
};

class FPCD::PatternXXXX : public UncompressedPattern
{
  public:
    PatternXXXX(const DictionaryEntry bytes, const int match_location)
        : UncompressedPattern(XXXX, 0xF, prefixSize, match_location,
          bytes)
    {
    }
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_FPCD_HH__
