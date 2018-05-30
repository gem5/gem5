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
 * Implementation of the CPack cache compressor.
 */

#include "mem/cache/compressors/cpack.hh"

#include <algorithm>
#include <cstdint>

#include "debug/CacheComp.hh"
#include "params/CPack.hh"

CPack::CompData::CompData(const std::size_t dictionary_size)
    : CompressionData()
{
}

CPack::CompData::~CompData()
{
}

CPack::CPack(const Params *p)
    : BaseCacheCompressor(p), dictionarySize(2*blkSize/8)
{
    dictionary.resize(dictionarySize);

    resetDictionary();
}

void
CPack::resetDictionary()
{
    // Reset number of valid entries
    numEntries = 0;

    // Set all entries as 0
    std::array<uint8_t, 4> zero_word = {0, 0, 0, 0};
    std::fill(dictionary.begin(), dictionary.end(), zero_word);
}

std::unique_ptr<CPack::Pattern>
CPack::compressWord(const uint32_t data)
{
    // Split data in bytes
    const std::array<uint8_t, 4> bytes = {
        static_cast<uint8_t>(data & 0xFF),
        static_cast<uint8_t>((data >> 8) & 0xFF),
        static_cast<uint8_t>((data >> 16) & 0xFF),
        static_cast<uint8_t>((data >> 24) & 0xFF)
    };

    // Start as a no-match pattern. A negative match location is used so that
    // patterns that depend on the dictionary entry don't match
    std::unique_ptr<Pattern> pattern =
        PatternFactory::getPattern(bytes, {0, 0, 0, 0}, -1);

    // Search for word on dictionary
    for (std::size_t i = 0; i < numEntries; i++) {
        // Try matching input with possible patterns
        std::unique_ptr<Pattern> temp_pattern =
            PatternFactory::getPattern(bytes, dictionary[i], i);

        // Check if found pattern is better than previous
        if (temp_pattern->getSizeBits() < pattern->getSizeBits()) {
            pattern = std::move(temp_pattern);
        }
    }

    // Update stats
    patternStats[pattern->getPatternNumber()]++;

    // Push into dictionary
    if ((numEntries < dictionarySize) && pattern->shouldAllocate()) {
        dictionary[numEntries++] = bytes;
    }

    return pattern;
}

std::unique_ptr<BaseCacheCompressor::CompressionData>
CPack::compress(const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat)
{
    std::unique_ptr<CompData> comp_data =
        std::unique_ptr<CompData>(new CompData(dictionarySize));

    // Compression size
    std::size_t size = 0;

    // Reset dictionary
    resetDictionary();

    // Compress every word sequentially
    for (std::size_t i = 0; i < blkSize/8; i++) {
        const uint32_t first_word = ((data[i])&0xFFFFFFFF00000000) >> 32;
        const uint32_t second_word = (data[i])&0x00000000FFFFFFFF;

        // Compress both words
        std::unique_ptr<Pattern> first_pattern = compressWord(first_word);
        std::unique_ptr<Pattern> second_pattern = compressWord(second_word);

        // Update total line compression size
        size += first_pattern->getSizeBits() + second_pattern->getSizeBits();

        // Print debug information
        DPRINTF(CacheComp, "Compressed %08x to %s\n", first_word,
                first_pattern->print());
        DPRINTF(CacheComp, "Compressed %08x to %s\n", second_word,
                second_pattern->print());

        // Append to pattern list
        comp_data->entries.push_back(std::move(first_pattern));
        comp_data->entries.push_back(std::move(second_pattern));
    }

    // Set final compression size
    comp_data->setSizeBits(size);

    // Set compression latency (Accounts for pattern matching, length
    // generation, packaging and shifting)
    comp_lat = Cycles(blkSize/8+5);

    // Set decompression latency (1 qword per cycle)
    decomp_lat = Cycles(blkSize/8);

    // Return compressed line
    return std::move(comp_data);
}

uint32_t
CPack::decompressWord(const Pattern* pattern)
{
    std::array<uint8_t, 4> data;

    // Search for matching entry
    std::vector<std::array<uint8_t, 4>>::iterator entry_it =
        dictionary.begin();
    std::advance(entry_it, pattern->getMatchLocation());

    // Decompress the match. If the decompressed value must be added to
    // the dictionary, do it
    if (pattern->decompress(*entry_it, data)) {
        dictionary[numEntries++] = data;
    }

    // Return word
    return (((((data[3] << 8) | data[2]) << 8) | data[1]) << 8) | data[0];
}

void
CPack::decompress(const CompressionData* comp_data, uint64_t* data)
{
    const CompData* cpack_comp_data = static_cast<const CompData*>(comp_data);

    // Reset dictionary
    resetDictionary();

    // Decompress every entry sequentially
    std::vector<uint32_t> decomp_words;
    for (const auto& entry : cpack_comp_data->entries) {
        const uint32_t word = decompressWord(&*entry);
        decomp_words.push_back(word);

        // Print debug information
        DPRINTF(CacheComp, "Decompressed %s to %x\n", entry->print(), word);
    }

    // Concatenate the decompressed words to generate the cache lines
    for (std::size_t i = 0; i < blkSize/8; i++) {
        data[i] = (static_cast<uint64_t>(decomp_words[2*i]) << 32) |
                        decomp_words[2*i+1];
    }
}

void
CPack::regStats()
{
    BaseCacheCompressor::regStats();

    // We store the frequency of each pattern
    patternStats
        .init(Pattern::getNumPatterns())
        .name(name() + ".pattern")
        .desc("Number of data entries that were compressed to this pattern.")
        ;

    for (unsigned i = 0; i < Pattern::getNumPatterns(); ++i) {
        patternStats.subname(i, Pattern::getName(i));
        patternStats.subdesc(i, "Number of data entries that match pattern " +
                                Pattern::getName(i));
    }
}

CPack*
CPackParams::create()
{
    return new CPack(this);
}
