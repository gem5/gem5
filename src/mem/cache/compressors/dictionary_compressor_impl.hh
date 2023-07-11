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
 * Implementation of a dictionary based cache compressor.
 */

#ifndef __MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_IMPL_HH__
#define __MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_IMPL_HH__

#include <algorithm>

#include "base/trace.hh"
#include "debug/CacheComp.hh"
#include "mem/cache/compressors/dictionary_compressor.hh"
#include "params/BaseDictionaryCompressor.hh"

namespace gem5
{

namespace compression
{

template <class T>
DictionaryCompressor<T>::CompData::CompData()
    : CompressionData()
{
}

template <class T>
void
DictionaryCompressor<T>::CompData::addEntry(std::unique_ptr<Pattern> pattern)
{
    // Increase size
    setSizeBits(getSizeBits() + pattern->getSizeBits());

    // Push new entry to list
    entries.push_back(std::move(pattern));
}

template <class T>
DictionaryCompressor<T>::DictionaryCompressor(const Params &p)
    : BaseDictionaryCompressor(p)
{
    dictionary.resize(dictionarySize);

    resetDictionary();
}

template <class T>
void
DictionaryCompressor<T>::resetDictionary()
{
    // Reset number of valid entries
    numEntries = 0;

    // Set all entries as 0
    std::fill(dictionary.begin(), dictionary.end(), toDictionaryEntry(0));
}

template <typename T>
std::unique_ptr<typename DictionaryCompressor<T>::CompData>
DictionaryCompressor<T>::instantiateDictionaryCompData() const
{
    return std::unique_ptr<DictionaryCompressor<T>::CompData>(new CompData());
}

template <typename T>
std::unique_ptr<typename DictionaryCompressor<T>::Pattern>
DictionaryCompressor<T>::compressValue(const T data)
{
    // Split data in bytes
    const DictionaryEntry bytes = toDictionaryEntry(data);

    // Start as a no-match pattern. A negative match location is used so that
    // patterns that depend on the dictionary entry don't match
    std::unique_ptr<Pattern> pattern =
        getPattern(bytes, toDictionaryEntry(0), -1);

    // Search for word on dictionary
    for (std::size_t i = 0; i < numEntries; i++) {
        // Try matching input with possible patterns
        std::unique_ptr<Pattern> temp_pattern =
            getPattern(bytes, dictionary[i], i);

        // Check if found pattern is better than previous
        if (temp_pattern->getSizeBits() < pattern->getSizeBits()) {
            pattern = std::move(temp_pattern);
        }
    }

    // Update stats
    dictionaryStats.patterns[pattern->getPatternNumber()]++;

    // Push into dictionary
    if (pattern->shouldAllocate()) {
        addToDictionary(bytes);
    }

    return pattern;
}

template <class T>
std::unique_ptr<Base::CompressionData>
DictionaryCompressor<T>::compress(const std::vector<Chunk>& chunks)
{
    std::unique_ptr<Base::CompressionData> comp_data =
        instantiateDictionaryCompData();

    // Reset dictionary
    resetDictionary();

    // Compress every value sequentially
    CompData* const comp_data_ptr = static_cast<CompData*>(comp_data.get());
    for (const auto& value : chunks) {
        std::unique_ptr<Pattern> pattern = compressValue(value);
        DPRINTF(CacheComp, "Compressed %016x to %s\n", value,
            pattern->print());
        comp_data_ptr->addEntry(std::move(pattern));
    }

    // Return compressed line
    return comp_data;
}

template <class T>
std::unique_ptr<Base::CompressionData>
DictionaryCompressor<T>::compress(const std::vector<Chunk>& chunks,
    Cycles& comp_lat, Cycles& decomp_lat)
{
    // Set latencies based on the degree of parallelization, and any extra
    // latencies due to shifting or packaging
    comp_lat = Cycles(compExtraLatency +
        (chunks.size() / compChunksPerCycle));
    decomp_lat = Cycles(decompExtraLatency +
        (chunks.size() / decompChunksPerCycle));

    return compress(chunks);
}

template <class T>
T
DictionaryCompressor<T>::decompressValue(const Pattern* pattern)
{
    // Search for matching entry
    auto entry_it = dictionary.begin();
    std::advance(entry_it, pattern->getMatchLocation());

    // Decompress the match. If the decompressed value must be added to
    // the dictionary, do it
    const DictionaryEntry data = pattern->decompress(*entry_it);
    if (pattern->shouldAllocate()) {
        addToDictionary(data);
    }

    // Return value
    return fromDictionaryEntry(data);
}

template <class T>
void
DictionaryCompressor<T>::decompress(const CompressionData* comp_data,
    uint64_t* data)
{
    const CompData* casted_comp_data = static_cast<const CompData*>(comp_data);

    // Reset dictionary
    resetDictionary();

    // Decompress every entry sequentially
    std::vector<T> decomp_values;
    for (const auto& entry : casted_comp_data->entries) {
        const T value = decompressValue(&*entry);
        decomp_values.push_back(value);
        DPRINTF(CacheComp, "Decompressed %s to %x\n", entry->print(), value);
    }

    // Concatenate the decompressed values to generate the original data
    for (std::size_t i = 0; i < blkSize/8; i++) {
        data[i] = 0;
        const std::size_t values_per_entry = sizeof(uint64_t)/sizeof(T);
        for (int j = values_per_entry - 1; j >= 0; j--) {
            data[i] |=
                static_cast<uint64_t>(decomp_values[values_per_entry*i+j]) <<
                (j*8*sizeof(T));
        }
    }
}

template <class T>
typename DictionaryCompressor<T>::DictionaryEntry
DictionaryCompressor<T>::toDictionaryEntry(T value)
{
    DictionaryEntry entry;
    for (int i = 0; i < sizeof(T); i++) {
        entry[i] = value & 0xFF;
        value >>= 8;
    }
    return entry;
}

template <class T>
T
DictionaryCompressor<T>::fromDictionaryEntry(const DictionaryEntry& entry)
{
    T value = 0;
    for (int i = sizeof(T) - 1; i >= 0; i--) {
        value <<= 8;
        value |= entry[i];
    }
    return value;
}

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_DICTIONARY_COMPRESSOR_IMPL_HH__
