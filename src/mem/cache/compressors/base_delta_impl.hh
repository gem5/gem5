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
 * Implementation of a base delta immediate compressor. @see BDI
 */

#ifndef __MEM_CACHE_COMPRESSORS_BASE_DELTA_IMPL_HH__
#define __MEM_CACHE_COMPRESSORS_BASE_DELTA_IMPL_HH__

#include "debug/CacheComp.hh"
#include "mem/cache/compressors/base_delta.hh"
#include "mem/cache/compressors/dictionary_compressor_impl.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Compressor, compression);
namespace compression
{

template <class BaseType, std::size_t DeltaSizeBits>
BaseDelta<BaseType, DeltaSizeBits>::BaseDelta(const Params &p)
    : DictionaryCompressor<BaseType>(p)
{
}

template <class BaseType, std::size_t DeltaSizeBits>
void
BaseDelta<BaseType, DeltaSizeBits>::resetDictionary()
{
    DictionaryCompressor<BaseType>::resetDictionary();

    // Add zero base for the immediate values
    addToDictionary(DictionaryCompressor<BaseType>::toDictionaryEntry(0));
}

template <class BaseType, std::size_t DeltaSizeBits>
void
BaseDelta<BaseType, DeltaSizeBits>::addToDictionary(DictionaryEntry data)
{
    assert(DictionaryCompressor<BaseType>::numEntries <
        DictionaryCompressor<BaseType>::dictionarySize);
    DictionaryCompressor<BaseType>::dictionary[
        DictionaryCompressor<BaseType>::numEntries++] = data;
}

template <class BaseType, std::size_t DeltaSizeBits>
std::unique_ptr<Base::CompressionData>
BaseDelta<BaseType, DeltaSizeBits>::compress(
    const std::vector<Base::Chunk>& chunks, Cycles& comp_lat,
    Cycles& decomp_lat)
{
    std::unique_ptr<Base::CompressionData> comp_data =
        DictionaryCompressor<BaseType>::compress(chunks, comp_lat, decomp_lat);

    // If there are more bases than the maximum, the compressor failed.
    // Otherwise, we have to take into account all bases that have not
    // been used, considering that there is an implicit zero base that
    // does not need to be added to the final size.
    const int diff = DEFAULT_MAX_NUM_BASES -
        DictionaryCompressor<BaseType>::numEntries;
    if (diff < 0) {
        comp_data->setSizeBits(DictionaryCompressor<BaseType>::blkSize * 8);
        DPRINTF(CacheComp, "Base%dDelta%d compression failed\n",
            8 * sizeof(BaseType), DeltaSizeBits);
    } else if (diff > 0) {
        comp_data->setSizeBits(comp_data->getSizeBits() +
            8 * sizeof(BaseType) * diff);
    }

    // Return compressed line
    return comp_data;
}

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_BASE_DELTA_IMPL_HH__
