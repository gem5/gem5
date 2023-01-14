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
 * Implementation of a base sim object for the templated dictionary-based
 * cache compressor.
 */

#include "mem/cache/compressors/dictionary_compressor.hh"
#include "params/BaseDictionaryCompressor.hh"

namespace gem5
{

namespace compression
{

BaseDictionaryCompressor::BaseDictionaryCompressor(const Params &p)
  : Base(p), dictionarySize(p.dictionary_size),
    numEntries(0), dictionaryStats(stats, *this)
{
}

BaseDictionaryCompressor::DictionaryStats::DictionaryStats(
    BaseStats& base_group, BaseDictionaryCompressor& _compressor)
  : statistics::Group(&base_group), compressor(_compressor),
    ADD_STAT(patterns, statistics::units::Count::get(),
             "Number of data entries that were compressed to this pattern")
{
}

void
BaseDictionaryCompressor::DictionaryStats::regStats()
{
    statistics::Group::regStats();

    // We store the frequency of each pattern
    patterns.init(compressor.getNumPatterns());
    for (unsigned i = 0; i < compressor.getNumPatterns(); ++i) {
        const std::string name = compressor.getName(i);
        patterns.subname(i, name);
        patterns.subdesc(i, "Number of data entries that match pattern " +
            name);
    }
}

} // namespace compression
} // namespace gem5
