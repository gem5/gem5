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
 * Implementation of the FPC-D cache compressor.
 */

#include "mem/cache/compressors/fpcd.hh"

#include "mem/cache/compressors/dictionary_compressor_impl.hh"
#include "params/FPCD.hh"

namespace Compressor {

FPCD::FPCD(const Params *p)
    : DictionaryCompressor<uint32_t>(p)
{
}

void
FPCD::addToDictionary(DictionaryEntry data)
{
    // The dictionary behaves as a table with FIFO as replacement policy
    if (numEntries == 2) {
        dictionary[penultimateIndex] = dictionary[previousIndex];
        dictionary[previousIndex] = data;
    } else {
        dictionary[numEntries++] = data;
    }
}

std::unique_ptr<Base::CompressionData>
FPCD::compress(const std::vector<Chunk>& chunks,
    Cycles& comp_lat, Cycles& decomp_lat)
{
    std::unique_ptr<Base::CompressionData> comp_data =
        DictionaryCompressor<uint32_t>::compress(chunks);

    // Set compression latency (Accounts for zero checks, ones check, match
    // previous check, match penultimate check, repeated values check, pattern
    // selection, shifting, at a rate of 16B per cycle)
    comp_lat = Cycles(blkSize/2);

    // Set decompression latency. The original claim of 2 cycles is likely
    // too unrealistic
    decomp_lat = Cycles(4);

    // Return compressed line
    return comp_data;
}

} // namespace Compressor

Compressor::FPCD*
FPCDParams::create()
{
    return new Compressor::FPCD(this);
}
