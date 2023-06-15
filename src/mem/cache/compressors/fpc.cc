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

#include "mem/cache/compressors/fpc.hh"

#include "mem/cache/compressors/dictionary_compressor_impl.hh"
#include "params/FPC.hh"

namespace gem5
{

namespace compression
{

FPC::FPCCompData::FPCCompData(int zero_run_size_bits)
  : CompData(), zeroRunSizeBits(zero_run_size_bits)
{
}

void
FPC::FPCCompData::addEntry(std::unique_ptr<Pattern> pattern)
{
    // If this is a zero match, check for zero runs
    if (pattern->getPatternNumber() == ZERO_RUN) {
        // If it is a new zero run, create it; otherwise, increase current
        // run's length
        if (!entries.size() ||
            (entries.back()->getPatternNumber() != ZERO_RUN)) {
            static_cast<ZeroRun*>(pattern.get())->setRealSize(zeroRunSizeBits);
        } else {
            // A zero run has a maximum length, given by the number of bits
            // used to represent it. When this limit is reached, a new run
            // must be created
            const int run_length =
                static_cast<ZeroRun*>(entries.back().get())->getRunLength();
            if (run_length == mask(zeroRunSizeBits)) {
                // The limit for this zero run has been reached, so a new
                // run must be started, with a sized pattern
                static_cast<ZeroRun*>(pattern.get())->setRealSize(
                    zeroRunSizeBits);
            } else {
                // Increase the current run's length.
                // Since the first zero entry of the run contains the size,
                // and all the following ones are created just to simplify
                // decompression, this fake pattern will have a size of 0 bits
                static_cast<ZeroRun*>(pattern.get())->setRunLength(
                    run_length + 1);
            }
        }
    }

    CompData::addEntry(std::move(pattern));
}

FPC::FPC(const Params &p)
  : DictionaryCompressor<uint32_t>(p), zeroRunSizeBits(p.zero_run_bits)
{
}

void
FPC::addToDictionary(const DictionaryEntry data)
{
    // There is no dictionary in FPC, so its size is zero, and no pattern
    // causes an insertion in the dictionary. The only reason we do not
    // assert it is that the UncompressedPattern implementation always
    // inserts by default
}

std::unique_ptr<DictionaryCompressor<uint32_t>::CompData>
FPC::instantiateDictionaryCompData() const
{
    return std::unique_ptr<DictionaryCompressor<uint32_t>::CompData>(
        new FPCCompData(zeroRunSizeBits));
}

} // namespace compression
} // namespace gem5
