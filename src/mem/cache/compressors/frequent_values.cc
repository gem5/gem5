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

#include "mem/cache/compressors/frequent_values.hh"

#include <algorithm>
#include <limits>

#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/CacheComp.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/FrequentValuesCompressor.hh"

namespace gem5
{

namespace compression
{

FrequentValues::FrequentValues(const Params &p)
    : Base(p),
      useHuffmanEncoding(p.max_code_length != 0),
      indexEncoder(p.max_code_length),
      counterBits(p.counter_bits),
      codeGenerationTicks(p.code_generation_ticks),
      checkSaturation(p.check_saturation),
      numVFTEntries(p.vft_entries),
      numSamples(p.num_samples),
      takenSamples(0),
      phase(SAMPLING),
      VFT((name() + ".VFT").c_str(), p.vft_entries, p.vft_assoc,
          p.vft_replacement_policy, p.vft_indexing_policy,
          VFTEntry(counterBits)),
      codeGenerationEvent([this] { phase = COMPRESSING; }, name())
{
    fatal_if((numVFTEntries - 1) > mask(chunkSizeBits),
             "There are more VFT entries than possible values.");
}

std::unique_ptr<Base::CompressionData>
FrequentValues::compress(const std::vector<Chunk> &chunks, Cycles &comp_lat,
                         Cycles &decomp_lat)
{
    std::unique_ptr<CompData> comp_data =
        std::unique_ptr<CompData>(new CompData());

    // Compression size
    std::size_t size = 0;

    // Compress every value sequentially. The compressed values are then
    // added to the final compressed data.
    for (const auto &chunk : chunks) {
        encoder::Code code;
        int length = 0;
        if (phase == COMPRESSING) {
            VFTEntry *entry = VFT.findEntry(chunk);

            // Theoretically, the code would be the index of the entry;
            // however, there is no practical need to do so, and we simply
            // use the value instead
            const unsigned uncompressed_index = uncompressedValue;
            const unsigned index = entry ? chunk : uncompressed_index;

            // If using an index encoder, apply it
            if (useHuffmanEncoding) {
                code = indexEncoder.encode(index);

                if (index == uncompressed_index) {
                    code.length += chunkSizeBits;
                } else if (code.length > 64) {
                    // If, for some reason, we could not generate an encoding
                    // for the value, generate the uncompressed encoding
                    code = indexEncoder.encode(uncompressed_index);
                    assert(code.length <= 64);
                    code.length += chunkSizeBits;
                }
            } else {
                const unsigned code_size = std::log2(numVFTEntries);
                if (entry) {
                    code = { index, code_size };
                } else {
                    code = { uncompressed_index, code_size + chunkSizeBits };
                }
            }
        } else {
            // Not compressing yet; simply copy the value over
            code = { chunk, chunkSizeBits };
        }
        length += code.length;

        DPRINTF(CacheComp,
                "Compressed %016x to %016x (Size = %d) "
                "(Phase: %d)\n",
                chunk, code.code, length, phase);

        comp_data->compressedValues.emplace_back(code, chunk);

        size += length;
    }

    // Set final compression size
    comp_data->setSizeBits(size);

    // Set latencies based on the degree of parallelization, and any extra
    // latencies due to shifting or packaging
    comp_lat = Cycles(compExtraLatency + (chunks.size() / compChunksPerCycle));
    decomp_lat =
        Cycles(decompExtraLatency + (chunks.size() / decompChunksPerCycle));

    // Return compressed line
    return comp_data;
}

void
FrequentValues::decompress(const CompressionData *comp_data, uint64_t *data)
{
    const CompData *casted_comp_data =
        static_cast<const CompData *>(comp_data);

    // Decompress every entry sequentially
    std::vector<Chunk> decomp_chunks;
    for (const auto &comp_chunk : casted_comp_data->compressedValues) {
        if (phase == COMPRESSING) {
            if (useHuffmanEncoding) {
                // Although in theory we have the codeword and have to find
                // its corresponding value, in order to make life easier we
                // search for the value and verify that the stored code
                // matches the table's
                [[maybe_unused]] const encoder::Code code =
                    indexEncoder.encode(comp_chunk.value);

                // Either the value will be found and the codes match, or the
                // value will not be found because it is an uncompressed entry
                assert(((code.length <= 64) &&
                        (code.code == comp_chunk.code.code)) ||
                       (comp_chunk.code.code ==
                        indexEncoder.encode(uncompressedValue).code));
            } else {
                // The value at the given VFT entry must match the one stored,
                // if it is not the uncompressed value
                assert((comp_chunk.code.code == uncompressedValue) ||
                       VFT.findEntry(comp_chunk.value));
            }
        }

        decomp_chunks.push_back(comp_chunk.value);
        DPRINTF(CacheComp, "Decompressed %016x to %016x\n",
                comp_chunk.code.code, comp_chunk.value);
    }

    // Concatenate the decompressed words to generate the cache lines
    fromChunks(decomp_chunks, data);
}

void
FrequentValues::sampleValues(const std::vector<uint64_t> &data,
                             bool is_invalidation)
{
    const std::vector<Chunk> chunks = toChunks(data.data());
    for (const Chunk &chunk : chunks) {
        VFTEntry *entry = VFT.findEntry(chunk);
        bool saturated = false;
        if (!is_invalidation) {
            // If a VFT hit, increase new value's counter; otherwise, insert
            // new value
            if (!entry) {
                entry = VFT.findVictim(chunk);
                assert(entry != nullptr);
                entry->value = chunk;
                VFT.insertEntry(chunk, entry);
            } else {
                VFT.accessEntry(entry);
            }
            entry->counter++;
            saturated = entry->counter.isSaturated();
        } else {
            // If a VFT hit, decrease value's counter
            if (entry) {
                VFT.accessEntry(entry);
                entry->counter--;
            }
        }

        // If any counter saturates, all counters are shifted right,
        // resulting in precision loss
        if (checkSaturation && saturated) {
            for (auto &entry : VFT) {
                entry.counter >>= 1;
            }
        }
    }

    takenSamples += chunks.size();
}

void
FrequentValues::generateCodes()
{
    // We need to find a pseudo value to store uncompressed values as
    // For that we generate all possible values from 0 to 1 size larger
    // than the number of real values.
    std::set<uint64_t> uncompressed_values;
    for (int i = 0; i < numVFTEntries + 1; ++i) {
        uncompressed_values.insert(uncompressed_values.end(), i);
    }

    for (const auto &entry : VFT) {
        // Remove the respective real value from the list of possible
        // pseudo values for the uncompressed value
        uncompressed_values.erase(entry.value);
    }

    // Select the first remaining possible value as the value
    // representing uncompressed values
    assert(uncompressed_values.size() >= 1);
    uncompressedValue = *uncompressed_values.begin();
    assert(VFT.findEntry(uncompressedValue) == nullptr);

    if (useHuffmanEncoding) {
        // Populate the queue, adding each entry as a tree with one node.
        // They are sorted such that the value with highest frequency is
        // the queue's top
        for (const auto &entry : VFT) {
            indexEncoder.sample(entry.value, entry.counter);
        }

        // Insert the uncompressed value in the tree assuming it has the
        // highest frequency, since it is in fact a group of all the values
        // not present in the VFT
        indexEncoder.sample(uncompressedValue, ULLONG_MAX);

        indexEncoder.generateCodeMaps();
    }

    // Generate the code map and mark the current phase as code generation
    phase = CODE_GENERATION;

    // Let us know when to change from the code generation phase to the
    // effective compression phase
    schedule(codeGenerationEvent, curTick() + codeGenerationTicks);
}

void
FrequentValues::probeNotify(const DataUpdate &data_update)
{
    // Do not update VFT if not sampling
    if (phase == SAMPLING) {
        // If the new data is not present, the notification is due to a
        // fill; otherwise, sample the old block's contents
        if (data_update.oldData.size() > 0) {
            sampleValues(data_update.oldData, true);
        }
        // If the new data is not present, the notification is due to an
        // invalidation; otherwise, sample the new block's contents
        if (data_update.newData.size() > 0) {
            sampleValues(data_update.newData, false);
        }

        // Check if it is done with the sampling phase. If so, generate the
        // codes that will be used for the compression phase
        if (takenSamples >= numSamples) {
            generateCodes();
        }
    }
}

void
FrequentValues::regProbeListeners()
{
    assert(listeners.empty());
    assert(cache != nullptr);
    listeners.push_back(new FrequentValuesListener(
        *this, cache->getProbeManager(), "Data Update"));
}

void
FrequentValues::FrequentValuesListener::notify(const DataUpdate &data_update)
{
    parent.probeNotify(data_update);
}

} // namespace compression
} // namespace gem5
