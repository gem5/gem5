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

#ifndef __MEM_CACHE_COMPRESSORS_FREQUENT_VALUES_HH__
#define __MEM_CACHE_COMPRESSORS_FREQUENT_VALUES_HH__

#include <climits>
#include <cstdint>
#include <memory>
#include <vector>

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_probe_arg.hh"
#include "mem/cache/compressors/base.hh"
#include "mem/cache/compressors/encoders/huffman.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

struct FrequentValuesCompressorParams;

namespace compression
{

/**
 * This compressor samples the cache for a while, trying to define the
 * most frequently used values. When these values are determined, they are
 * associated to shorter representations (codes). Then the compressor can
 * start its effective compression phase, in which occurrences of these
 * values are substituted by their codes.
 */
class FrequentValues : public Base
{
  private:
    class CompData;

    using DataUpdate = CacheDataUpdateProbeArg;

    class FrequentValuesListener : public ProbeListenerArgBase<DataUpdate>
    {
      protected:
        FrequentValues &parent;

      public:
        FrequentValuesListener(FrequentValues &_parent, ProbeManager *pm,
                               const std::string &name)
            : ProbeListenerArgBase(pm, name), parent(_parent)
        {}

        void notify(const DataUpdate &data_update) override;
    };

    std::vector<FrequentValuesListener *> listeners;

    /** Whether Huffman encoding is applied to the VFT indices. */
    const bool useHuffmanEncoding;

    /** The encoder applied to the VFT indices. */
    encoder::Huffman indexEncoder;

    /** Number of bits in the saturating counters. */
    const int counterBits;

    /** Ticks needed to perform the CODE_GENERATION phase. */
    const Tick codeGenerationTicks;

    /** Whether an action must be performed when counters saturate. */
    const bool checkSaturation;

    /** Maximum number of VFT entries, and thus of codewords too. */
    const unsigned numVFTEntries;

    /** Number of samples in the sampling phase. */
    const unsigned numSamples;

    /** Number of samples taken so far. */
    unsigned takenSamples;

    /**
     * The phase that the compressor is at. It assumes that sampling and
     * code generation are done only once.
     */
    enum Phase
    {
        SAMPLING,
        CODE_GENERATION,
        COMPRESSING
    };

    Phase phase;

    class VFTEntry : public TaggedEntry
    {
      public:
        /**
         * The value is stored as a 64 bit entry to accomodate for the
         * uncompressed value. All real values must be 32 bits.
         */
        uint64_t value;

        /**
         * The ideal counter width (in bits) is determined by the maximum
         * number of times a given value appears in the cache
         * (log2(cache_size / chunkSizeBits)). If smaller counters are used
         * their values should be rescaled when saturated.
         */
        SatCounter32 counter;

        VFTEntry(std::size_t num_bits)
            : TaggedEntry(), value(0), counter(num_bits)
        {}

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            value = 0;
            counter.reset();
        }
    };

    /**
     * The Value Frequency Table, a small cache that keeps track and estimates
     * the frequency distribution of values in the cache.
     */
    AssociativeSet<VFTEntry> VFT;

    /**
     * A pseudo value is used as the representation of uncompressed values.
     * This value is a random value that is not present in the VFT. It is
     * selected at the end of the sampling phase.
     */
    uint64_t uncompressedValue;

    /** Event to handle finishing code generation and starting compression. */
    EventFunctionWrapper codeGenerationEvent;

    /**
     * Sample values from a packet, adding them to the VFT.
     *
     * @param data The line being sampled.
     * @param is_invalidation whether this event comes from an invalidation.
     */
    void sampleValues(const std::vector<uint64_t> &data, bool is_invalidation);

    /** End sampling phase and start the code generation. */
    void generateCodes();

    std::unique_ptr<Base::CompressionData>
    compress(const std::vector<Chunk> &chunks, Cycles &comp_lat,
             Cycles &decomp_lat) override;

    void decompress(const CompressionData *comp_data, uint64_t *data) override;

  public:
    typedef FrequentValuesCompressorParams Params;
    FrequentValues(const Params &p);
    ~FrequentValues() = default;

    /**
     * Process a notification event from the ProbeListener.
     *
     * @param data_update The data regarding the entry's contents update.
     */
    void probeNotify(const DataUpdate &data_update);

    void regProbeListeners() override;
};

class FrequentValues::CompData : public CompressionData
{
  public:
    /**
     * A compressed value contains its encoding, and the compressed data
     * itself.
     */
    struct CompressedValue
    {
        /** The codeword.*/
        encoder::Code code;

        /**
         * Original value, stored both for when the codeword marks an
         * uncompressed entry, and to verify correctness.
         */
        uint64_t value;

        CompressedValue(encoder::Code _code, uint64_t _value)
            : code(_code), value(_value)
        {}
    };

    /**
     * The values contained in the original data, after being compressed
     * sequentially.
     */
    std::vector<CompressedValue> compressedValues;
};

} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_FREQUENT_VALUES_HH__
