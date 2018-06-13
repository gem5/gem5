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
 * Implementation of the BDI cache compressor.
 */

#include "mem/cache/compressors/bdi.hh"

#include <algorithm>
#include <climits>
#include <cstring>
#include <type_traits>

#include "debug/CacheComp.hh"
#include "params/BDI.hh"

// Number of bytes in a qword
#define BYTES_PER_QWORD 8

// Declare BDI encoding names
const char* BDI::ENCODING_NAMES[] =
    {"Zero", "Repeated_Values", "Base8_1", "Base8_2", "Base8_4", "Base4_1",
     "Base4_2", "Base2_1", "Uncompressed"};

BDI::BDICompData::BDICompData(const uint8_t encoding)
    : CompressionData(), _encoding(encoding)
{
}

uint8_t
BDI::BDICompData::getEncoding() const
{
    return _encoding;
}

std::string
BDI::BDICompData::getName() const
{
    return ENCODING_NAMES[_encoding];
}

BDI::BDICompDataZeros::BDICompDataZeros()
    : BDICompData(ZERO)
{
    // Calculate compressed size
    calculateCompressedSize();
}

uint64_t
BDI::BDICompDataZeros::access(const int index) const
{
    return 0;
}

void
BDI::BDICompDataZeros::calculateCompressedSize()
{
    // Number of bits used by Encoding
    std::size_t size = encodingBits;

    setSizeBits(size);
}

BDI::BDICompDataRep::BDICompDataRep(const uint64_t rep_value)
    : BDICompData(REP_VALUES)
{
    // Set base value
    base = rep_value;

    // Calculate compressed size
    calculateCompressedSize();
}

uint64_t
BDI::BDICompDataRep::access(const int index) const
{
    return base;
}

void
BDI::BDICompDataRep::calculateCompressedSize()
{
    // Number of bits used by Encoding
    std::size_t size = encodingBits;

    // Number of bits used by Base
    size += sizeof(base)*CHAR_BIT;

    setSizeBits(size);
}

BDI::BDICompDataUncompressed::BDICompDataUncompressed(
    const uint64_t* data, const std::size_t blk_size)
    : BDICompData(UNCOMPRESSED), blkSize(blk_size),
      _data(data, data + blk_size/CHAR_BIT)
{
    // Calculate compressed size
    calculateCompressedSize();
}

uint64_t
BDI::BDICompDataUncompressed::access(const int index) const
{
    return _data[index];
}

void
BDI::BDICompDataUncompressed::calculateCompressedSize()
{
    // Number of bits used by Encoding
    std::size_t size = encodingBits;

    // Number of bits used by uncompressed line
    size += blkSize*CHAR_BIT;

    setSizeBits(size);
}

template <class TB, class TD>
BDI::BDICompDataBaseDelta<TB, TD>::BDICompDataBaseDelta(const uint8_t encoding,
    const std::size_t blk_size, const std::size_t max_num_bases)
    : BDICompData(encoding), maxNumBases(max_num_bases)
{
    // Reserve the maximum possible size for the vectors
    bases.reserve(maxNumBases);
    bitMask.reserve(blk_size/sizeof(TD));
    deltas.reserve(blk_size/sizeof(TD));

    // Push virtual base 0 to bases list
    bases.push_back(0);
}

template <class TB, class TD>
void
BDI::BDICompDataBaseDelta<TB, TD>::calculateCompressedSize()
{
    // Number of bits used by Encoding
    std::size_t size = encodingBits;

    // Number of bits used by BitMask
    size += bitMask.size()*std::ceil(std::log2(bases.size()));

    // Number of bits used by Bases. bases[0] is implicit in a hardware
    // implementation, therefore its size is 0
    size += (maxNumBases-1)*sizeof(TB)*CHAR_BIT;

    // Number of bits used by Deltas
    size += deltas.size()*sizeof(TD)*CHAR_BIT;

    CompressionData::setSizeBits(size);
}

template <class TB, class TD>
bool
BDI::BDICompDataBaseDelta<TB, TD>::addBase(const TB base)
{
    // Can't add base if reached limit of number of bases
    if (bases.size() >= maxNumBases) {
        return false;
    }

    // Push new base to end of bases list
    bases.push_back(base);

    // New delta is 0, as it is a difference between the new base and itself
    addDelta(bases.size() - 1, 0);

    return true;
}

template <class TB, class TD>
void
BDI::BDICompDataBaseDelta<TB, TD>::addDelta(const std::size_t base_index,
    const TD delta)
{
    // Insert new delta with respect to the given base
    bitMask.push_back(base_index);

    // Insert new delta
    deltas.push_back(delta);
}

template <class TB, class TD> bool
BDI::BDICompDataBaseDelta<TB, TD>::compress(const uint64_t* data,
    const std::size_t blk_size)
{
    // Parse through data in a sizeof(TB) granularity
    for (std::size_t byte_start = 0; byte_start < blk_size;
         byte_start += sizeof(TB))
    {
        // Get current value
        TB curValue;
        std::memcpy(&curValue, ((uint8_t*)data) + byte_start,
                    sizeof(TB));

        // Iterate through all bases to search for a valid delta
        bool foundDelta = false;
        for (int base_index = 0; base_index < bases.size(); base_index++) {
            // Calculate delta relative to currently parsed base
            typename std::make_signed<TB>::type delta = curValue -
                                                        bases[base_index];

            // Check if the delta is within the limits of the delta size. If
            // that is the case, add delta to compressed data and keep parsing
            // the input data
            typename std::make_signed<TB>::type limit =
                ULLONG_MAX>>((BYTES_PER_QWORD-sizeof(TD))*CHAR_BIT+1);
            if ((delta >= -limit) && (delta <= limit)) {
                addDelta(base_index, delta);
                foundDelta = true;
                break;
            }
        }

        // If we cannot find a base that can accommodate this new line's data,
        // add this value as the new base and insert its respective delta of 0.
        // If the new base can't be added, it means that we reached the base
        // limit, so line is uncompressible using the given encoding
        if (!foundDelta && !addBase(curValue)) {
            return false;
        }
    }

    // Calculate compressed size
    calculateCompressedSize();

    return true;
}

template <class TB, class TD>
uint64_t
BDI::BDICompDataBaseDelta<TB, TD>::access(const int index) const
{
    // We decompress all base-delta pairs that form the 64-bit entry
    // corresponding to the given 64-bit-array index
    uint64_t value = 0;

    // Get relationship between the size of an uint64_t base and size of TB
    const std::size_t size_diff = sizeof(uint64_t)/sizeof(TB);

    // Mask for a base entry
    const uint64_t mask = ULLONG_MAX>>((BYTES_PER_QWORD-sizeof(TB))*CHAR_BIT);

    // Size, in bits, of a base entry. Cant be const because compiler will
    // optimize and create a 64 bit instance, which will generate a shift size
    // compilation error
    int base_size_bits = sizeof(TB)*CHAR_BIT;

    // Concatenate all base-delta entries until they form a 64-bit entry
    for (int delta_index = size_diff * (index + 1) - 1;
         delta_index >= (int)(size_diff * index); delta_index--) {
        // Get base and delta entries corresponding to the current delta
        assert(delta_index < deltas.size());
        const TD delta = deltas[delta_index];
        const int base_index = bitMask[delta_index];
        assert(base_index < bases.size());
        const TB base = bases[base_index];

        // Concatenate decompressed value
        value <<= base_size_bits;
        value |= static_cast<uint64_t>((base + delta) & mask);
    }

    return value;
}

BDI::BDI(const Params *p)
    : BaseCacheCompressor(p), useMoreCompressors(p->use_more_compressors),
      qwordsPerCacheLine(blkSize/BYTES_PER_QWORD)
{
    static_assert(sizeof(ENCODING_NAMES)/sizeof(char*) == NUM_ENCODINGS,
                  "Number of encodings doesn't match the number of names");
}

bool
BDI::isZeroPackable(const uint64_t* data) const
{
    return std::all_of(data, data + qwordsPerCacheLine,
                       [](const uint64_t entry){ return entry == 0; });
}

bool
BDI::isSameValuePackable(const uint64_t* data) const
{
    // We don't want to copy the whole array to the lambda expression
    const uint64_t rep_value = data[0];
    return std::all_of(data, data + qwordsPerCacheLine,
                       [rep_value](const uint64_t entry)
                           {return entry == rep_value;});
}

template <class TB, class TD>
std::unique_ptr<BDI::BDICompData>
BDI::tryCompress(const uint64_t* data, const uint8_t encoding) const
{
    // Instantiate compressor
    auto temp_data = std::unique_ptr<BDICompDataBaseDelta<TB, TD>>(
        new BDICompDataBaseDelta<TB, TD>(encoding, blkSize));

    // Try compressing. Return nullptr if compressor can't compress given line
    if (temp_data->compress(data, blkSize)) {
        return std::move(temp_data);
    } else {
        return std::unique_ptr<BDICompData>{};
    }
}

void
BDI::decompress(const BaseCacheCompressor::CompressionData* comp_data,
                uint64_t* data)
{
    // Decompress and go back to host endianness
    for (std::size_t i = 0; i < qwordsPerCacheLine; i++)
        data[i] = static_cast<const BDICompData*>(comp_data)->access(i);
}

std::unique_ptr<BaseCacheCompressor::CompressionData>
BDI::compress(const uint64_t* data, Cycles& comp_lat, Cycles& decomp_lat)
{
    std::unique_ptr<BDICompData> bdi_data;

    // Check if it is a zero line
    if (isZeroPackable(data)) {
        bdi_data = std::unique_ptr<BDICompData>(new BDICompDataZeros());

        // Set compression latency (compare 1 qword per cycle)
        comp_lat = Cycles(qwordsPerCacheLine);
    // Check if all values in the line are the same
    } else if (isSameValuePackable(data)) {
        bdi_data = std::unique_ptr<BDICompData>(new BDICompDataRep(data[0]));

        // Set compression latency (compare 1 qword per cycle)
        comp_lat = Cycles(qwordsPerCacheLine);
    } else {
        // Initialize compressed data as an uncompressed instance
        bdi_data = std::unique_ptr<BDICompData>(
                  new BDICompDataUncompressed(data, blkSize));

        // Base size-delta size ratio. Used to optimize run and try to
        // compress less combinations when their size is worse than the
        // current best. It does not reflect the compression ratio, as
        // it does not take the metadata into account.
        int base_delta_ratio = 2;

        // Check which base-delta size combination is the best. This is
        // optimized by giving priority to trying the compressor that would
        // generate the smallest compression size. This way we waste less
        // simulation time by not doing all possible combinations
        for (int ratio = 8; ratio >= base_delta_ratio; ratio/=2) {
            for (int base_size = 8; base_size >= ratio; base_size/=2) {
                // If using more compressors, parse all delta sizes from 1 to
                // one size smaller than the base size, otherwise just parse
                // highest possible delta. When we only instantiate one delta
                // size per base size, we use less area and energy, at the
                // cost of lower compression efficiency
                const int delta_size = base_size/ratio;
                if (!useMoreCompressors && delta_size != base_size/2) {
                    continue;
                }

                std::unique_ptr<BDICompData> temp_bdi_data;

                // Get the compression result for the current combination
                if ((base_size == 8)&&(delta_size == 4)) {
                    temp_bdi_data = tryCompress<uint64_t, int32_t>(data,
                                                                   BASE8_4);
                } else if ((base_size == 8)&&(delta_size == 2)) {
                    temp_bdi_data = tryCompress<uint64_t, int16_t>(data,
                                                                   BASE8_2);
                } else if ((base_size == 8)&&(delta_size == 1)) {
                    temp_bdi_data = tryCompress<uint64_t, int8_t>(data,
                                                                  BASE8_1);
                } else if ((base_size == 4)&&(delta_size == 2)) {
                    temp_bdi_data = tryCompress<uint32_t, int16_t>(data,
                                                                   BASE4_2);
                } else if ((base_size == 4)&&(delta_size == 1)) {
                    temp_bdi_data = tryCompress<uint32_t, int8_t>(data,
                                                                  BASE4_1);
                } else if ((base_size == 2)&&(delta_size == 1)) {
                    temp_bdi_data = tryCompress<uint16_t, int8_t>(data,
                                                                  BASE2_1);
                } else {
                    fatal("Invalid combination of base and delta sizes.");
                }

                // If compressor was successful, check if new compression
                // improves the compression factor
                if (temp_bdi_data &&
                    (bdi_data->getSizeBits() > temp_bdi_data->getSizeBits()))
                {
                    bdi_data = std::move(temp_bdi_data);
                    base_delta_ratio = ratio;
                }

                // Clear temp pointer
                temp_bdi_data.reset(nullptr);
            }
        }

        // Set compression latency. A successful compressor will stop all
        // other compressors when it knows no other will generate a better
        // compression. However, this requires either hard-coding, or a complex
        // function that can calculate the exact size of every compressor for
        // every cache line size. We decide to take a conservative
        // optimization: assume that all compressors with a given base size
        // delta size ratio (no-metadata ratio) must wait for each other.
        // In the worst case scenario the data is left uncompressed, so
        // it loses the time of the worst base size (2 bytes per cycle)
        comp_lat = Cycles(blkSize/base_delta_ratio);
    }

    // Update stats
    encodingStats[bdi_data->getEncoding()]++;

    // Pack compression results (1 extra cycle)
    comp_lat += Cycles(1);

    // Set decompression latency (latency of an adder)
    decomp_lat = Cycles(1);

    // Print debug information
    DPRINTF(CacheComp, "BDI: Compressed cache line to encoding %s (%d bits)\n",
            bdi_data->getName(), bdi_data->getSizeBits());

    return std::move(bdi_data);
}

void
BDI::regStats()
{
    BaseCacheCompressor::regStats();

    // We store the frequency of each encoding
    encodingStats
        .init(NUM_ENCODINGS)
        .name(name() + ".encoding")
        .desc("Number of data entries that were compressed to this encoding.")
        ;

    for (unsigned i = 0; i < NUM_ENCODINGS; ++i) {
        encodingStats.subname(i, ENCODING_NAMES[i]);
        encodingStats.subdesc(i, "Number of data entries that match " \
                                 "encoding " + std::string(ENCODING_NAMES[i]));
    }
}

BDI*
BDIParams::create()
{
    return new BDI(this);
}

#undef BYTES_PER_QWORD
