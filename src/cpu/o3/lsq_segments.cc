/*
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

#include "cpu/o3/lsq_segments.hh"

#include <algorithm>
#include <cassert>
#include <utility>

#include "base/intmath.hh"

namespace gem5
{

namespace o3
{

namespace
{

/** Returns the first and last block_size aligned block of a segment. */
std::pair<Addr, Addr>
blockRange(const MemSegment &segment, Addr block_size)
{
    assert(segment.size > 0);
    return {segment.vaddr / block_size,
            (segment.vaddr + segment.size - 1) / block_size};
}

} // anonymous namespace

void
appendSegment(std::vector<MemSegment> &segments, Addr vaddr, unsigned size,
              const std::vector<bool> &byte_enable)
{
    assert(byte_enable.empty() || byte_enable.size() == size);
    const unsigned active_bytes =
        byte_enable.empty()
            ? size
            : std::count(byte_enable.begin(), byte_enable.end(), true);
    if (active_bytes > 0) {
        segments.push_back({vaddr, size, active_bytes});
    }
}

unsigned
sumBlocks(std::span<const MemSegment> segments, Addr block_size)
{
    assert(block_size > 0);
    unsigned blocks = 0;
    for (const MemSegment &segment : segments) {
        const auto [first, last] = blockRange(segment, block_size);
        blocks += last - first + 1;
    }
    return blocks;
}

unsigned
distinctBlocks(std::span<const MemSegment> segments, Addr block_size)
{
    assert(block_size > 0);
    std::vector<std::pair<Addr, Addr>> ranges;
    ranges.reserve(segments.size());
    for (const MemSegment &segment : segments) {
        ranges.push_back(blockRange(segment, block_size));
    }
    std::sort(ranges.begin(), ranges.end());

    // Merge overlapping block ranges and count the blocks they cover.
    unsigned blocks = 0;
    auto it = ranges.begin();
    while (it != ranges.end()) {
        const Addr first = it->first;
        Addr last = it->second;
        for (++it; it != ranges.end() && it->first <= last; ++it) {
            last = std::max(last, it->second);
        }
        blocks += last - first + 1;
    }
    return blocks;
}

unsigned
crossingSegments(std::span<const MemSegment> segments, Addr block_size)
{
    assert(block_size > 0);
    unsigned crossing = 0;
    for (const MemSegment &segment : segments) {
        const auto [first, last] = blockRange(segment, block_size);
        if (first != last) {
            ++crossing;
        }
    }
    return crossing;
}

unsigned
totalActiveBytes(std::span<const MemSegment> segments)
{
    unsigned bytes = 0;
    for (const MemSegment &segment : segments) {
        bytes += segment.activeBytes;
    }
    return bytes;
}

unsigned
chunks(std::span<const MemSegment> segments, Addr chunk_size, bool coalesce)
{
    assert(chunk_size > 0);
    if (coalesce) {
        return divCeil(totalActiveBytes(segments), chunk_size);
    }

    unsigned count = 0;
    for (const MemSegment &segment : segments) {
        count += divCeil(segment.activeBytes, chunk_size);
    }
    return count;
}

} // namespace o3
} // namespace gem5
