/*
 * Copyright (c) 2026 Barcelona Supercomputing Center
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

#include "cpu/o3/lsq_segments.hh"

#include <algorithm>
#include <cassert>
#include <utility>

namespace gem5
{

namespace o3
{

namespace
{

/**
 * Calls fn(first, last) for each run of consecutive block_size aligned
 * blocks that hold an enabled byte of the segment, in address order.
 */
template <class Fn>
void
forEachBlockRun(const MemSegment &segment, Addr block_size, Fn &&fn)
{
    assert(segment.size > 0);
    if (segment.byteEnable.empty()) {
        fn(segment.vaddr / block_size,
           (segment.vaddr + segment.size - 1) / block_size);
        return;
    }

    assert(segment.byteEnable.size() == segment.size);
    bool in_run = false;
    Addr first = 0;
    Addr last = 0;
    for (unsigned i = 0; i < segment.size; i++) {
        if (!segment.byteEnable[i]) {
            continue;
        }
        const Addr block = (segment.vaddr + i) / block_size;
        if (in_run && block - last <= 1) {
            last = block;
            continue;
        }
        if (in_run) {
            fn(first, last);
        }
        first = last = block;
        in_run = true;
    }
    if (in_run) {
        fn(first, last);
    }
}

/** Returns the number of block_size aligned blocks a segment touches. */
unsigned
touchedBlocks(const MemSegment &segment, Addr block_size)
{
    unsigned blocks = 0;
    forEachBlockRun(segment, block_size, [&blocks](Addr first, Addr last) {
        blocks += last - first + 1;
    });
    return blocks;
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
    if (active_bytes == 0) {
        return;
    }
    // The byte enables only matter if some byte is disabled.
    segments.push_back({vaddr, size, active_bytes,
                        active_bytes < size ? byte_enable
                                            : std::vector<bool>()});
}

unsigned
sumBlocks(std::span<const MemSegment> segments, Addr block_size)
{
    assert(block_size > 0);
    unsigned blocks = 0;
    for (const MemSegment &segment : segments) {
        blocks += touchedBlocks(segment, block_size);
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
        forEachBlockRun(segment, block_size, [&ranges](Addr first, Addr last) {
            ranges.emplace_back(first, last);
        });
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

} // namespace o3
} // namespace gem5
