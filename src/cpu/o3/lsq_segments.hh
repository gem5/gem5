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

#ifndef __CPU_O3_LSQ_SEGMENTS_HH__
#define __CPU_O3_LSQ_SEGMENTS_HH__

#include <span>
#include <vector>

#include "base/types.hh"
#include "cpu/op_class.hh"
#include "mem/request.hh"

namespace gem5
{

namespace o3
{

/** A range of contiguous bytes accessed by a memory request. */
struct MemSegment
{
    /** Virtual address of the first byte. */
    Addr vaddr;
    /** Number of bytes in the range. */
    unsigned size;
    /** Number of enabled bytes in the range. */
    unsigned activeBytes;
};

/** Description of a memory access given to LSQ occupancy policies. */
struct MemAccessInfo
{
    /** Accessed segments. Segments without enabled bytes are left out. */
    std::span<const MemSegment> segments;
    /** Cache line size of the system. */
    Addr lineSize;
    OpClass opClass;
    bool isLoad;
    bool isAtomic;
    Request::Flags flags;
};

/**
 * Appends the segment [vaddr, vaddr + size) to segments, unless none of
 * its bytes is enabled. An empty byte_enable enables every byte.
 */
void appendSegment(std::vector<MemSegment> &segments, Addr vaddr,
                   unsigned size, const std::vector<bool> &byte_enable);

/**
 * Returns the number of block_size aligned blocks touched by the
 * segments. A block touched by several segments is counted once per
 * segment.
 */
unsigned sumBlocks(std::span<const MemSegment> segments, Addr block_size);

/**
 * Returns the number of distinct block_size aligned blocks touched by the
 * segments.
 */
unsigned distinctBlocks(std::span<const MemSegment> segments, Addr block_size);

/**
 * Returns the number of segments that touch more than one block_size
 * aligned block.
 */
unsigned crossingSegments(std::span<const MemSegment> segments,
                          Addr block_size);

/** Returns the number of enabled bytes of the segments. */
unsigned totalActiveBytes(std::span<const MemSegment> segments);

/**
 * Returns the number of chunk_size chunks needed to hold the enabled bytes
 * of the segments. If coalesce is false, the enabled bytes of each segment
 * are rounded up to whole chunks separately.
 */
unsigned chunks(std::span<const MemSegment> segments, Addr chunk_size,
                bool coalesce);

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_LSQ_SEGMENTS_HH__
