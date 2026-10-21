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

#include "cpu/o3/lsq_occupancy.hh"

#include "base/logging.hh"

namespace gem5
{

namespace o3
{

ConstantOccupancy::ConstantOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), entries(p.entries)
{}

unsigned
ConstantOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return entries;
}

PerSegmentOccupancy::PerSegmentOccupancy(const Params &p)
    : LSQOccupancyPolicy(p)
{}

unsigned
PerSegmentOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return info.segments.size();
}

PerBlockOccupancy::PerBlockOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), blockSize(p.blockSize), coalesce(p.coalesce)
{
    fatal_if(blockSize == 0, "%s: blockSize must not be zero.\n", name());
}

unsigned
PerBlockOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return coalesce ? distinctBlocks(info.segments, blockSize)
                    : sumBlocks(info.segments, blockSize);
}

PerCacheLineOccupancy::PerCacheLineOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), coalesce(p.coalesce)
{}

unsigned
PerCacheLineOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return coalesce ? distinctBlocks(info.segments, info.lineSize)
                    : sumBlocks(info.segments, info.lineSize);
}

} // namespace o3
} // namespace gem5
