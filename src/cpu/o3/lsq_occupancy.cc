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

#include <algorithm>

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

PerChunkOccupancy::PerChunkOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), chunkSize(p.chunkSize), coalesce(p.coalesce)
{
    fatal_if(chunkSize == 0, "%s: chunkSize must not be zero.\n", name());
}

unsigned
PerChunkOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return chunks(info.segments, chunkSize, coalesce);
}

LineCrossOccupancy::LineCrossOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), entries(p.entries), extraEntries(p.extraEntries)
{}

unsigned
LineCrossOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return entries +
           extraEntries * crossingSegments(info.segments, info.lineSize);
}

MaxOccupancy::MaxOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), policies(p.policies)
{
    fatal_if(policies.empty(), "%s: policies must not be empty.\n", name());
}

unsigned
MaxOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    unsigned entries = 0;
    for (const LSQOccupancyPolicy *policy : policies) {
        entries = std::max(entries, policy->occupiedEntries(info));
    }
    return entries;
}

SumOccupancy::SumOccupancy(const Params &p)
    : LSQOccupancyPolicy(p), policies(p.policies)
{
    fatal_if(policies.empty(), "%s: policies must not be empty.\n", name());
}

unsigned
SumOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    unsigned entries = 0;
    for (const LSQOccupancyPolicy *policy : policies) {
        entries += policy->occupiedEntries(info);
    }
    return entries;
}

ClampOccupancy::ClampOccupancy(const Params &p)
    : LSQOccupancyPolicy(p),
      policy(p.policy),
      minEntries(p.minEntries),
      maxEntries(p.maxEntries)
{
    fatal_if(!policy, "%s: policy must be set.\n", name());
    fatal_if(minEntries > maxEntries,
             "%s: minEntries (%u) must not exceed maxEntries (%u).\n", name(),
             minEntries, maxEntries);
}

unsigned
ClampOccupancy::occupiedEntries(const MemAccessInfo &info) const
{
    return std::clamp(policy->occupiedEntries(info), minEntries, maxEntries);
}

} // namespace o3
} // namespace gem5
