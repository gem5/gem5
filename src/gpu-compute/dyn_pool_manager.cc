/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/dyn_pool_manager.hh"

namespace gem5
{

// return the min number of elements that the manager can reserve given
// a request for "size" elements
uint32_t
DynPoolManager::minAllocatedElements(uint32_t size)
{
    fatal_if(size <= 0 || size > poolSize(), "Illegal VGPR region size=%d\n",
             size);

    return size % minAllocation() > 0 ?
        (minAllocation() - (size % minAllocation())) + size : size;
}

std::string
DynPoolManager::printRegion()
{
    std::string _cout;
    uint32_t reservedEntries = 0;

    /*
      Iterate over all elements in freeSpaceRecord, checking first element
      of each pair to see how much space in it has been allocated already.
      This only counts the partially allocated regions.  Thus, in addition,
      count the elements in reservedSpaceRecord.
    */
    auto it_free = freeSpaceRecord.begin();
    while (it_free != freeSpaceRecord.end()) {
        reservedEntries += it_free->first;
        ++it_free;
    }
    reservedEntries += (reservedSpaceRecord * totalRegSpace);

    if (reservedEntries == 0)
        _cout = "VRF is empty\n";
    else {
        _cout = "VRF reserves " + std::to_string(reservedEntries) + " VGPRs\n";
    }
    return _cout;
}

// reset freeSpace and reservedSpace
void
DynPoolManager::resetRegion(const int & regsPerSimd){
    totalRegSpace = regsPerSimd;
    reservedSpaceRecord = 0;
    freeSpaceRecord.clear();

    // reset available free space
    _totRegSpaceAvailable = regsPerSimd;
    freeSpaceRecord.push_back(std::make_pair(0,regsPerSimd));
}

bool
DynPoolManager::canAllocate(uint32_t numRegions, uint32_t size)
{
    uint32_t actualSize = minAllocatedElements(size);
    DPRINTF(GPUVRF,"Can Allocate %d\n",actualSize);
    return (_totRegSpaceAvailable >= actualSize);
}

uint32_t
DynPoolManager::allocateRegion(const uint32_t size,
                                    uint32_t *reservedPoolSize)
{
    uint32_t startIdx = (unsigned)-1;
    uint32_t actualSize = minAllocatedElements(size);
    auto it = freeSpaceRecord.begin();
    while (it != freeSpaceRecord.end()) {
        if (it->second >= actualSize) {
            // assign the next block starting from here
            startIdx = it->first;
            _regionSize = actualSize;
            *reservedPoolSize = actualSize;
            _totRegSpaceAvailable -= actualSize;

            // This case sees if this chunk size is exactly equal to
            // the size of the requested chunk. If yes, then this can't
            // contribute to future requests and hence, should be removed
            if (it->second == actualSize) {
                it = freeSpaceRecord.erase(it);
                // once entire freeSpaceRecord allocated, increment
                // reservedSpaceRecord count
                ++reservedSpaceRecord;
            } else {
                it->first += actualSize;
                it->second -= actualSize;
            }
            break;
        }
        it++;
    }
    DPRINTF(GPUVRF,"totRegSpace %d allocating Register at %d and"
                " size %d\n",_totRegSpaceAvailable,startIdx,actualSize);
    return startIdx;
}

void
DynPoolManager::freeRegion(uint32_t firstIdx,
                                uint32_t lastIdx)
{
    // lastIdx-firstIdx should give the size of free space
    DPRINTF(GPUVRF,"freeing Region at %d %d, size %d\n",
                firstIdx,lastIdx,lastIdx-firstIdx);

    // Current dynamic register allocation does not handle wraparound
    assert(firstIdx < lastIdx);
    _totRegSpaceAvailable += lastIdx-firstIdx;
    freeSpaceRecord.push_back(std::make_pair(firstIdx,lastIdx-firstIdx));
    // remove corresponding entry from reservedSpaceRecord too
    --reservedSpaceRecord;
}

uint32_t
DynPoolManager::regionSize(std::pair<uint32_t, uint32_t> &region)
{
    bool wrapAround = (region.first > region.second);
    if (!wrapAround) {
        return region.second - region.first + 1;
    } else {
        return region.second + poolSize() - region.first + 1;
    }
}

} // namespace gem5
