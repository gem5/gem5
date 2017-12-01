/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: John Kalamatianos
 */

#include "gpu-compute/simple_pool_manager.hh"

#include "base/logging.hh"

// return the min number of elements that the manager can reserve given
// a request for "size" elements
uint32_t
SimplePoolManager::minAllocatedElements(uint32_t size)
{
    fatal_if(size <= 0 || size > poolSize(), "Illegal VGPR region size=%d\n",
             size);

    return size % minAllocation() > 0 ?
        (minAllocation() - (size % minAllocation())) + size : size;
}

std::string
SimplePoolManager::printRegion()
{
    std::string _cout;
    if (_reservedGroups == 0)
        _cout = "VRF is empty\n";
    else if (_reservedGroups > 0) {
        uint32_t reservedEntries = _reservedGroups * _regionSize;
        _cout = "VRF reserves " + std::to_string(reservedEntries) + " VGPRs\n";
    }

    return _cout;
}

bool
SimplePoolManager::canAllocate(uint32_t numRegions, uint32_t size)
{
    assert(numRegions * minAllocatedElements(size) <= poolSize());

    return _reservedGroups == 0;
}

void
SimplePoolManager::freeRegion(uint32_t firstIdx, uint32_t lastIdx)
{
    assert(_reservedGroups > 0);
    --_reservedGroups;

    if (!_reservedGroups)
        _nxtFreeIdx = 0;
}

uint32_t
SimplePoolManager::allocateRegion(const uint32_t size,
                                  uint32_t *reservedPoolSize)
{
    uint32_t actualSize = minAllocatedElements(size);
    uint32_t startIdx = _nxtFreeIdx;
    _nxtFreeIdx += actualSize;
    _regionSize = actualSize;
    assert(_nxtFreeIdx < poolSize());
    *reservedPoolSize = actualSize;
    ++_reservedGroups;

    return startIdx;
}

uint32_t
SimplePoolManager::regionSize(std::pair<uint32_t, uint32_t> &region)
{
    bool wrapAround = (region.first > region.second);
    if (!wrapAround) {
        return region.second - region.first + 1;
    } else {
        return region.second + poolSize() - region.first + 1;
    }
}
