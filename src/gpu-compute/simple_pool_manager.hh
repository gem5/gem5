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
 */

#ifndef __SIMPLE_POOL_MANAGER_HH__
#define __SIMPLE_POOL_MANAGER_HH__

#include <cassert>
#include <cstdint>

#include "gpu-compute/pool_manager.hh"
#include "params/SimplePoolManager.hh"

// Simple Pool Manager: allows one region per pool. No region merging is
// supported.
class SimplePoolManager : public PoolManager
{
  public:
    SimplePoolManager(const PoolManagerParams *p)
        : PoolManager(p), _regionSize(0), _nxtFreeIdx(0),
          _reservedGroups(0)
    {
    }

    uint32_t minAllocatedElements(uint32_t size);
    std::string printRegion();
    bool canAllocate(uint32_t numRegions, uint32_t size);
    uint32_t allocateRegion(const uint32_t size, uint32_t *reservedPoolSize);
    void freeRegion(uint32_t firstIdx, uint32_t lastIdx);
    uint32_t regionSize(std::pair<uint32_t,uint32_t> &region);

  private:
    // actual size of a region (normalized to the minimum size that can
    // be reserved)
    uint32_t _regionSize;
    // next index to allocate a region
    int _nxtFreeIdx;
    // number of groups that reserve a region
    uint32_t _reservedGroups;
};

#endif // __SIMPLE_POOL_MANAGER_HH__
