/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __DYN_POOL_MANAGER_HH__
#define __DYN_POOL_MANAGER_HH__

#include <cassert>
#include <cstdint>

#include "gpu-compute/pool_manager.hh"
#include "params/DynPoolManager.hh"

namespace gem5
{

// Dynamic Pool Manager: allows multiple WGs on the same pool
class DynPoolManager : public PoolManager
{
  public:
    DynPoolManager(const PoolManagerParams &p)
        : PoolManager(p), _regionSize(0)
    {
        _totRegSpaceAvailable = p.pool_size;
    }

    uint32_t allocateRegion(const uint32_t size, uint32_t *reservedPoolSize) override;
    bool canAllocate(uint32_t numRegions, uint32_t size) override;
    void freeRegion(uint32_t firstIdx, uint32_t lastIdx) override;
    uint32_t minAllocatedElements(uint32_t size);
    std::string printRegion() override;
    uint32_t regionSize(std::pair<uint32_t,uint32_t> &region) override;
    void resetRegion(const int & regsPerSimd) override;

  private:
    // actual size of a region (normalized to the minimum size that can
    // be reserved)
    uint32_t _regionSize;
    // total registers available - across chunks
    uint32_t _totRegSpaceAvailable;
    // regIndex and freeSpace record
    std::list<std::pair<int,int>> freeSpaceRecord;
    int reservedSpaceRecord;
    // total registers to be allocated -- treat as a const
    int totalRegSpace;
};

} // namespace gem5

#endif // __DYN_POOL_MANAGER_HH__
