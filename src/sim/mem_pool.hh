/*
 * Copyright (c) 2011-2021 Advanced Micro Devices, Inc.
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

#ifndef __MEM_POOL_HH__
#define __MEM_POOL_HH__

#include <vector>

#include "base/addr_range.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

/** Class for handling allocation of physical pages in SE mode. */
class MemPool : public Serializable
{
  private:
    Addr pageShift = 0;

    /** Start page of pool. */
    Counter startPageNum = 0;

    /** Page number of free memory. */
    Counter freePageNum = 0;

    /** The size of the pool, in number of pages. */
    Counter _totalPages = 0;

    MemPool() {}

    friend class MemPools;

  public:
    MemPool(Addr page_shift, Addr ptr, Addr limit);

    Counter startPage() const;
    Counter freePage() const;
    void setFreePage(Counter value);
    Addr freePageAddr() const;
    Counter totalPages() const;

    Counter allocatedPages() const;
    Counter freePages() const;

    Addr startAddr() const;
    Addr allocatedBytes() const;
    Addr freeBytes() const;
    Addr totalBytes() const;

    Addr allocate(Addr npages);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class MemPools : public Serializable
{
  private:
    Addr pageShift;

    std::vector<MemPool> pools;

  public:
    MemPools(Addr page_shift) : pageShift(page_shift) {}

    void populate(const AddrRangeList &memories);

    /// Allocate npages contiguous unused physical pages.
    /// @return Starting address of first page
    Addr allocPhysPages(int npages, int pool_id=0);

    /** Amount of physical memory that exists in a pool. */
    Addr memSize(int pool_id=0) const;

    /** Amount of physical memory that is still free in a pool. */
    Addr freeMemSize(int pool_id=0) const;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __MEM_POOL_HH__
