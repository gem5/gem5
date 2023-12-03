/*
 * Copyright (c) 2011-2021 Advanced Micro Devices, Inc.
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
 */

#include "sim/mem_pool.hh"

#include "base/addr_range.hh"
#include "base/logging.hh"

namespace gem5
{

MemPool::MemPool(Addr page_shift, Addr ptr, Addr limit)
    : pageShift(page_shift),
      startPageNum(ptr >> page_shift),
      freePageNum(ptr >> page_shift),
      _totalPages((limit - ptr) >> page_shift)
{
    gem5_assert(_totalPages > 0);
}

Counter
MemPool::startPage() const
{
    return startPageNum;
}

Counter
MemPool::freePage() const
{
    return freePageNum;
}

void
MemPool::setFreePage(Counter value)
{
    freePageNum = value;
}

Addr
MemPool::freePageAddr() const
{
    return freePageNum << pageShift;
}

Counter
MemPool::totalPages() const
{
    return _totalPages;
}

Counter
MemPool::allocatedPages() const
{
    return freePageNum - startPageNum;
}

Counter
MemPool::freePages() const
{
    return _totalPages - allocatedPages();
}

Addr
MemPool::startAddr() const
{
    return startPage() << pageShift;
}

Addr
MemPool::allocatedBytes() const
{
    return allocatedPages() << pageShift;
}

Addr
MemPool::freeBytes() const
{
    return freePages() << pageShift;
}

Addr
MemPool::totalBytes() const
{
    return totalPages() << pageShift;
}

Addr
MemPool::allocate(Addr npages)
{
    Addr return_addr = freePageAddr();
    freePageNum += npages;

    fatal_if(freePages() <= 0,
             "Out of memory, please increase size of physical memory.");

    return return_addr;
}

void
MemPool::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "page_shift", pageShift);
    paramOut(cp, "start_page", startPageNum);
    paramOut(cp, "free_page_num", freePageNum);
    paramOut(cp, "total_pages", _totalPages);
}

void
MemPool::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "page_shift", pageShift);
    paramIn(cp, "start_page", startPageNum);
    paramIn(cp, "free_page_num", freePageNum);
    paramIn(cp, "total_pages", _totalPages);
}

void
MemPools::populate(const AddrRangeList &memories)
{
    for (const auto &mem : memories)
        pools.emplace_back(pageShift, mem.start(), mem.end());
}

Addr
MemPools::allocPhysPages(int npages, int pool_id)
{
    return pools[pool_id].allocate(npages);
}

Addr
MemPools::memSize(int pool_id) const
{
    return pools[pool_id].totalBytes();
}

Addr
MemPools::freeMemSize(int pool_id) const
{
    return pools[pool_id].freeBytes();
}

void
MemPools::serialize(CheckpointOut &cp) const
{
    ScopedCheckpointSection sec(cp, "mempools");
    int num_pools = pools.size();
    SERIALIZE_SCALAR(num_pools);

    for (int i = 0; i < num_pools; i++)
        pools[i].serializeSection(cp, csprintf("pool%d", i));
}

void
MemPools::unserialize(CheckpointIn &cp)
{
    // Delete previous mem_pools
    pools.clear();

    ScopedCheckpointSection sec(cp, "mempools");
    int num_pools = 0;
    UNSERIALIZE_SCALAR(num_pools);

    for (int i = 0; i < num_pools; i++) {
        MemPool pool;
        pool.unserializeSection(cp, csprintf("pool%d", i));
        pools.push_back(pool);
    }
}

} // namespace gem5
