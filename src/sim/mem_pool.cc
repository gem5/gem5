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

#include "sim/mem_pool.hh"

#include "sim/system.hh"

namespace gem5
{

MemPool::MemPool(System *system, Addr ptr, Addr limit)
        : sys(system), freePageNum(ptr >> sys->getPageShift()),
        _totalPages(limit >> sys->getPageShift())
{
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
    return freePageNum << sys->getPageShift();
}

Counter
MemPool::totalPages() const
{
    return _totalPages;
}

Counter
MemPool::allocatedPages() const
{
    return freePageNum;
}

Counter
MemPool::freePages() const
{
    return _totalPages - freePageNum;
}

Addr
MemPool::allocatedBytes() const
{
    return allocatedPages() << sys->getPageShift();
}

Addr
MemPool::freeBytes() const
{
    return freePages() << sys->getPageShift();
}

Addr
MemPool::totalBytes() const
{
    return totalPages() << sys->getPageShift();
}

Addr
MemPool::allocate(Addr npages)
{
    Addr return_addr = freePageAddr();
    freePageNum += npages;
    Addr next_return_addr = freePageAddr();

    if (sys->m5opRange().valid() &&
        next_return_addr >= sys->m5opRange().start()) {
        warn("Reached m5ops MMIO region\n");
        return_addr = sys->m5opRange().end();
        freePageNum = (return_addr >> sys->getPageShift()) + npages;
    }

    fatal_if((freePages() <= 0), "Out of memory, "
             "please increase size of physical memory.");

    return return_addr;
}

} // namespace gem5
