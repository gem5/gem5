/*
 * Copyright (c) 2017-2020 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
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

#include "sim/mem_state.hh"

#include <cassert>

#include "arch/generic/tlb.hh"
#include "debug/Vma.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/process.hh"
#include "sim/syscall_debug_macros.hh"
#include "sim/system.hh"
#include "sim/vma.hh"

MemState::MemState(Process *owner, Addr brk_point, Addr stack_base,
                   Addr max_stack_size, Addr next_thread_stack_base,
                   Addr mmap_end)
    : _ownerProcess(owner),
      _pageBytes(owner->system->getPageBytes()), _brkPoint(brk_point),
      _stackBase(stack_base), _stackSize(max_stack_size),
      _maxStackSize(max_stack_size), _stackMin(stack_base - max_stack_size),
      _nextThreadStackBase(next_thread_stack_base),
      _mmapEnd(mmap_end), _endBrkPoint(brk_point)
{
}

MemState&
MemState::operator=(const MemState &in)
{
    if (this == &in)
        return *this;

    _pageBytes = in._pageBytes;
    _brkPoint = in._brkPoint;
    _stackBase = in._stackBase;
    _stackSize = in._stackSize;
    _maxStackSize = in._maxStackSize;
    _stackMin = in._stackMin;
    _nextThreadStackBase = in._nextThreadStackBase;
    _mmapEnd = in._mmapEnd;
    _endBrkPoint = in._endBrkPoint;
    _vmaList = in._vmaList; /* This assignment does a deep copy. */

    return *this;
}

void
MemState::resetOwner(Process *owner)
{
    _ownerProcess = owner;
}

bool
MemState::isUnmapped(Addr start_addr, Addr length)
{
    Addr end_addr = start_addr + length;
    const AddrRange range(start_addr, end_addr);
    for (const auto &vma : _vmaList) {
        if (vma.intersects(range))
            return false;
    }

    /**
     * In case someone skips the VMA interface and just directly maps memory
     * also consult the page tables to make sure that this memory isnt mapped.
     */
    for (auto start = start_addr; start < end_addr;
         start += _pageBytes) {
        if (_ownerProcess->pTable->lookup(start) != nullptr) {
            panic("Someone allocated physical memory at VA %p without "
                  "creating a VMA!\n", start);
            return false;
        }
    }
    return true;
}

void
MemState::updateBrkRegion(Addr old_brk, Addr new_brk)
{
    /**
     * To make this simple, avoid reducing the heap memory area if the
     * new_brk point is less than the old_brk; this occurs when the heap is
     * receding because the application has given back memory. The brk point
     * is still tracked in the MemState class as an independent field so that
     * it can be returned to the application; we just do not update the
     * region unless we expand it out.
     */
    if (new_brk < old_brk) {
        _brkPoint = new_brk;
        return;
    }

    /**
     * The regions must be page aligned but the break point can be set on
     * byte boundaries. Ensure that the restriction is maintained here by
     * extending the request out to the end of the page. (The roundUp
     * function will not round up an already aligned page.)
     */
    auto page_aligned_brk = roundUp(new_brk, _pageBytes);

    /**
     * Create a new mapping for the heap region. We only create a mapping
     * for the extra memory that is requested so we do not create a situation
     * where there can be overlapping mappings in the regions.
     *
     * Since we do not track the type of the region and we also do not
     * coalesce the regions together, we can create a fragmented set of
     * heap regions. To resolve this, we keep the furthest point ever mapped
     * by the _endBrkPoint field.
     */
    if (page_aligned_brk > _endBrkPoint) {
        auto length = page_aligned_brk - _endBrkPoint;
        /**
         * Check if existing mappings impede the expansion of brk expansion.
         * If brk cannot expand, it must return the original, unmodified brk
         * address and should not modify the mappings here.
         */
        if (!isUnmapped(_endBrkPoint, length)) {
            return;
        }

        /**
         * Note that the heap regions are always contiguous but there is
         * no mechanism right now to coalesce together memory that belongs
         * to the same region with similar access permissions. This could be
         * implemented if it actually becomes necessary; probably only
         * necessary if the list becomes too long to walk.
         */
        mapRegion(_endBrkPoint, length, "heap");
        _endBrkPoint = page_aligned_brk;
    }

    _brkPoint = new_brk;
}

void
MemState::mapRegion(Addr start_addr, Addr length,
                    const std::string& region_name, int sim_fd, Addr offset)
{
    DPRINTF(Vma, "memstate: creating vma (%s) [0x%x - 0x%x]\n",
            region_name.c_str(), start_addr, start_addr + length);

    /**
     * Avoid creating a region that has preexisting mappings. This should
     * not happen under normal circumstances so consider this to be a bug.
     */
    assert(isUnmapped(start_addr, length));

    /**
     * Record the region in our list structure.
     */
    _vmaList.emplace_back(AddrRange(start_addr, start_addr + length),
                          _pageBytes, region_name, sim_fd, offset);
}

void
MemState::unmapRegion(Addr start_addr, Addr length)
{
    Addr end_addr = start_addr + length;
    const AddrRange range(start_addr, end_addr);

    auto vma = std::begin(_vmaList);
    while (vma != std::end(_vmaList)) {
        if (vma->isStrictSuperset(range)) {
            DPRINTF(Vma, "memstate: split vma [0x%x - 0x%x] into "
                    "[0x%x - 0x%x] and [0x%x - 0x%x]\n",
                    vma->start(), vma->end(),
                    vma->start(), start_addr,
                    end_addr, vma->end());
            /**
             * Need to split into two smaller regions.
             * Create a clone of the old VMA and slice it to the right.
             */
            _vmaList.push_back(*vma);
            _vmaList.back().sliceRegionRight(start_addr);

            /**
             * Slice old VMA to encapsulate the left region.
             */
            vma->sliceRegionLeft(end_addr);

            /**
             * Region cannot be in any more VMA, because it is completely
             * contained in this one!
             */
            break;
        } else if (vma->isSubset(range)) {
            DPRINTF(Vma, "memstate: destroying vma [0x%x - 0x%x]\n",
                    vma->start(), vma->end());
            /**
             * Need to nuke the existing VMA.
             */
            vma = _vmaList.erase(vma);

            continue;

        } else if (vma->intersects(range)) {
            /**
             * Trim up the existing VMA.
             */
            if (vma->start() < start_addr) {
                DPRINTF(Vma, "memstate: resizing vma [0x%x - 0x%x] "
                        "into [0x%x - 0x%x]\n",
                        vma->start(), vma->end(),
                        vma->start(), start_addr);
                /**
                 * Overlaps from the right.
                 */
                vma->sliceRegionRight(start_addr);
            } else {
                DPRINTF(Vma, "memstate: resizing vma [0x%x - 0x%x] "
                        "into [0x%x - 0x%x]\n",
                        vma->start(), vma->end(),
                        end_addr, vma->end());
                /**
                 * Overlaps from the left.
                 */
                vma->sliceRegionLeft(end_addr);
            }
        }

        vma++;
    }

    /**
     * TLBs need to be flushed to remove any stale mappings from regions
     * which were unmapped. Currently the entire TLB is flushed. This results
     * in functionally correct execution, but real systems do not flush all
     * entries when a single mapping changes since it degrades performance.
     * There is currently no general method across all TLB implementations
     * that can flush just part of the address space.
     */
    for (auto *tc: _ownerProcess->system->threads) {
        tc->getDTBPtr()->flushAll();
        tc->getITBPtr()->flushAll();
    }

    do {
        if (!_ownerProcess->pTable->isUnmapped(start_addr, _pageBytes))
            _ownerProcess->pTable->unmap(start_addr, _pageBytes);

        start_addr += _pageBytes;

        /**
         * The regions need to always be page-aligned otherwise the while
         * condition will loop indefinitely. (The Addr type is currently
         * defined to be uint64_t in src/base/types.hh; it can underflow
         * since it is unsigned.)
         */
        length -= _pageBytes;
    } while (length > 0);
}

void
MemState::remapRegion(Addr start_addr, Addr new_start_addr, Addr length)
{
    Addr end_addr = start_addr + length;
    const AddrRange range(start_addr, end_addr);

    auto vma = std::begin(_vmaList);
    while (vma != std::end(_vmaList)) {
        if (vma->isStrictSuperset(range)) {
            /**
             * Create clone of the old VMA and slice right.
             */
            _vmaList.push_back(*vma);
            _vmaList.back().sliceRegionRight(start_addr);

            /**
             * Create clone of the old VMA and slice it left.
             */
            _vmaList.push_back(*vma);
            _vmaList.back().sliceRegionLeft(end_addr);

            /**
             * Slice the old VMA left and right to adjust the file backing,
             * then overwrite the virtual addresses!
             */
            vma->sliceRegionLeft(start_addr);
            vma->sliceRegionRight(end_addr);
            vma->remap(new_start_addr);

            /**
             * The region cannot be in any more VMAs, because it is
             * completely contained in this one!
             */
            break;
        } else if (vma->isSubset(range)) {
            /**
             * Just go ahead and remap it!
             */
            vma->remap(vma->start() - start_addr + new_start_addr);
        } else if (vma->intersects(range)) {
            /**
             * Create a clone of the old VMA.
             */
            _vmaList.push_back(*vma);

            if (vma->start() < start_addr) {
                /**
                 * Overlaps from the right.
                 */
                _vmaList.back().sliceRegionRight(start_addr);

                /**
                 * Remap the old region.
                 */
                vma->sliceRegionLeft(start_addr);
                vma->remap(new_start_addr);
            } else {
                /**
                 * Overlaps from the left.
                 */
                _vmaList.back().sliceRegionLeft(end_addr);

                /**
                 * Remap the old region.
                 */
                vma->sliceRegionRight(end_addr);
                vma->remap(new_start_addr + vma->start() - start_addr);
            }
        }

        vma++;
    }

    /**
     * TLBs need to be flushed to remove any stale mappings from regions
     * which were remapped. Currently the entire TLB is flushed. This results
     * in functionally correct execution, but real systems do not flush all
     * entries when a single mapping changes since it degrades performance.
     * There is currently no general method across all TLB implementations
     * that can flush just part of the address space.
     */
    for (auto *tc: _ownerProcess->system->threads) {
        tc->getDTBPtr()->flushAll();
        tc->getITBPtr()->flushAll();
    }

    do {
        if (!_ownerProcess->pTable->isUnmapped(start_addr, _pageBytes))
            _ownerProcess->pTable->remap(start_addr, _pageBytes,
                                         new_start_addr);

        start_addr += _pageBytes;
        new_start_addr += _pageBytes;

        /**
         * The regions need to always be page-aligned otherwise the while
         * condition will loop indefinitely. (The Addr type is currently
         * defined to be uint64_t in src/base/types.hh; it can underflow
         * since it is unsigned.)
         */
        length -= _pageBytes;
    } while (length > 0);
}

bool
MemState::fixupFault(Addr vaddr)
{
    /**
     * Check if we are accessing a mapped virtual address. If so then we
     * just haven't allocated it a physical page yet and can do so here.
     */
    for (const auto &vma : _vmaList) {
        if (vma.contains(vaddr)) {
            Addr vpage_start = roundDown(vaddr, _pageBytes);
            _ownerProcess->allocateMem(vpage_start, _pageBytes);

            /**
             * We are assuming that fresh pages are zero-filled, so there is
             * no need to zero them out when there is no backing file.
             * This assumption will not hold true if/when physical pages
             * are recycled.
             */
            if (vma.hasHostBuf()) {
                /**
                 * Write the memory for the host buffer contents for all
                 * ThreadContexts associated with this process.
                 */
                for (auto &cid : _ownerProcess->contextIds) {
                    auto *tc = _ownerProcess->system->threads[cid];
                    SETranslatingPortProxy
                        virt_mem(tc, SETranslatingPortProxy::Always);
                    vma.fillMemPages(vpage_start, _pageBytes, virt_mem);
                }
            }
            return true;
        }
    }

    /**
     * Check if the stack needs to be grown in the case where the ISAs
     * process argsInit does not explicitly map the entire stack.
     *
     * Check if this is already on the stack and there's just no page there
     * yet.
     */
    if (vaddr >= _stackMin && vaddr < _stackBase) {
        _ownerProcess->allocateMem(roundDown(vaddr, _pageBytes), _pageBytes);
        return true;
    }

    /**
     * We've accessed the next page of the stack, so extend it to include
     * this address.
     */
    if (vaddr < _stackMin && vaddr >= _stackBase - _maxStackSize) {
        while (vaddr < _stackMin) {
            _stackMin -= _pageBytes;
            if (_stackBase - _stackMin > _maxStackSize) {
                fatal("Maximum stack size exceeded\n");
            }
            _ownerProcess->allocateMem(_stackMin, _pageBytes);
            inform("Increasing stack size by one page.");
        }
        return true;
    }

    return false;
}

Addr
MemState::extendMmap(Addr length)
{
    Addr start = _mmapEnd;

    if (_ownerProcess->mmapGrowsDown())
        start = _mmapEnd - length;

    // Look for a contiguous region of free virtual memory.  We can't assume
    // that the region beyond mmap_end is free because of fixed mappings from
    // the user.
    while (!isUnmapped(start, length)) {
        DPRINTF(Vma, "memstate: cannot extend vma for mmap region at %p. "
                "Virtual address range is already reserved! Skipping a page "
                "and trying again!\n", start);
        start = (_ownerProcess->mmapGrowsDown()) ? start - _pageBytes :
            start + _pageBytes;
    }

    DPRINTF(Vma, "memstate: extending mmap region (old %p) (new %p)\n",
            _mmapEnd,
            _ownerProcess->mmapGrowsDown() ? start : start + length);

    _mmapEnd = _ownerProcess->mmapGrowsDown() ? start : start + length;

    return start;
}

std::string
MemState::printVmaList()
{
    std::stringstream file_content;

    for (auto vma : _vmaList) {
        std::stringstream line;
        line << std::hex << vma.start() << "-";
        line << std::hex << vma.end() << " ";
        line << "r-xp 00000000 00:00 0 ";
        line << "[" << vma.getName() << "]" << std::endl;
        file_content << line.str();
    }

    return file_content.str();
}
