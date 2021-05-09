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

#ifndef SRC_SIM_MEM_STATE_HH
#define SRC_SIM_MEM_STATE_HH

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "debug/Vma.hh"
#include "mem/page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/serialize.hh"
#include "sim/vma.hh"

namespace gem5
{

class Process;
struct ProcessParams;
class System;

/**
 * This class holds the memory state for the Process class and all of its
 * derived, architecture-specific children.
 *
 * The class represents the Process' address space which may change
 * dynamically while the simulation is running. They are updated by system
 * calls and faults. Each change represents a modification to the process
 * address space.
 *
 * The class is meant to be allocated dynamically and shared through a
 * pointer interface. Multiple process can potentially share portions of their
 * virtual address space if specific options are passed into the clone(2)
 * system call.
 */
class MemState : public Serializable
{
  public:
    MemState(Process *owner, Addr brk_point, Addr stack_base,
             Addr max_stack_size, Addr next_thread_stack_base,
             Addr mmap_end);

    MemState& operator=(const MemState &in);

    /**
     * Change the Process owner in case this MemState is copied.
     */
    void resetOwner(Process *owner);

    /**
     * Get/set base addresses and sizes for the stack and data segments of
     * the process' memory.
     */
    Addr getBrkPoint() const { return _brkPoint; }
    Addr getStackBase() const { return _stackBase; }
    Addr getStackSize() const { return _stackSize; }
    Addr getMaxStackSize() const { return _maxStackSize; }
    Addr getStackMin() const { return _stackMin; }
    Addr getNextThreadStackBase() const { return _nextThreadStackBase; }
    Addr getMmapEnd() const { return _mmapEnd; }
    void setBrkPoint(Addr brk_point) { _brkPoint = brk_point; }
    void setStackBase(Addr stack_base) { _stackBase = stack_base; }
    void setStackSize(Addr stack_size) { _stackSize = stack_size; }
    void setMaxStackSize(Addr max_stack) { _maxStackSize = max_stack; }
    void setStackMin(Addr stack_min) { _stackMin = stack_min; }
    void setNextThreadStackBase(Addr ntsb) { _nextThreadStackBase = ntsb; }
    void setMmapEnd(Addr mmap_end) { _mmapEnd = mmap_end; }

    /*
     * Extend the end of the mmap region by length bytes. Once a contiguous
     * region of free virtual memory is found the start of that region is
     * returned.
     */
    Addr extendMmap(Addr length);

    /**
     * Check if any page in the virtual address range from start_addr to
     * start_addr + length is already mapped in the page table.
     *
     * @param start_addr Starting address of region to check.
     * @param length Length of the range to check.
     *
     * @return true if all pages in the range are unmapped in page table
     */
    bool isUnmapped(Addr start_addr, Addr length);

    /**
     * Add a new memory region. The region represents a contiguous virtual
     * address range which can map to physical memory or a host-backed file.
     * Regions which are not file-backed should use -1 for sim_fd and 0 for
     * offset.
     *
     * @param start_addr Starting address of the region.
     * @param length Size of the region.
     * @param name Name of region. Optional.
     * @param sim_fd File descriptor for file-backed regions or -1.
     * @param offset Offset in file in which region starts.
     */
    void mapRegion(Addr start_addr, Addr length,
                   const std::string& name="anon", int sim_fd=-1,
                   Addr offset=0);

    /**
     * Unmap a pre-existing region. Depending on the range being unmapped
     * the resulting new regions will either be split, resized, or
     * removed completely.
     *
     * @param start_addr Starting address of region to unmap.
     * @param length Size of region to unmap.
     */
    void unmapRegion(Addr start_addr, Addr length);

    /**
     * Remap a pre-existing region. This changes the virtual address
     * range of the region. This will result in regions being expanded
     * if there is overlap with another region or simply moving the range
     * otherwise.
     *
     * @param start_addr Start address of region being remapped.
     * @param new_start_addr New start address of the region.
     * @param length Length of the newly remapped region.
     */
    void remapRegion(Addr start_addr, Addr new_start_addr, Addr length);

    /**
     * Change the end of a process' program break. This represents the end
     * of the heap segment of a process.
     *
     * @param old_brk Old program break address
     * @param new_brk New program break address
     */
    void updateBrkRegion(Addr old_brk, Addr new_brk);

    /**
     * Attempt to fix up a fault at vaddr by allocating a page. The fault
     * likely occurred because a virtual page which does not have physical
     * page assignment is being accessed.
     *
     * @param vaddr The virtual address which is causing the fault.
     * @return Whether the fault has been fixed.
     */
    bool fixupFault(Addr vaddr);

    /**
     * Given the vaddr and size, this method will chunk the allocation into
     * page granularity and then request physical pages (frames) from the
     * system object. After retrieving a frame, the method updates the page
     * table mappings.
     *
     * @param vaddr The virtual address in need of a frame allocation.
     * @param size The size in bytes of the requested mapping.
     * @param clobber This flag specifies whether mappings in the page tables
     *        can be overwritten and replaced with the new mapping.
     */
    void allocateMem(Addr vaddr, int64_t size, bool clobber = false);

    void
    serialize(CheckpointOut &cp) const override
    {
        paramOut(cp, "brkPoint", _brkPoint);
        paramOut(cp, "stackBase", _stackBase);
        paramOut(cp, "stackSize", _stackSize);
        paramOut(cp, "maxStackSize", _maxStackSize);
        paramOut(cp, "stackMin", _stackMin);
        paramOut(cp, "nextThreadStackBase", _nextThreadStackBase);
        paramOut(cp, "mmapEnd", _mmapEnd);

        ScopedCheckpointSection sec(cp, "vmalist");
        paramOut(cp, "size", _vmaList.size());
        int count = 0;
        for (auto vma : _vmaList) {
            ScopedCheckpointSection sec(cp, csprintf("Vma%d", count++));
            paramOut(cp, "name", vma.getName());
            paramOut(cp, "addrRangeStart", vma.start());
            paramOut(cp, "addrRangeEnd", vma.end());
        }
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        paramIn(cp, "brkPoint", _brkPoint);
        paramIn(cp, "stackBase", _stackBase);
        paramIn(cp, "stackSize", _stackSize);
        paramIn(cp, "maxStackSize", _maxStackSize);
        paramIn(cp, "stackMin", _stackMin);
        paramIn(cp, "nextThreadStackBase", _nextThreadStackBase);
        paramIn(cp, "mmapEnd", _mmapEnd);

        int count;
        ScopedCheckpointSection sec(cp, "vmalist");
        paramIn(cp, "size", count);
        for (int i = 0; i < count; ++i) {
            ScopedCheckpointSection sec(cp, csprintf("Vma%d", i));
            std::string name;
            Addr start;
            Addr end;
            paramIn(cp, "name", name);
            paramIn(cp, "addrRangeStart", start);
            paramIn(cp, "addrRangeEnd", end);
            _vmaList.emplace_back(AddrRange(start, end), _pageBytes, name);
        }
    }

    /**
     * Print the list of VMAs in a format similar to /proc/self/maps
     */
    std::string printVmaList();

  private:
    /**
     * @param
     */
    void replicatePage(const MemState &in, Addr vaddr, Addr new_paddr,
                       bool alloc_page);

    /**
     * @param
     */
    System * system() const;

    /**
     * Owner process of MemState. Used to manipulate page tables.
     */
    Process * _ownerProcess;

    Addr _pageBytes;
    Addr _brkPoint;
    Addr _stackBase;
    Addr _stackSize;
    Addr _maxStackSize;
    Addr _stackMin;
    Addr _nextThreadStackBase;
    Addr _mmapEnd;

    /**
     * Keeps record of the furthest mapped heap location.
     */
    Addr _endBrkPoint;

    /**
     * The _vmaList member is a list of virtual memory areas in the target
     * application space that have been allocated by the target. In most
     * operating systems, lazy allocation is used and these structures (or
     * equivalent ones) are used to track the valid address ranges.
     *
     * This could use a more efficient data structure like an interval
     * tree, but it is unclear whether the vmas will be modified often enough
     * for the improvement in lookup time to matter. Unmapping VMAs currently
     * modifies the list while iterating so the STL container must either
     * support this or the unmapping method must be changed.
     */
    std::list<VMA> _vmaList;
};

} // namespace gem5

#endif
