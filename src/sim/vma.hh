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

#ifndef __SRC_MEM_VMA_HH__
#define __SRC_MEM_VMA_HH__

#include <string>

#include "base/addr_range.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Vma.hh"
#include "mem/se_translating_port_proxy.hh"

namespace gem5
{

class VMA
{
  class MappedFileBuffer;

  public:
    VMA(AddrRange r, Addr page_bytes, const std::string &vma_name = "anon",
        int fd = -1, off_t off = 0, int bus_signal = 0)
        : _addrRange(r),
          _pageBytes(page_bytes),
          _hostFileOffset(off),
          _vmaName(vma_name),
          _busSignal(bus_signal)
    {
        DPRINTF(Vma, "Creating vma start %#x len %llu end %#x\n",
                r.start(), r.size(), r.end());

        if (fd != -1) {
            _origHostBuf =
                std::make_shared<MappedFileBuffer>(fd, r.size(), off);
        }

        sanityCheck();
    }

    /**
     * Remap the virtual memory area starting at new_start.
     */
    void
    remap(Addr new_start)
    {
        _addrRange = AddrRange(new_start, new_start + _addrRange.size());

        DPRINTF(Vma, "Remapping vma start %#x end %#x\n", _addrRange.start(),
                _addrRange.end());

        sanityCheck();
    }

    /**
     * Check if the virtual memory area has an equivalent buffer on the
     * host machine.
     */
    bool hasHostBuf() const { return _origHostBuf != nullptr; }

    int
    getBusSignal() const
    {
        return _busSignal;
    }

    int
    faultSignal(Addr addr) const
    {
        if (!_busSignal || !contains(addr)) {
            return 0;
        }

        const Addr offset = addr - _addrRange.start();
        const Addr page_offset = offset - offset % _pageBytes;
        const uint64_t file_offset = _hostFileOffset + page_offset;
        return file_offset >= _origHostBuf->getFileSize() ? _busSignal : 0;
    }

    /**
     * Copy memory from a buffer which resides on the host machine into a
     * section of memory on the target.
     */
    void fillMemPages(Addr start, Addr size, PortProxy &port) const;

    /**
     * Returns true if desired range exists within this virtual memory area
     * and does not include the start and end addresses.
     */
    bool isStrictSuperset(const AddrRange &range) const;

    /**
     * Remove the address range to the right of slice_addr.
     */
    void sliceRegionRight(Addr slice_addr);

    /**
     * Remove the address range to the left of slice_addr.
     */
    void sliceRegionLeft(Addr slice_addr);

    const std::string& getName() { return _vmaName; }
    off_t getFileMappingOffset() const
    {
        return hasHostBuf() ? _hostFileOffset : 0;
    }

    /**
     * Defer AddrRange related calls to the AddrRange.
     */
    Addr size() { return _addrRange.size(); }
    Addr start() { return _addrRange.start(); }
    Addr end() { return _addrRange.end(); }

    bool
    mergesWith(const AddrRange& r) const
    {
        return _addrRange.mergesWith(r);
    }

    bool
    intersects(const AddrRange& r) const
    {
        return _addrRange.intersects(r);
    }

    bool
    isSubset(const AddrRange& r) const
    {
        return _addrRange.isSubset(r);
    }

    bool
    contains(const Addr& a) const
    {
        return _addrRange.contains(a);
    }

  private:
    void sanityCheck();

    /**
     * Address range for this virtual memory area.
     */
    AddrRange _addrRange;

    /**
     * Number of bytes in an OS page.
     */
    Addr _pageBytes;

    /** File offset corresponding to the start of this VMA. */
    off_t _hostFileOffset;

    /**
     * The host file backing will be chopped up and reassigned as pages are
     * mapped, remapped, and unmapped. In addition to the current host
     * pointer and length, each virtual memory area will also keep a
     * reference-counted handle to the original host memory. The last virtual
     * memory area to die cleans up the host memory it handles.
     */
    std::shared_ptr<MappedFileBuffer> _origHostBuf;

    /**
     * Human-readable name associated with the virtual memory area.
     * The name is useful for debugging and also exposing vma state through
     * the psuedo file system (i.e. Linux's /proc/self/maps) to the
     * application.
     */
    std::string _vmaName;

    /** Target signal for accesses to file-backed pages beyond EOF. */
    int _busSignal;

    /**
     * MappedFileBuffer retains a file mapping and descriptor so file-backed
     * pages can be populated if the file grows after the target mmap call.
     */
    class MappedFileBuffer
    {
      public:
        MappedFileBuffer(int fd, size_t length, off_t offset);
        ~MappedFileBuffer();

        uint64_t getFileSize() const;
        size_t readAt(off_t offset, uint8_t *buffer, size_t size) const;

      private:
        void *_buffer;       // Host buffer ptr
        size_t _length;      // Length of host mapping
        off_t _offset;       // Offset in file at which mapping starts
        int _fd;             // Duplicated backing file descriptor
    };
};

} // namespace gem5

#endif // __SRC_MEM_VMA_HH__
