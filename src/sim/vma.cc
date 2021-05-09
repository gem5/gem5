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

#include "sim/vma.hh"

#include <sys/mman.h>
#include <sys/stat.h>

#include "base/types.hh"

namespace gem5
{

void
VMA::fillMemPages(Addr start, Addr size, PortProxy &port) const
{
    auto offset = start - _addrRange.start();

    /**
     * Try to copy a full page, but don't overrun the size of the file.
     */
    if (offset < _hostBufLen) {
        auto size = std::min(_hostBufLen - offset, _pageBytes);
        port.writeBlob(start, (uint8_t*)_hostBuf + offset, size);
    }
}

bool
VMA::isStrictSuperset(const AddrRange &r) const
{
    return (r.start() > _addrRange.start() && r.end() < _addrRange.end());
}

void
VMA::sliceRegionRight(Addr slice_addr)
{
    if (hasHostBuf()) {
        auto nonoverlap_len = slice_addr - _addrRange.start();
        _hostBufLen = std::min(_hostBufLen, nonoverlap_len);
    }

    _addrRange = AddrRange(_addrRange.start(), slice_addr);

    DPRINTF(Vma, "slice right vma start %#x end %#x\n", _addrRange.start(),
            _addrRange.end());

    sanityCheck();
}

void
VMA::sliceRegionLeft(Addr slice_addr)
{
    if (hasHostBuf()) {
        auto overlap_len = slice_addr - _addrRange.start();

        if (overlap_len >= _hostBufLen) {
            _hostBufLen = 0;
            _hostBuf = nullptr;
            _origHostBuf = nullptr;
        } else {
            _hostBufLen -= overlap_len;
        }

        _hostBuf = (void *)((uint8_t *)_hostBuf + overlap_len);
    }

    _addrRange = AddrRange(slice_addr, _addrRange.end());

    DPRINTF(Vma, "slice left vma start %#x end %#x\n", _addrRange.start(),
            _addrRange.end());

    sanityCheck();
}

void
VMA::sanityCheck()
{
    /**
     * Avoid regions without a length.
     */
    assert(_addrRange.start() != _addrRange.end());

    /**
     * Avoid regions with an end point before the start point
     */
    assert(_addrRange.start() < _addrRange.end());

    /**
     *  Avoid non-aligned regions; we assume in the code that the
     *  regions are page aligned so consider this to be a bug.
     */
    assert((_addrRange.start() % _pageBytes) == 0);
    assert((_addrRange.end() % _pageBytes) == 0);
}

VMA::MappedFileBuffer::MappedFileBuffer(int fd, size_t length,
                                        off_t offset)
    : _buffer(nullptr), _length(length)
{
    panic_if(_length == 0, "Tried to mmap file of length zero");

    struct stat file_stat;
    if (fstat(fd, &file_stat) > 0) {
        panic("Cannot stat file: %s\n", strerror(errno));
    }

    // Don't bother mapping more than the actual file size
    panic_if(offset > file_stat.st_size,
             "Tried to mmap with offset greater than file size");
    _length = std::min((size_t)(file_stat.st_size - offset), _length);

    // cannot call mmap with _length == 0
    if (_length) {
        _buffer = mmap(NULL, _length, PROT_READ,
                                 MAP_PRIVATE, fd, offset);
        if (_buffer == MAP_FAILED) {
            panic("Failed to map file into host address space: %s",
                  strerror(errno));
        }
    } else {
        panic("Tried to mmap 0 bytes");
    }
}

VMA::MappedFileBuffer::~MappedFileBuffer()
{
    if (_buffer) {
        panic_if(munmap(_buffer, _length) == -1,
                 "mmap: failed to unmap file-backed host memory: %s",
                 strerror(errno));
    }
}

} // namespace gem5
