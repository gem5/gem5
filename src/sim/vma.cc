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

#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "base/types.hh"

namespace gem5
{

void
VMA::fillMemPages(Addr start, Addr size, PortProxy &port) const
{
    const off_t file_offset = _hostFileOffset + (start - _addrRange.start());
    const uint64_t file_size = _origHostBuf->getFileSize();
    if (static_cast<uint64_t>(file_offset) >= file_size) {
        return;
    }

    const size_t bytes = std::min<uint64_t>(size, file_size - file_offset);
    std::vector<uint8_t> buffer(bytes);
    const size_t bytes_read =
        _origHostBuf->readAt(file_offset, buffer.data(), bytes);
    if (bytes_read) {
        port.writeBlob(start, buffer.data(), bytes_read);
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
        _hostFileOffset += overlap_len;
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

VMA::MappedFileBuffer::MappedFileBuffer(int fd, size_t length, off_t offset)
    : _buffer(nullptr), _length(length), _offset(offset), _fd(dup(fd))
{
    panic_if(_length == 0, "Tried to mmap file of length zero");
    panic_if(offset < 0, "Tried to mmap file at a negative offset");
    panic_if(_fd < 0, "Cannot duplicate file descriptor: %s", strerror(errno));

    struct stat file_stat;
    if (fstat(_fd, &file_stat) < 0) {
        panic("Cannot stat file: %s\n", strerror(errno));
    }

    // A file mapping beyond EOF still has a target VMA; leave its host
    // backing empty so the target fault path can report SIGBUS.
    if (offset >= file_stat.st_size) {
        _length = 0;
        return;
    }

    // Don't map more host bytes than remain in the file.
    _length =
        std::min(static_cast<size_t>(file_stat.st_size - offset), _length);

    // cannot call mmap with _length == 0
    if (_length) {
        _buffer = mmap(NULL, _length, PROT_READ, MAP_PRIVATE, _fd, offset);
        if (_buffer == MAP_FAILED) {
            panic("Failed to map file into host address space: %s",
                  strerror(errno));
        }
    }
}

VMA::MappedFileBuffer::~MappedFileBuffer()
{
    if (_buffer) {
        panic_if(munmap(_buffer, _length) == -1,
                 "mmap: failed to unmap file-backed host memory: %s",
                 strerror(errno));
    }
    (void)close(_fd);
}

uint64_t
VMA::MappedFileBuffer::getFileSize() const
{
    struct stat file_stat;
    panic_if(fstat(_fd, &file_stat) < 0, "Cannot stat file-backed mapping: %s",
             strerror(errno));
    return file_stat.st_size;
}

size_t
VMA::MappedFileBuffer::readAt(off_t offset, uint8_t *buffer, size_t size) const
{
    size_t bytes_read = 0;
    const off_t mapping_end = _offset + _length;
    if (_buffer && offset >= _offset && offset < mapping_end) {
        const size_t mapped_bytes =
            std::min<size_t>(size, mapping_end - offset);
        std::memcpy(buffer,
                    static_cast<uint8_t *>(_buffer) + (offset - _offset),
                    mapped_bytes);
        bytes_read = mapped_bytes;
    }

    while (bytes_read < size) {
        const ssize_t result = pread(_fd, buffer + bytes_read,
                                     size - bytes_read, offset + bytes_read);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        panic_if(result < 0, "Failed to read file-backed mapping: %s",
                 strerror(errno));
        if (result == 0) {
            break;
        }
        bytes_read += result;
    }
    return bytes_read;
}

} // namespace gem5
