/*
 * Copyright (c) 2026 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include "base/loader/image_file_data.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include "base/loader/compression_file_format.hh"
#include "base/logging.hh"

namespace gem5
{

namespace loader
{

ImageFileData::ImageFileData(const std::string &fname, bool try_decompress)
{
    _filename = fname;

    // Open the file.
    int fd = open(fname.c_str(), O_RDONLY);
    fatal_if(fd < 0, "Failed to open file %s.\n"
        "This error typically occurs when the file path specified is "
        "incorrect.\n", fname);

    if (try_decompress) {
        fd = decompressImageFile(fd, fname);
    }

    // Find the length of the file by seeking to the end.
    off_t off = lseek(fd, 0, SEEK_END);
    fatal_if(off < 0, "Failed to determine size of file %s.\n", fname);
    _len = static_cast<size_t>(off);

    // Mmap the whole shebang.
    _data = (uint8_t *)mmap(NULL, _len, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);

    panic_if(_data == MAP_FAILED, "Failed to mmap file %s.\n", fname);
}

ImageFileData::~ImageFileData()
{
    munmap((void *)_data, _len);
}

} // namespace loader
} // namespace gem5
