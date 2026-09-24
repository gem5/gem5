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

#include "base/loader/gzip_compression.hh"

#include <unistd.h>
#include <zlib.h>

namespace gem5
{

namespace loader
{

GzipCompressionFormat::GzipCompressionFormat() : CompressionFileFormat()
{}

bool
GzipCompressionFormat::matches(int fd) const
{
    const uint8_t magic[] = {0x1f, 0x8b};
    return hasMagic(fd, magic, sizeof(magic));
}

int
GzipCompressionFormat::decompress(int fd) const
{
    const size_t blk_sz = 4096;

    gzFile fdz = gzdopen(fd, "rb");
    if (!fdz) {
        return -1;
    }

    fd = makeTempFile("/gem5-gz-obj-XXXXXX");
    if (fd < 0) {
        gzclose(fdz);
        return fd;
    }

    auto buf = new uint8_t[blk_sz];
    int r;
    while ((r = gzread(fdz, buf, blk_sz)) > 0) {
        if (!writeAll(fd, buf, r)) {
            delete[] buf;
            gzclose(fdz);
            close(fd);
            return -1;
        }
    }
    delete[] buf;
    gzclose(fdz);
    if (r < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

const char *
GzipCompressionFormat::name() const
{
    return "gzip";
}

void
registerGzipCompressionFormat()
{
    static GzipCompressionFormat gzipCompressionFormat;
}

} // namespace loader
} // namespace gem5
