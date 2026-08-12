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

#include "base/loader/compression_file_format.hh"

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include "base/loader/gzip_compression.hh"
#include "base/logging.hh"

namespace gem5
{

namespace loader
{

namespace
{

typedef std::vector<CompressionFileFormat *> CompressionFileFormatList;

CompressionFileFormatList &
compressionFileFormats()
{
    static CompressionFileFormatList formats;
    return formats;
}

void
registerBuiltinCompressionFileFormats()
{
    registerGzipCompressionFormat();
}

} // anonymous namespace

CompressionFileFormat::CompressionFileFormat()
{
    compressionFileFormats().emplace_back(this);
}

bool
CompressionFileFormat::hasMagic(int fd, const uint8_t *magic, size_t magic_len)
{
    std::vector<uint8_t> buf(magic_len);
    ssize_t sz = pread(fd, buf.data(), magic_len, 0);
    panic_if(sz < 0, "Couldn't read magic bytes from object file");
    if (static_cast<size_t>(sz) != magic_len) {
        return false;
    }

    return memcmp(buf.data(), magic, magic_len) == 0;
}

bool
CompressionFileFormat::writeAll(int fd, const uint8_t *buf, size_t size)
{
    while (size > 0) {
        ssize_t sz = write(fd, buf, size);
        if (sz <= 0) {
            return false;
        }

        size -= sz;
        buf += sz;
    }

    return true;
}

int
CompressionFileFormat::makeTempFile(const char *suffix)
{
    std::string tmpnam_str = std::string(P_tmpdir) + suffix;
    char *tmpnam = tmpnam_str.data();
    int fd = mkstemp(tmpnam);
    if (fd < 0) {
        return fd;
    }

    if (unlink(tmpnam) != 0) {
        warn("couldn't remove temporary file %s\n", tmpnam);
    }

    return fd;
}

int
decompressImageFile(int fd, const std::string &filename)
{
    registerBuiltinCompressionFileFormats();

    for (const auto &format : compressionFileFormats()) {
        if (!format->matches(fd)) {
            continue;
        }

        int decompressed_fd = format->decompress(fd);
        panic_if(decompressed_fd < 0, "Failed to uncompress %s file %s.\n",
                 format->name(), filename);
        return decompressed_fd;
    }

    return fd;
}

} // namespace loader
} // namespace gem5
