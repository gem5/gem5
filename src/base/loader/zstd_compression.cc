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

#include "base/loader/zstd_compression.hh"

#include <unistd.h>
#include <zstd.h>

#include <vector>

#include "base/logging.hh"

namespace gem5
{

namespace loader
{

ZstdCompressionFormat::ZstdCompressionFormat() : CompressionFileFormat()
{}

bool
ZstdCompressionFormat::matches(int fd) const
{
    const uint8_t magic[] = {0x28, 0xb5, 0x2f, 0xfd};
    return hasMagic(fd, magic, sizeof(magic));
    if (hasMagic(fd, magic, sizeof(magic))) {
        return true;
    }

    // Testing for skippable frames
    for (uint8_t variant = 0x50; variant <= 0x5f; ++variant) {
        const uint8_t skippable_magic[] = {variant, 0x2a, 0x4d, 0x18};
        if (hasMagic(fd, skippable_magic, sizeof(skippable_magic))) {
            return true;
        }
    }
    return false;
}

int
ZstdCompressionFormat::decompress(int in_fd) const
{
    int out_fd = makeTempFile("/gem5-zstd-obj-XXXXXX");
    if (out_fd < 0) {
        close(in_fd);
        return out_fd;
    }

    ZSTD_DStream *stream = ZSTD_createDStream();
    if (!stream) {
        close(in_fd);
        close(out_fd);
        return -1;
    }

    size_t ret = ZSTD_initDStream(stream);
    if (ZSTD_isError(ret)) {
        warn("couldn't initialize zstd decompressor: %s\n",
             ZSTD_getErrorName(ret));
        ZSTD_freeDStream(stream);
        close(in_fd);
        close(out_fd);
        return -1;
    }

    std::vector<uint8_t> in_buf(ZSTD_DStreamInSize());
    std::vector<uint8_t> out_buf(ZSTD_DStreamOutSize());

    ret = 1;
    ssize_t read_size = 0;
    while ((read_size = read(in_fd, in_buf.data(), in_buf.size())) > 0) {
        ZSTD_inBuffer input = {in_buf.data(), static_cast<size_t>(read_size),
                               0};

        bool output_full = false;
        do {
            ZSTD_outBuffer output = {out_buf.data(), out_buf.size(), 0};
            ret = ZSTD_decompressStream(stream, &output, &input);
            if (ZSTD_isError(ret)) {
                warn("couldn't decompress zstd stream: %s\n",
                     ZSTD_getErrorName(ret));
                ZSTD_freeDStream(stream);
                close(in_fd);
                close(out_fd);
                return -1;
            }

            if (!writeAll(out_fd, out_buf.data(), output.pos)) {
                ZSTD_freeDStream(stream);
                close(in_fd);
                close(out_fd);
                return -1;
            }

            output_full = output.pos == output.size;
        } while (input.pos < input.size || (ret != 0 && output_full));
    }

    ZSTD_freeDStream(stream);
    close(in_fd);

    if (read_size < 0 || ret != 0) {
        close(out_fd);
        return -1;
    }

    return out_fd;
}

const char *
ZstdCompressionFormat::name() const
{
    return "zstd";
}

void
registerZstdCompressionFormat()
{
    static ZstdCompressionFormat zstdCompressionFormat;
}

} // namespace loader
} // namespace gem5
