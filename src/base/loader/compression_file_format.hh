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

#ifndef __BASE_LOADER_COMPRESSION_FILE_FORMAT_HH__
#define __BASE_LOADER_COMPRESSION_FILE_FORMAT_HH__

#include <cstddef>
#include <cstdint>
#include <string>

namespace gem5
{

namespace loader
{

/**
 * Override this class when providing support for a new compression format
 * for guest binaries (this means decompressing a guest binary on the fly
 * before running it instead of manually decompressing it beforehand).
 *
 * The decompression works by decompressing the binary and by writing
 * the output stream (writeAll) to a temporary file (makeTempFile)
 * which will be mapped later on to the guest address space
 *
 * The class/interface has two main APIs that have to be implemented
 *
 * matches => returns true if the file pointed by the file descriptor
 * has been compressed with the class format
 *
 * decompress => implements the decompression
 */
class CompressionFileFormat
{
  protected:
    CompressionFileFormat();
    virtual ~CompressionFileFormat() = default;

    static bool hasMagic(int fd, const uint8_t *magic, size_t magic_len);
    // Writes the output stream to a file fd. Returns false if there has
    // been an error, true otherwise
    static bool writeAll(int fd, const uint8_t *buf, size_t size);
    static int makeTempFile(const char *suffix);

  public:
    CompressionFileFormat(const CompressionFileFormat &) = delete;
    void operator=(const CompressionFileFormat &) = delete;

    /**
     * Can the file fd be decompressed by this handler?
     *
     * @param fd file descriptor
     * @return true if this can decompress fd, false otherwise
     */
    virtual bool matches(int fd) const = 0;

    /**
     * Decompress the file fd. Return the new file descriptor
     * for the decompressed binary
     *
     * @param fd file descriptor
     * @return new_fd pointing to the decompressed file
     */
    virtual int decompress(int fd) const = 0;
    virtual const char *name() const = 0;
};

int decompressImageFile(int fd, const std::string &filename);

} // namespace loader
} // namespace gem5

#endif // __BASE_LOADER_COMPRESSION_FILE_FORMAT_HH__
