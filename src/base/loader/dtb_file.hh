/*
 * Copyright (c) 2013 The Regents of The University of Michigan
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

#ifndef __BASE_LOADER_DTB_FILE_HH__
#define __BASE_LOADER_DTB_FILE_HH__

#include "base/compiler.hh"
#include "base/loader/image_file.hh"

namespace gem5
{

namespace loader
{

/** @file
 * This implements an image file format to support loading
 * and modifying flattened device tree blobs for use with
 * current and future ARM Linux kernels.
 */
class DtbFile : public ImageFile
{
  protected:
    /** Bool marking if this dtb file has replaced the original
     *  read in DTB file with a new modified buffer
     */
    bool fileDataMmapped;
    uint8_t *fileData = nullptr;
    size_t length = 0;

  public:
    DtbFile(const std::string &name);
    ~DtbFile();

    /** Adds the passed in Command Line options and initrd for the kernel
     * to the proper location in the device tree.
     * @param _cmdline command line to append
     * @param cmdline_len length of the command line string
     * @param initrd_addr starting physical address of initrd
     * @param initrd_len length of initrd data in memory
     * @return returns true on success, false otherwise
     */
    bool addBootData(const char *_cmdline, size_t cmdline_len,
                     off_t initrd_addr, size_t initrd_len);

    /** Adds the passed in Command Line options for the kernel
     * to the proper location in the device tree.
     * @param _args command line to append
     * @param len length of the command line string
     * @return returns true on success, false otherwise
     */
    inline bool
    addBootCmdLine(const char *_args, size_t len)
    {
        return addBootData(_args, len, 0, 0);
    }

    /** Parse the DTB file enough to find the provided release
     * address and return it.
     * @return release address for SMP boot
     */
    Addr findReleaseAddr();

    MemoryImage buildImage() const override;
};

} // namespace loader
} // namespace gem5

#endif //__BASE_LOADER_DTB_FILE_HH__
