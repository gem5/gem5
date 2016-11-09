/*
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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#include "base/loader/object_file.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <zlib.h>

#include <cstdio>
#include <list>
#include <string>

#include "base/cprintf.hh"
#include "base/loader/aout_object.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/ecoff_object.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/raw_object.hh"
#include "base/loader/symtab.hh"
#include "mem/port_proxy.hh"

using namespace std;

ObjectFile::ObjectFile(const string &_filename,
                       size_t _len, uint8_t *_data,
                       Arch _arch, OpSys _op_sys)
    : filename(_filename), fileData(_data), len(_len),
      arch(_arch), opSys(_op_sys), entry(0), globalPtr(0),
      text{0, nullptr, 0}, data{0, nullptr, 0}, bss{0, nullptr, 0}
{
}


ObjectFile::~ObjectFile()
{
    if (fileData) {
        ::munmap((char*)fileData, len);
        fileData = NULL;
    }
}


bool
ObjectFile::loadSection(Section *sec, PortProxy& mem_proxy, Addr addr_mask,
                        Addr offset)
{
    if (sec->size != 0) {
        Addr addr = (sec->baseAddr & addr_mask) + offset;
        if (sec->fileImage) {
            mem_proxy.writeBlob(addr, sec->fileImage, sec->size);
        }
        else {
            // no image: must be bss
            mem_proxy.memsetBlob(addr, 0, sec->size);
        }
    }
    return true;
}


bool
ObjectFile::loadSections(PortProxy& mem_proxy, Addr addr_mask, Addr offset)
{
    return (loadSection(&text, mem_proxy, addr_mask, offset)
            && loadSection(&data, mem_proxy, addr_mask, offset)
            && loadSection(&bss, mem_proxy, addr_mask, offset));
}

static bool
hasGzipMagic(int fd)
{
    uint8_t buf[2] = {0};
    size_t sz = pread(fd, buf, 2, 0);
    panic_if(sz != 2, "Couldn't read magic bytes from object file");
    return ((buf[0] == 0x1f) && (buf[1] == 0x8b));
}

static int
doGzipLoad(int fd)
{
    const size_t blk_sz = 4096;

    gzFile fdz = gzdopen(fd, "rb");
    if (!fdz) {
        return -1;
    }

    size_t tmp_len = strlen(P_tmpdir);
    char *tmpnam = (char*) malloc(tmp_len + 20);
    strcpy(tmpnam, P_tmpdir);
    strcpy(tmpnam+tmp_len, "/gem5-gz-obj-XXXXXX"); // 19 chars
    fd = mkstemp(tmpnam); // repurposing fd variable for output
    if (fd < 0) {
        free(tmpnam);
        gzclose(fdz);
        return fd;
    }

    if (unlink(tmpnam) != 0)
        warn("couldn't remove temporary file %s\n", tmpnam);

    free(tmpnam);

    auto buf = new uint8_t[blk_sz];
    int r; // size of (r)emaining uncopied data in (buf)fer
    while ((r = gzread(fdz, buf, blk_sz)) > 0) {
        auto p = buf; // pointer into buffer
        while (r > 0) {
            auto sz = write(fd, p, r);
            assert(sz <= r);
            r -= sz;
            p += sz;
        }
    }
    delete[] buf;
    gzclose(fdz);
    if (r < 0) { // error
        close(fd);
        return -1;
    }
    assert(r == 0); // finished successfully
    return fd; // return fd to decompressed temporary file for mmap()'ing
}

ObjectFile *
createObjectFile(const string &fname, bool raw)
{
    // open the file
    int fd = open(fname.c_str(), O_RDONLY);
    if (fd < 0) {
        return NULL;
    }

    // decompress GZ files
    if (hasGzipMagic(fd)) {
        fd = doGzipLoad(fd);
        if (fd < 0) {
            return NULL;
        }
    }

    // find the length of the file by seeking to the end
    off_t off = lseek(fd, 0, SEEK_END);
    fatal_if(off < 0,
             "Failed to determine size of object file %s\n", fname);
    auto len = static_cast<size_t>(off);

    // mmap the whole shebang
    uint8_t *file_data = (uint8_t *)mmap(NULL, len, PROT_READ, MAP_SHARED,
                                         fd, 0);
    close(fd);

    if (file_data == MAP_FAILED) {
        return NULL;
    }

    ObjectFile *file_obj = NULL;

    // figure out what we have here
    if ((file_obj = ElfObject::tryFile(fname, len, file_data)) != NULL) {
        return file_obj;
    }

    if ((file_obj = EcoffObject::tryFile(fname, len, file_data)) != NULL) {
        return file_obj;
    }

    if ((file_obj = AoutObject::tryFile(fname, len, file_data)) != NULL) {
        return file_obj;
    }

    if ((file_obj = DtbObject::tryFile(fname, len, file_data)) != NULL) {
        return file_obj;
    }

    if (raw)
        return RawObject::tryFile(fname, len, file_data);

    // don't know what it is
    munmap((char*)file_data, len);
    return NULL;
}
