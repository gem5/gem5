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

#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <list>
#include <string>

#include "base/loader/aout_object.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/ecoff_object.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/raw_object.hh"
#include "base/loader/symtab.hh"
#include "base/cprintf.hh"
#include "mem/port_proxy.hh"

using namespace std;

ObjectFile::ObjectFile(const string &_filename, int _fd,
                       size_t _len, uint8_t *_data,
                       Arch _arch, OpSys _opSys)
    : filename(_filename), descriptor(_fd), fileData(_data), len(_len),
      arch(_arch), opSys(_opSys), globalPtr(0)
{
}


ObjectFile::~ObjectFile()
{
    close();
}


bool
ObjectFile::loadSection(Section *sec, PortProxy& memProxy, Addr addrMask, Addr offset)
{
    if (sec->size != 0) {
        Addr addr = (sec->baseAddr & addrMask) + offset;
        if (sec->fileImage) {
            memProxy.writeBlob(addr, sec->fileImage, sec->size);
        }
        else {
            // no image: must be bss
            memProxy.memsetBlob(addr, 0, sec->size);
        }
    }
    return true;
}


bool
ObjectFile::loadSections(PortProxy& memProxy, Addr addrMask, Addr offset)
{
    return (loadSection(&text, memProxy, addrMask, offset)
            && loadSection(&data, memProxy, addrMask, offset)
            && loadSection(&bss, memProxy, addrMask, offset));
}


void
ObjectFile::close()
{
    if (descriptor >= 0) {
        ::close(descriptor);
        descriptor = -1;
    }

    if (fileData) {
        ::munmap((char*)fileData, len);
        fileData = NULL;
    }
}


ObjectFile *
createObjectFile(const string &fname, bool raw)
{
    // open the file
    int fd = open(fname.c_str(), O_RDONLY);
    if (fd < 0) {
        return NULL;
    }

    // find the length of the file by seeking to the end
    size_t len = (size_t)lseek(fd, 0, SEEK_END);

    // mmap the whole shebang
    uint8_t *fileData =
        (uint8_t *)mmap(NULL, len, PROT_READ, MAP_SHARED, fd, 0);
    if (fileData == MAP_FAILED) {
        close(fd);
        return NULL;
    }

    ObjectFile *fileObj = NULL;

    // figure out what we have here
    if ((fileObj = EcoffObject::tryFile(fname, fd, len, fileData)) != NULL) {
        return fileObj;
    }

    if ((fileObj = AoutObject::tryFile(fname, fd, len, fileData)) != NULL) {
        return fileObj;
    }

    if ((fileObj = ElfObject::tryFile(fname, fd, len, fileData)) != NULL) {
        return fileObj;
    }

    if ((fileObj = DtbObject::tryFile(fname, fd, len, fileData)) != NULL) {
        return fileObj;
    }

    if (raw)
        return RawObject::tryFile(fname, fd, len, fileData);

    // don't know what it is
    close(fd);
    munmap((char*)fileData, len);
    return NULL;
}
