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
 *
 * Authors: Anthony Gutierrez
 */

#include <sys/mman.h>
#include <err.h>
#include <unistd.h>

#include <cassert>

#include "base/loader/dtb_object.hh"
#include "sim/byteswap.hh"
#include "fdt.h"
#include "libfdt.h"

ObjectFile *
DtbObject::tryFile(const std::string &fname, size_t len, uint8_t *data)
{
    // Check if this is a FDT file by looking for magic number
    if (fdt_magic((void*)data) == FDT_MAGIC) {
        return new DtbObject(fname, len, data,
                             ObjectFile::UnknownArch, ObjectFile::UnknownOpSys);
    } else {
        return NULL;
    }
}

DtbObject::DtbObject(const std::string &_filename, size_t _len, uint8_t *_data,
                     Arch _arch, OpSys _opSys)
    : ObjectFile(_filename, _len, _data, _arch, _opSys)
{
    text.baseAddr = 0;
    text.size = len;
    text.fileImage = fileData;

    data.baseAddr = 0;
    data.size = 0;
    data.fileImage = NULL;

    bss.baseAddr = 0;
    bss.size = 0;
    bss.fileImage = NULL;

    fileDataMmapped = true;
}

DtbObject::~DtbObject()
{
    // Make sure to clean up memory properly depending
    // on how buffer was allocated.
    if (fileData && !fileDataMmapped) {
        delete [] fileData;
        fileData = NULL;
    } else if (fileData) {
        munmap(fileData, len);
        fileData = NULL;
    }
}

bool
DtbObject::addBootCmdLine(const char* _args, size_t len)
{
    const char* root_path = "/";
    const char* node_name = "chosen";
    const char* full_path_node_name = "/chosen";
    const char* property_name = "bootargs";

    // Make a new buffer that has extra space to add nodes/properties
    int newLen = 2*this->len;
    uint8_t* fdt_buf_w_space = new uint8_t[newLen];
    // Copy and unpack flattened device tree into new buffer
    int ret = fdt_open_into((void*)fileData, (void*)fdt_buf_w_space, (newLen));
    if (ret < 0) {
        warn("Error resizing buffer of flattened device tree, "
             "errno: %d\n", ret);
        delete [] fdt_buf_w_space;
        return false;
    }

    // First try finding the /chosen node in the dtb
    int offset = fdt_path_offset((void*)fdt_buf_w_space, full_path_node_name);
    if (offset < 0) {
        // try adding the node by walking dtb tree to proper insertion point
        offset = fdt_path_offset((void*)fdt_buf_w_space, root_path);
        offset = fdt_add_subnode((void*)fdt_buf_w_space, offset, node_name);
        // if we successfully add the subnode, get the offset
        if (offset >= 0)
          offset = fdt_path_offset((void*)fdt_buf_w_space, full_path_node_name);

        if (offset < 0) {
            warn("Error finding or adding \"chosen\" subnode to flattened "
                 "device tree, errno: %d\n", offset);
            delete [] fdt_buf_w_space;
            return false;
        }
    }

    // Set the bootargs property in the /chosen node
    ret = fdt_setprop((void*)fdt_buf_w_space, offset, property_name,
                      (const void*)_args, len+1);
    if (ret < 0) {
        warn("Error setting \"bootargs\" property to flattened device tree, "
             "errno: %d\n", ret);
        delete [] fdt_buf_w_space;
        return false;
    }

    // Repack the dtb for kernel use
    ret = fdt_pack((void*)fdt_buf_w_space);
    if (ret < 0) {
        warn("Error re-packing flattened device tree structure, "
             "errno: %d\n", ret);
        delete [] fdt_buf_w_space;
        return false;
    }

    text.size = newLen;
    text.fileImage = fdt_buf_w_space;

    // clean up old buffer and set to new fdt blob
    munmap(fileData, this->len);
    fileData = fdt_buf_w_space;
    fileDataMmapped = false;
    this->len = newLen;

    return true;
}

Addr
DtbObject::findReleaseAddr()
{
    void *fd = (void*)fileData;

    int offset = fdt_path_offset(fd, "/cpus/cpu@0");
    int len;

    const void* temp = fdt_getprop(fd, offset, "cpu-release-addr", &len);
    Addr rel_addr = 0;

    if (len > 3)
        rel_addr = betoh(*static_cast<const uint32_t*>(temp));
    if (len == 8)
        rel_addr = (rel_addr << 32) | betoh(*(static_cast<const uint32_t*>(temp)+1));

    return rel_addr;
}

bool
DtbObject::loadAllSymbols(SymbolTable *symtab, Addr base, Addr offset,
                          Addr addr_mask)
{
    return false;
}

bool
DtbObject::loadGlobalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                             Addr addr_mask)
{
    // nothing to do here
    return false;
}

bool
DtbObject::loadLocalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                            Addr addr_mask)
{
    // nothing to do here
    return false;
}
