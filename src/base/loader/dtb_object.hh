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

#ifndef __DTB_OBJECT_HH__
#define __DTB_OBJECT_HH__

#include "base/loader/object_file.hh"

/** @file
 * This implements an object file format to support loading
 * and modifying flattened device tree blobs for use with
 * current and future ARM Linux kernels.
 */
class DtbObject : public ObjectFile
{
    protected:
        DtbObject(const std::string &_filename, size_t _len, uint8_t *_data,
                  Arch _arch, OpSys _opSys);

        /** Bool marking if this dtb file has replaced the original
         *  read in DTB file with a new modified buffer
         */
        bool fileDataMmapped;

    public:
        virtual ~DtbObject();

        /** Adds the passed in Command Line options for the kernel
          * to the proper location in the device tree.
          * @param _args command line to append
          * @param len length of the command line string
          * @return returns true on success, false otherwise
          */
        bool addBootCmdLine(const char* _args, size_t len);

        /** Parse the DTB file enough to find the provided release
         * address and return it.
         * @return release address for SMP boot
         */
        Addr findReleaseAddr();

        bool loadAllSymbols(SymbolTable *symtab, Addr base = 0,
                            Addr offset = 0, Addr addrMask = maxAddr);
        bool loadGlobalSymbols(SymbolTable *symtab, Addr base = 0,
                               Addr offset = 0, Addr addrMask = maxAddr);
        bool loadLocalSymbols(SymbolTable *symtab, Addr base = 0,
                              Addr offset = 0, Addr addrMask = maxAddr);

        /** Static function that tries to load file as a
          * flattened device tree blob.
          * @param fname path to file
          * @param len length of file
          * @param data mmap'ed data buffer containing file contents
          * @return ObjectFile representing closest match of file type
          */
        static ObjectFile *tryFile(const std::string &fname,
                                   size_t len, uint8_t *data);
};

#endif //__DTB_OBJECT_HH__
