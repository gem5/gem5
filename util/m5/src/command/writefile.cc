/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include <errno.h>

#include <cstring>
#include <fstream>
#include <iostream>

#include "args.hh"
#include "command.hh"
#include "dispatch_table.hh"

namespace
{

void
write_file(const DispatchTable &dt, const std::string &filename,
           const std::string &host_filename)
{
    std::cout << "Opening \"" << filename << "\"." << std::endl;
    std::ifstream src(filename, std::ios_base::in | std::ios_base::binary);

    if (!src) {
        std::cerr << "Error opening \"" << filename << "\": " <<
            strerror(errno) << std::endl;
        exit(2);
    }

    char buf[256 * 1024];
    int offset = 0;

    memset(buf, 0, sizeof(buf));

    while (true) {
        src.seekg(offset);
        src.read(buf, sizeof(buf));
        int len = src.gcount();
        if (!src && !src.eof()) {
            std::cerr << "Error reading \"" << filename << "\": " <<
                strerror(errno) << std::endl;
            exit(2);
        }
        char *wbuf = buf;
        while (len) {
            int bytes = (*dt.m5_write_file)(
                    wbuf, len, offset, host_filename.c_str());
            len -= bytes;
            offset += bytes;
            wbuf += bytes;
        }
        if (src.eof())
            break;
    }
    std::cout << "Wrote " << offset << " bytes." << std::endl;
}

bool
do_write_file(const DispatchTable &dt, Args &args)
{
    const std::string &filename = args.pop();
    const std::string &host_filename = args.pop(filename);

    write_file(dt, filename, host_filename);

    return true;
}

Command write_file_cmd = {
    "writefile", 1, 2, do_write_file, "<filename> [host filename]\n"
        "        Write a file to the host, optionally with a different "
            "name" };

} // anonymous namespace
