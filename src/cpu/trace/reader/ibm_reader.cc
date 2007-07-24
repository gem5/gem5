/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declaration of a IBM memory trace format reader.
 */
#include <sstream>

#include "base/misc.hh" // for fatal
#include "cpu/trace/reader/ibm_reader.hh"
#include "params/IBMReader.hh"

using namespace std;

IBMReader::IBMReader(const string &name, const string &filename)
    : MemTraceReader(name)
{
    if (strcmp((filename.c_str() + filename.length() -3), ".gz") == 0) {
        // Compressed file, need to use a pipe to gzip.
        stringstream buf;
        buf << "gzip -d -c " << filename << endl;
        trace = popen(buf.str().c_str(), "r");
    } else {
        trace = fopen(filename.c_str(), "rb");
    }
    if (!trace) {
        fatal("Can't open file %s", filename);
    }
}

Tick
IBMReader::getNextReq(MemReqPtr &req)
{
    MemReqPtr tmp_req;

    int c = getc(trace);
    if (c != EOF) {
        tmp_req = new MemReq();
        //int cpu_id = (c & 0xf0) >> 4;
        int type = c & 0x0f;
        // We have L1 miss traces, so all accesses are 128 bytes
        tmp_req->size = 128;

        tmp_req->paddr = 0;
        for (int i = 2; i >= 0; --i) {
            c = getc(trace);
            if (c == EOF) {
                fatal("Unexpected end of file");
            }
            tmp_req->paddr |= ((c & 0xff) << (8 * i));
        }
        tmp_req->paddr = tmp_req->paddr << 7;

        switch(type) {
          case IBM_COND_EXCLUSIVE_FETCH:
          case IBM_READ_ONLY_FETCH:
            tmp_req->cmd = Read;
            break;
          case IBM_EXCLUSIVE_FETCH:
          case IBM_FETCH_NO_DATA:
            tmp_req->cmd = Write;
            break;
          case IBM_INST_FETCH:
            tmp_req->cmd = Read;
            break;
          default:
            fatal("Unknown trace entry type.");
        }

    }
    req = tmp_req;
    return 0;
}

IBMReader *
IBMReaderParams::create()
{
    return new IBMReader(name, filename);
}
