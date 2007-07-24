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
 * Declaration of a memory trace reader for a M5 memory trace.
 */

#include "cpu/trace/reader/m5_reader.hh"
#include "mem/trace/m5_format.hh"
#include "mem/mem_cmd.hh"
#include "params/M5Reader.hh"

using namespace std;

M5Reader::M5Reader(const string &name, const string &filename)
    : MemTraceReader(name)
{
    traceFile.open(filename.c_str(), ios::binary);
}

Tick
M5Reader::getNextReq(MemReqPtr &req)
{
    M5Format ref;

    MemReqPtr tmp_req;
    // Need to read EOF char before eof() will return true.
    traceFile.read((char*) &ref, sizeof(ref));
    if (!traceFile.eof()) {
        //traceFile.read((char*) &ref, sizeof(ref));
#ifndef NDEBUG
        int gcount = traceFile.gcount();
        assert(gcount != 0 || traceFile.eof());
        assert(gcount == sizeof(ref));
        assert(ref.cmd < 12);
#endif
        tmp_req = new MemReq();
        tmp_req->paddr = ref.paddr;
        tmp_req->asid = ref.asid;
        // Assume asid == thread_num
        tmp_req->thread_num = ref.asid;
        tmp_req->cmd = (MemCmdEnum)ref.cmd;
        tmp_req->size = ref.size;
        tmp_req->dest = ref.dest;
    } else {
        ref.cycle = 0;
    }
    req = tmp_req;
    return ref.cycle;
}

M5Reader *
M5ReaderParams::create()
{
    return new M5Reader(name, filename);
}
