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
 * Definition of a Intel ITX memory trace format reader.
 */

#ifndef __ITX_READER_HH__
#define __ITX_READER_HH__

#include <cstdio>
#include <string>

#include "cpu/trace/reader/mem_trace_reader.hh"
#include "mem/mem_req.hh"

/**
 * A memory trace reader for the Intel ITX memory trace format.
 */
class ITXReader : public MemTraceReader
{
  private:
    /** Trace file. */
    FILE *trace;

    bool codeVirtValid;
    Addr codeVirtAddr;
    bool codePhysValid;
    Addr codePhysAddr;

    int traceFormat;

    enum ITXType {
        ITXRead,
        ITXWrite,
        ITXWriteback,
        ITXCode,
        ITXCodeComp
    };

  public:
    /**
     * Construct an ITXReader.
     */
    ITXReader(const std::string &name, const std::string &filename);

    /**
     * Read the next request from the trace. Returns the request in the
     * provided MemReqPtr and the cycle of the request in the return value.
     * @param req Return the next request from the trace.
     * @return ITX traces don't store timing information, return 0
     */
    virtual Tick getNextReq(MemReqPtr &req);
};

#endif //__ITX_READER_HH__

