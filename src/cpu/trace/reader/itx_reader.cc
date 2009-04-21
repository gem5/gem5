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
 * Declaration of a Intel ITX memory trace format reader.
 */
#include <sstream>

#include "base/misc.hh" // for fatal
#include "cpu/trace/reader/itx_reader.hh"
#include "params/ITXReader.hh"

using namespace std;

ITXReader::ITXReader(const string &name, const string &filename)
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
    traceFormat = 0;
    int c;
    for (int i = 0; i < 4; ++i) {
        c = getc(trace);
        if (c == EOF) {
            fatal("Unexpected end of trace file.");
        }
        traceFormat |= (c & 0xff) << (8 * i);
    }
    if (traceFormat > 2)
        fatal("Invalid trace format.");
}

Tick
ITXReader::getNextReq(MemReqPtr &req)
{
    MemReqPtr tmp_req = new MemReq();
    bool phys_val;
    do {
        int c = getc(trace);
        if (c != EOF) {
            // Decode first byte
            // phys_val<1> | type <2:0> | size <3:0>
            phys_val = c & 0x80;
            tmp_req->size = (c & 0x0f) + 1;
            int type = (c & 0x70) >> 4;

            // Could be a compressed instruction entry, expand if necessary
            if (type == ITXCodeComp) {
                if (traceFormat != 2) {
                    fatal("Compressed code entry in non CompCode trace.");
                }
                if (!codeVirtValid) {
                    fatal("Corrupt CodeComp entry.");
                }

                tmp_req->vaddr = codeVirtAddr;
                codeVirtAddr += tmp_req->size;
                if (phys_val) {
                    if (!codePhysValid) {
                        fatal("Corrupt CodeComp entry.");
                    }
                    tmp_req->paddr = codePhysAddr;
                    if (((tmp_req->paddr & 0xfff) + tmp_req->size) & ~0xfff) {
                        // Crossed page boundary, next physical address is
                        // invalid
                        codePhysValid = false;
                    } else {
                        codePhysAddr += tmp_req->size;
                    }
                    assert(tmp_req->paddr >> 36 == 0);
                } else {
                    codePhysValid = false;
                }
                type = ITXCode;
                tmp_req->cmd = Read;
            } else {
                // Normal entry
                tmp_req->vaddr = 0;
                for (int i = 0; i < 4; ++i) {
                    c = getc(trace);
                    if (c == EOF) {
                        fatal("Unexpected end of trace file.");
                    }
                    tmp_req->vaddr |= (c & 0xff) << (8 * i);
                }
                if (type == ITXCode) {
                    codeVirtAddr = tmp_req->vaddr + tmp_req->size;
                    codeVirtValid = true;
                }
                tmp_req->paddr = 0;
                if (phys_val) {
                    c = getc(trace);
                    if (c == EOF) {
                        fatal("Unexpected end of trace file.");
                    }
                    // Get the page offset from the virtual address.
                    tmp_req->paddr = tmp_req->vaddr & 0xfff;
                    tmp_req->paddr |= (c & 0xf0) << 8;
                    tmp_req->paddr |= (Addr)(c & 0x0f) << 32;
                    for (int i = 2; i < 4; ++i) {
                        c = getc(trace);
                        if (c == EOF) {
                            fatal("Unexpected end of trace file.");
                        }
                        tmp_req->paddr |= (Addr)(c & 0xff) << (8 * i);
                    }
                    if (type == ITXCode) {
                        if (((tmp_req->paddr & 0xfff) + tmp_req->size)
                            & ~0xfff) {
                            // Crossing the page boundary, next physical
                            // address isn't valid
                            codePhysValid = false;
                        } else {
                            codePhysAddr = tmp_req->paddr + tmp_req->size;
                            codePhysValid = true;
                        }
                    }
                    assert(tmp_req->paddr >> 36 == 0);
                } else if (type == ITXCode) {
                    codePhysValid = false;
                }
                switch(type) {
                  case ITXRead:
                    tmp_req->cmd = Read;
                    break;
                  case ITXWrite:
                    tmp_req->cmd = Write;
                    break;
                  case ITXWriteback:
                    tmp_req->cmd = Writeback;
                    break;
                  case ITXCode:
                    tmp_req->cmd = Read;
                    tmp_req->flags |= INST_FETCH;
                    break;
                  default:
                    fatal("Unknown ITX type");
                }
            }
        } else {
            // EOF need to return a null request
            MemReqPtr null_req;
            req = null_req;
            return 0;
        }
    } while (!phys_val);
    req = tmp_req;
    assert(!req || (req->paddr >> 36) == 0);
    return 0;
}

ITXReader *
ITXReaderParams::create()
{
    return new ITXReader(name, filename);
}
