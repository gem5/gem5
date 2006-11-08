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
 *
 * Authors: Nathan Binkert
 */

#include "arch/sparc/arguments.hh"
#include "arch/sparc/vtophys.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"

using namespace SparcISA;

Arguments::Data::~Data()
{
    while (!data.empty()) {
        delete [] data.front();
        data.pop_front();
    }
}

char *
Arguments::Data::alloc(size_t size)
{
    char *buf = new char[size];
    data.push_back(buf);
    return buf;
}

uint64_t
Arguments::getArg(bool fp)
{
    //The caller uses %o0-%05 for the first 6 arguments even if their floating
    //point. Double precision floating point values take two registers/args.
    //Quads, structs, and unions are passed as pointers. All arguments beyond
    //the sixth are passed on the stack past the 16 word window save area,
    //space for the struct/union return pointer, and space reserved for the
    //first 6 arguments which the caller may use but doesn't have to.
    if (number < 6) {
        return tc->readIntReg(8 + number);
    } else {
        Addr sp = tc->readIntReg(14);
        VirtualPort *vp = tc->getVirtPort(tc);
        uint64_t arg = vp->read<uint64_t>(sp + 92 + (number-6) * sizeof(uint64_t));
        tc->delVirtPort(vp);
        return arg;
    }
}

