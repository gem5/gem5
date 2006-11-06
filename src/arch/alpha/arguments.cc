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

#include "arch/alpha/arguments.hh"
#include "arch/alpha/vtophys.hh"
#include "cpu/thread_context.hh"
#include "mem/vport.hh"

using namespace AlphaISA;

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
    if (number < 6) {
        if (fp)
            return tc->readFloatRegBits(16 + number);
        else
            return tc->readIntReg(16 + number);
    } else {
        Addr sp = tc->readIntReg(30);
        VirtualPort *vp = tc->getVirtPort(tc);
        uint64_t arg = vp->read<uint64_t>(sp + (number-6) * sizeof(uint64_t));
        tc->delVirtPort(vp);
        return arg;
    }
}

